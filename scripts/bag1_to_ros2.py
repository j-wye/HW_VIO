#!/usr/bin/env python3
"""ROS1 MARS-LVIG .bag  ->  Humble-native rosbag2, in ONE pass.

Does decode + crop + resize + topic-filter + IMU unit fix + lidar->height
distillation together, so the 356 GB full-resolution raw intermediate the old
multi-step pipeline produced is never written.

Naming convention (user-set): the original ROS1 file is renamed to
<Name>_original.bag and the converted rosbag2 directory takes the original
name, e.g.  datasets/AMtown03_original.bag -> datasets/AMtown03/ .

Image (must match the intrinsics in configs/*/config.yaml exactly):
    2448x2048 is 153:128, NOT 4:3, so a direct resize to 640x480 would be
    anamorphic and would silently invalidate the radtan model. Instead:
      crop 106 rows off the top AND bottom  -> 2448x1836  (exactly 4:3)
      uniform scale 40/153                  -> 640x480    (exact integers)
    (Width cannot be cropped to reach 4:3: that needs 2731 px > 2448.)
    Output stays bgr8 (colour): MSCEqF converts to mono8 internally via
    cv_bridge, and colour keeps the stream usable for visualisation.

IMU: /livox/imu linear_acceleration is published in **g** (measured 1.0028 /
1.0051 / 0.9972 at rest across our three bags) which violates sensor_msgs/Imu.
MSCEqF assumes m/s^2 (gravity_ is documented in m/s^2 and the static
initializer computes b0 = acc_mean - R0^T (gravity*z)), so we scale by 9.80665
here, once, in the bag itself.

Height: /livox/lidar is a 3D scanning point cloud (livox_ros_driver/CustomMsg,
72k pts/frame) from the nadir-mounted Avia, and it is most of the bag volume.
We do not need the cloud -- we distill it to a 1D ground distance: the lidar
x-axis points down (verified: at rest the built-in IMU reads (-1,0,0) g), and
median(x of valid points) tracks RTK relative altitude with corr 0.74-0.97,
+-10 m differences being real terrain relief. Published as sensor_msgs/Range
on **/height** (the barometer/altimeter surrogate; height_above_takeoff is
broken in these bags). On the ground every return is below the Avia minimum
range -> all-zero points -> no /height message (a real altimeter would do the
same).

    python3 scripts/bag1_to_ros2.py --src datasets/AMtown03_original.bag --dst datasets/AMtown03
"""
import argparse
import os
import struct
import sys

import numpy as np

CROP = 106
OUT_W, OUT_H = 640, 480
G = 9.80665

# Livox CustomPoint wire format: uint32 offset_time, float32 xyz, u8 refl/tag/line
LIVOX_PT = np.dtype([('ot', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                     ('refl', 'u1'), ('tag', 'u1'), ('line', 'u1')])
HEIGHT_MIN_VALID = 500          # frames with fewer valid returns publish nothing

KEEP_IMAGE = '/left_camera/image/compressed'
KEEP_IMU = '/livox/imu'          # the IMU we use (camera extrinsic derivable)
KEEP_LIDAR = '/livox/lidar'      # distilled to /height, cloud itself dropped
# Dropped: /dji_osdk_ros/imu (no published camera extrinsic -> unusable),
# /dji_osdk_ros/height_above_takeoff (measured broken: stays ~0 over an 80 m
# climb, anti-correlates with RTK), attitude/rc/battery/status/... everything else.
# GT topics are re-serialized properly (ROS1 wire format != ROS2 CDR: ROS1 Header
# carries `seq` and CDR adds a 4-byte encapsulation, so a raw byte passthrough
# produces bags that ros2 cannot deserialize -- verified broken, hence this table).
KEEP_CONVERT = {
    '/dji_osdk_ros/rtk_position': 'sensor_msgs/msg/NavSatFix',   # ground truth
    '/dji_osdk_ros/rtk_velocity': 'geometry_msgs/msg/Vector3Stamped',
    '/dji_osdk_ros/rtk_yaw': 'std_msgs/msg/Int16',
    # !! receiver_lla header stamps are on GPS TIME -- 18 s (2023 leap offset) ahead of
    # every other topic. Measured: shifting by -18 s drops the median position error vs
    # RTK from 84.79 m to 7.46 m. We therefore write it with the ROS1 record time (host
    # clock) instead of header.stamp; see USE_RECORD_TIME below.
    '/ublox_driver/receiver_lla': 'sensor_msgs/msg/NavSatFix',   # HKairport only
    # Real (non-RTK) GNSS from the DJI receiver. HKisland03/AMvalley03 carry no
    # /ublox_driver/* at all, so this is the only actual GNSS stream in them --
    # task A (fusion) otherwise has to simulate GNSS from the RTK ground truth,
    # which makes the anchor and the scoring reference the same signal.
    '/dji_osdk_ros/gps_position': 'sensor_msgs/msg/NavSatFix',
    # Flight-controller attitude. NOT used to pre-correct /height: /height stays
    # the raw slant range because its two consumers want opposite things (a
    # feature depth prior wants the slant, a z anchor wants the vertical), and
    # baking a correction in would be irreversible once the source bag is gone.
    # Measured tilt: median 6.9/5.0 deg, p90 16.7/15.0 deg -> 0.6/3.7 m of slant
    # error at 85 m AGL. Recorded so the correction stays possible, in analysis.
    # NB this is the FC's fused estimate, not RTK-grade truth: never score VIO
    # attitude against it.
    '/dji_osdk_ros/attitude': 'geometry_msgs/msg/QuaternionStamped',
}
# Topics whose header.stamp is on a different clock -> use the ROS1 record time instead.
USE_RECORD_TIME = {'/ublox_driver/receiver_lla'}

# Output names. The source namespaces (/dji_osdk_ros/..., /ublox_driver/...) name the
# dataset authors' ROS drivers, not what the topic is to us; write the useful ones under
# names that say what they are. The camera, IMU and rtk_position keep their original names on
# purpose: they are compiled into src/run_offline.cpp, src/run_feeder.cpp and scripts/eval_vio.py,
# and the whole measured accuracy record was produced through them.
OUT_NAME = {
    '/left_camera/image/compressed': '/camera/image_raw',
    '/livox/imu': '/imu/data',
    '/dji_osdk_ros/rtk_position': '/ground_truth/fix',
    '/dji_osdk_ros/rtk_velocity': '/ground_truth/velocity_ned',
    '/dji_osdk_ros/rtk_yaw': '/ground_truth/yaw_raw',
    '/ublox_driver/receiver_lla': '/gnss/ublox_spp',   # independent u-blox SPP, 4.69 m vs RTK
    '/dji_osdk_ros/gps_position': '/gnss/dji_rtk_aided',  # a delayed copy of the RTK solution
    '/dji_osdk_ros/attitude': '/fc/attitude_enu',      # fused FC estimate, body_FLU -> ENU
}


def parse_livox(raw):
    """CustomMsg bytes -> (stamp_sec_float, structured point array). No copies."""
    o = 12                                    # seq + stamp
    sec, nsec = struct.unpack_from('<II', raw, 4)
    flen, = struct.unpack_from('<I', raw, o); o += 4 + flen
    o += 12                                   # timebase(u64) + point_num(u32)
    o += 4                                    # lidar_id(u8) + rsvd[3]
    alen, = struct.unpack_from('<I', raw, o); o += 4
    return sec + nsec * 1e-9, np.frombuffer(raw, dtype=LIVOX_PT, count=alen, offset=o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True)
    ap.add_argument('--dst', required=True)
    ap.add_argument('--start', type=float, default=0.0, help='seconds after bag start')
    ap.add_argument('--duration', type=float, default=None)
    ap.add_argument('--image-topic', default=None,
                    help='override the image output topic (default: OUT_NAME mapping)')
    args = ap.parse_args()

    if os.path.exists(args.dst):
        sys.exit(f"refusing to overwrite existing {args.dst}")

    import cv2
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image, Imu, Range
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.dst, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    declared = {}

    def declare(name, typ):
        if name not in declared:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=name, type=typ, serialization_format='cdr'))
            declared[name] = typ

    img_out = args.image_topic or OUT_NAME[KEEP_IMAGE]
    imu_out = OUT_NAME[KEEP_IMU]
    declare(img_out, 'sensor_msgs/msg/Image')
    declare(imu_out, 'sensor_msgs/msg/Imu')
    declare('/height', 'sensor_msgs/msg/Range')

    from sensor_msgs.msg import NavSatFix
    from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
    from std_msgs.msg import Int16

    def cvt_navsatfix(m):
        out = NavSatFix()
        out.header.stamp.sec = int(m.header.stamp.sec)
        out.header.stamp.nanosec = int(m.header.stamp.nanosec)
        out.header.frame_id = m.header.frame_id
        out.status.status = int(m.status.status)
        out.status.service = int(m.status.service)
        out.latitude, out.longitude, out.altitude = \
            float(m.latitude), float(m.longitude), float(m.altitude)
        out.position_covariance = [float(x) for x in m.position_covariance]
        out.position_covariance_type = int(m.position_covariance_type)
        return out

    def cvt_vector3stamped(m):
        out = Vector3Stamped()
        out.header.stamp.sec = int(m.header.stamp.sec)
        out.header.stamp.nanosec = int(m.header.stamp.nanosec)
        out.header.frame_id = m.header.frame_id
        out.vector.x, out.vector.y, out.vector.z = \
            float(m.vector.x), float(m.vector.y), float(m.vector.z)
        return out

    def cvt_int16(m):
        out = Int16()
        out.data = int(m.data)
        return out

    def cvt_quaternionstamped(m):
        out = QuaternionStamped()
        out.header.stamp.sec = int(m.header.stamp.sec)
        out.header.stamp.nanosec = int(m.header.stamp.nanosec)
        out.header.frame_id = m.header.frame_id
        out.quaternion.w, out.quaternion.x, out.quaternion.y, out.quaternion.z = \
            float(m.quaternion.w), float(m.quaternion.x), \
            float(m.quaternion.y), float(m.quaternion.z)
        return out

    CVT = {'sensor_msgs/msg/NavSatFix': cvt_navsatfix,
           'geometry_msgs/msg/Vector3Stamped': cvt_vector3stamped,
           'geometry_msgs/msg/QuaternionStamped': cvt_quaternionstamped,
           'std_msgs/msg/Int16': cvt_int16}

    n_img = n_imu = n_hgt = n_pass = 0
    with Reader(args.src) as r:
        t0 = r.start_time / 1e9
        want = [KEEP_IMAGE, KEEP_IMU, KEEP_LIDAR] + list(KEEP_CONVERT)
        conns = [c for c in r.connections if c.topic in want]
        print(f"[in ] {args.src}")
        print(f"[keep] {sorted({c.topic for c in conns})}")
        for c in conns:
            if c.topic in KEEP_CONVERT:
                declare(OUT_NAME.get(c.topic, c.topic), KEEP_CONVERT[c.topic])

        for conn, rec, raw in r.messages(connections=conns):
            rel = rec / 1e9 - t0
            if rel < args.start:
                continue
            if args.duration is not None and rel > args.start + args.duration:
                break
            # rosbag2 record timestamp: use header.stamp, NOT the ROS1 record time.
            # The livox driver batches messages, so record dt is bursty (median
            # 0.012 ms with 183 ms gaps) while header dt is a uniform 4.976 ms.
            # `ros2 bag play` paces by this column: with record times a replay
            # floods subscribers in bursts and shallow sensor-QoS queues drop
            # most of each burst (measured 75 Hz received of a 208 Hz stream).
            def hdr_ns(msg):
                return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

            if conn.topic == KEEP_IMAGE:
                m = ts.deserialize_ros1(raw, conn.msgtype)
                im = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
                if im is None:
                    continue
                h = im.shape[0] - 2 * CROP
                im = cv2.resize(im[CROP:CROP + h], (OUT_W, OUT_H),
                                interpolation=cv2.INTER_AREA)
                out = Image()
                out.header.stamp.sec = int(m.header.stamp.sec)
                out.header.stamp.nanosec = int(m.header.stamp.nanosec)
                out.header.frame_id = m.header.frame_id or 'cam0'
                out.height, out.width = OUT_H, OUT_W
                out.encoding, out.is_bigendian, out.step = 'bgr8', 0, OUT_W * 3
                out.data = im.tobytes()
                writer.write(img_out, serialize_message(out), hdr_ns(out))
                n_img += 1

            elif conn.topic == KEEP_IMU:
                m = ts.deserialize_ros1(raw, conn.msgtype)
                out = Imu()
                out.header.stamp.sec = int(m.header.stamp.sec)
                out.header.stamp.nanosec = int(m.header.stamp.nanosec)
                out.header.frame_id = m.header.frame_id or 'imu'
                out.linear_acceleration.x = float(m.linear_acceleration.x) * G
                out.linear_acceleration.y = float(m.linear_acceleration.y) * G
                out.linear_acceleration.z = float(m.linear_acceleration.z) * G
                out.angular_velocity.x = float(m.angular_velocity.x)
                out.angular_velocity.y = float(m.angular_velocity.y)
                out.angular_velocity.z = float(m.angular_velocity.z)
                out.orientation_covariance[0] = -1.0
                writer.write(imu_out, serialize_message(out), hdr_ns(out))
                n_imu += 1

            elif conn.topic == KEEP_LIDAR:
                t_hdr, pts = parse_livox(raw)
                v = pts['x'][pts['x'] > 0.5]        # lidar +x = down; 0 = no return
                if len(v) < HEIGHT_MIN_VALID:
                    continue                        # on the ground: nothing in range
                out = Range()
                out.header.stamp.sec = int(t_hdr)
                out.header.stamp.nanosec = int(round((t_hdr - int(t_hdr)) * 1e9)) % 1_000_000_000
                out.header.frame_id = 'livox'
                out.radiation_type = Range.INFRARED
                out.field_of_view = 1.22            # the ~70 deg cone the median is taken over
                out.min_range, out.max_range = 1.0, 450.0
                out.range = float(np.median(v))
                writer.write('/height', serialize_message(out), hdr_ns(out))
                n_hgt += 1

            else:
                # GT topics: deserialize the ROS1 message and re-serialize as ROS2 CDR
                m = ts.deserialize_ros1(raw, conn.msgtype)
                out = CVT[KEEP_CONVERT[conn.topic]](m)
                # std_msgs/Int16 (rtk_yaw) has no header -> keep the record time.
                # receiver_lla's header is on GPS time -> record time as well.
                use_rec = conn.topic in USE_RECORD_TIME or not hasattr(out, 'header')
                writer.write(OUT_NAME.get(conn.topic, conn.topic), serialize_message(out),
                             rec if use_rec else hdr_ns(out))
                n_pass += 1

            if (n_img + n_imu + n_hgt + n_pass) % 20000 == 0:
                print(f"  ... img={n_img} imu={n_imu} height={n_hgt} other={n_pass}", flush=True)

    del writer
    # header-stamp record times reorder rows w.r.t. write order; the rosbag2
    # player assumes insertion order == time order and stalls otherwise
    # (images measured at 2-7.5 Hz instead of 10). Re-sort the store.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from sort_bag_db3 import sort_bag
    sort_bag(args.dst)
    print(f"[out] {args.dst}  images={n_img} ({OUT_W}x{OUT_H} bgr8)  imu={n_imu}  "
          f"height={n_hgt}  other={n_pass}")


if __name__ == '__main__':
    main()
