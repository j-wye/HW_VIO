# HKairport_GNSS03

> ## DISABLED -- not in the evaluation set

397 s, 3971 images, 1.9 km path, AGL 80 m, cruise 7.1 m/s, Hong Kong. Has real GNSS.

DISABLED for quantitative work: the camera<->IMU offset is not constant over this flight (tau = +0.109 s at t=120 s, -0.089 s at t>=240 s, measured with timeshift scan). No single timeshift_cam_imu can be right, so any accuracy number from it would be an artefact of the bag. Keep the data; use it only for qualitative checks until the stamps are diagnosed and re-stamped.

| | |
|---|---|
| rosbag2 | `datasets/HKairport_GNSS03/` |
| config | `configs/HKairport_GNSS03/config.yaml` (derived from the official calibration) |
| official calib group | `HK_GNSS` (official MARS-LVIG calibration) |
| image | `/camera/image_raw` 640x480 bgr8 @10 Hz |
| IMU | `/imu/data` ~208 Hz, already in m/s^2 |
| height | `/height` `sensor_msgs/Range` @10 Hz (down-looking LiDAR, baro surrogate) |
| ground truth | `/ground_truth/fix` @5 Hz (+ `/ground_truth/velocity_ned`, `/ground_truth/yaw_raw`) |
| timeshift_cam_imu | `-0.089` s (measured) |

