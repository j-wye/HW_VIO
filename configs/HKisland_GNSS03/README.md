# HKisland_GNSS03

> ## DISABLED -- not in the evaluation set

391 s, 3911 images, 1.7 km path, AGL 86 m, cruise **9.0 m/s**, Hong Kong.

Held out of the evaluation set by default; promoting it is a project decision. It was flown 2023-10-24, ONE DAY before HKairport_GNSS03 on the same hardware, and that is the flight disqualified for a tau that swings 198 ms mid-flight -- so it got the strictest check of any sequence here: 8 windows over 90-330 s giving -0.0866/-0.0927/-0.0957/-0.0942/-0.0893/-0.0935/-0.0933/-0.0905. Spread 9.1 ms, std 3.0 ms, and the linear trend over the whole span is only -1.6 ms, i.e. SMALLER than the 2.9 ms residual scatter: noise, not the monotone/step drift HKairport shows. All eight gains sit in -0.95..-1.08. Adopted -0.092 (R^2-weighted), which equals HKisland03 exactly -- same camera/IMU pair, independent flights. Carries the full /ublox_driver/* suite, so receiver_lla (10 Hz real GNSS) is in the converted bag; its header stamps are GPS time, measured +18.000 s ahead (-18 s minimises error against RTK: 59.11 -> 4.72 m), so the converter writes it with the ROS1 record time instead (USE_RECORD_TIME). Same route as HKisland03 flown ~11 months apart (path 1726 vs 1719 m, cruise 8.96 vs 8.96 m/s, extent 280x435 m both), so the pair doubles as a repeatability check.

| | |
|---|---|
| rosbag2 | `datasets/HKisland_GNSS03/` |
| config | `configs/HKisland_GNSS03/config.yaml` (derived from the official calibration) |
| official calib group | `HK_GNSS` (official MARS-LVIG calibration) |
| image | `/camera/image_raw` 640x480 bgr8 @10 Hz |
| IMU | `/imu/data` ~208 Hz, already in m/s^2 |
| height | `/height` `sensor_msgs/Range` @10 Hz (down-looking LiDAR, baro surrogate) |
| ground truth | `/ground_truth/fix` @5 Hz (+ `/ground_truth/velocity_ned`, `/ground_truth/yaw_raw`) |
| timeshift_cam_imu | `-0.092` s (measured) |

