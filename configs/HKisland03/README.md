# HKisland03

380 s, 3800 images, 1.7 km path, AGL 85 m, cruise **9.0 m/s**, Hong Kong.

In the evaluation set since 2026-09-06. The RTK clock leads the camera/IMU clock by 0.52 s here (cross-correlation of heading rate against the gyro), so it is scored with gt_time_offset +0.52 s. It does NOT inherit the HKairport_GNSS03 defect: tau measured over 4 windows (110/180/250/310 s) at -0.0915/-0.0962/-0.0933/-0.0899, spread 6.3 ms with all four gains at -1.00 +/- 0.015, versus HKairport swinging +0.109 -> -0.089 (198 ms). Adopted -0.092 (R^2-weighted). Different flight and date (2022-11-29) from HKairport_GNSS03 (2023-10-25), so the shared site name is not shared hardware state. Adds a 9.0 m/s speed point below AMtown03 (11.9 m/s) over terrain unlike it, and its /height is clean (corr 0.955, residual std 7.8 m).

| | |
|---|---|
| rosbag2 | `datasets/HKisland03/` |
| config | `configs/HKisland03/config.yaml` (derived from the official calibration) |
| official calib group | `HKisland` (official MARS-LVIG calibration) |
| image | `/camera/image_raw` 640x480 bgr8 @10 Hz |
| IMU | `/imu/data` ~208 Hz, already in m/s^2 |
| height | `/height` `sensor_msgs/Range` @10 Hz (down-looking LiDAR, baro surrogate) |
| ground truth | `/ground_truth/fix` @5 Hz (+ `/ground_truth/velocity_ned`, `/ground_truth/yaw_raw`) |
| timeshift_cam_imu | `-0.092` s (measured) |

