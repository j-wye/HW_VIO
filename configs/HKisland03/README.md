# HKisland03

380 s, 3800 images, 1.7 km path, AGL 85 m, cruise **9.0 m/s**, Hong Kong.

In the evaluation set since 2026-09-06. The RTK clock leads the camera/IMU clock here: the 1 s drift is minimised at gt_time_offset +0.35 s (heading-rate/gyro cross-correlation peaks at +0.52 s with a broad maximum), so it is scored with +0.35 s. tau is stable here: measured over 4 windows (110/180/250/310 s) at -0.0915/-0.0962/-0.0933/-0.0899, spread 6.3 ms with all four gains at -1.00 +/- 0.015. Adopted -0.092 (R^2-weighted). Flown 2022-11-29. Adds a 9.0 m/s speed point below AMtown03 (11.9 m/s) over terrain unlike it, and its /height is clean (corr 0.955, residual std 7.8 m).

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

## 설정

고정 변수 세트에서 **`num_clones: 13`** 하나만 바꿨다.

이 시퀀스는 게이트가 약 6 Hz로 주입해서 clone 11개가 9 m/s 비행의 1.8 s밖에 못 덮는다. 두 개를 늘리면 주입률을 올리지 않고 창만 길어진다. SE3 12.17 → 6.84 m, 60 s 윈도우 중앙값 2.95 → 2.64, 1 s drift 0.595 → 0.476

이 파일로 측정한 값: SE3 6.84 m, 60 s 윈도우 2.64 / 4.60 / 4.66 m, 1 s drift 0.476 m, 주입 5.96 Hz.

GT 클럭 오프셋 +0.35 s.
