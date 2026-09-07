# AMtown03

620 s, 6199 images, 4.8 km path, AGL 80 m, cruise **11.9 m/s**, Armenia.

Primary accuracy sequence. Its speed-ablation partner AMtown01 (4.0 m/s, same route/altitude/day) has been retired, so a controlled speed comparison now has to come from HKisland03 (9.0 m/s) and AMvalley03 (12.0 m/s) instead -- different sites, so a weaker control than the AMtown pair was.

| | |
|---|---|
| rosbag2 | `datasets/AMtown03/` |
| config | `configs/AMtown03/config.yaml` (derived from the official calibration) |
| official calib group | `AMtown` (official MARS-LVIG calibration) |
| image | `/camera/image_raw` 640x480 bgr8 @10 Hz |
| IMU | `/imu/data` ~208 Hz, already in m/s^2 |
| height | `/height` `sensor_msgs/Range` @10 Hz (down-looking LiDAR, baro surrogate) |
| ground truth | `/ground_truth/fix` @5 Hz (+ `/ground_truth/velocity_ned`, `/ground_truth/yaw_raw`) |
| timeshift_cam_imu | `-0.067` s (measured) |

## 설정

고정 변수 세트를 그대로 쓴다 (`configs/README.md`). 바꾼 값 없음.

이 파일로 측정한 값: SE3 25.09 m, 60 s 윈도우 10.89 / 20.96 / 31.76 m (중앙값 / p90 / 최악), 1 s drift 1.199 m, 주입 7.17 Hz.

게이트를 끄고 같은 파일로 돌리면 SE3 47.75 m, 1 s drift 1.666 m — 게이트가 오차를 절반으로 줄인다.
