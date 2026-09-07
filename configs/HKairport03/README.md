# HKairport03

> ## DISABLED -- not in the evaluation set

366 s, 3657 images, 1.5 km path, AGL 55 m, cruise **8.6 m/s**, Hong Kong.

DISABLED for quantitative work: tau STEPS mid-flight, the same defect that disqualified HKairport_GNSS03. Measured timeshift: t=80/140/200 -> -0.0900/-0.0914/-0.0829, then t=270/280/295 -> -0.0091 flat. Bracketing it finer, t=250 is -0.0865 and t=270 is -0.0091: a 77 ms STEP between 250 and 270 s that then holds, not scatter. The image stamps themselves are perfectly regular (dt 100.08 ms, std 0.13, zero gaps >150 ms, 3657 frames), so nothing was dropped -- what moved is the relation between the stamp and the actual exposure, i.e. the camera driver re-stamped. No single timeshift_cam_imu can be right, and the full-sequence result shows it: 8.78 % drift, SE3 181.9 m. Segmented, the diagnosis holds: 0-250 s at tau=-0.087 gives 2.30 % drift and SE3 34.7 m (measured at tau=-0.087; this file ships tau=-0.09), while 265-400 s is unusable at either tau (27.75 % at -0.009, 52.84 % at -0.090) -- so the post-step part is damaged beyond the offset. Quantitative use is limited to the first 250 s (run with --duration 250) and the RTK clock leads the camera/IMU clock by 2.72 s there (scored with gt_time_offset -2.72 s).

| | |
|---|---|
| rosbag2 | `datasets/HKairport03/` |
| config | `configs/HKairport03/config.yaml` (derived from the official calibration) |
| official calib group | `HKairport` (official MARS-LVIG calibration) |
| image | `/camera/image_raw` 640x480 bgr8 @10 Hz |
| IMU | `/imu/data` ~208 Hz, already in m/s^2 |
| height | `/height` `sensor_msgs/Range` @10 Hz (down-looking LiDAR, baro surrogate) |
| ground truth | `/ground_truth/fix` @5 Hz (+ `/ground_truth/velocity_ned`, `/ground_truth/yaw_raw`) |
| timeshift_cam_imu | `-0.09` s (measured) |

## 설정

고정 변수 세트에서 **`optical_flow_win_size: 15`** 하나만 바꿨다.

활주로와 계류장은 텍스처가 희박해 21 px 창이 서로 다른 depth를 하나의 flow 벡터로 뭉갠다. SE3 46.44 → 10.48 m, 60 s 윈도우 중앙값 34.02 → 6.84, 1 s drift 3.891 → 0.879

이 파일로 측정한 값: SE3 10.48 m, 60 s 윈도우 6.84 / 10.85 / 11.15 m, 1 s drift 0.879 m, 주입 5.86 Hz (앞 250 s 구간).

**앞 250 s만 쓸 것.** 카메라가 250~270 s에서 타임스탬프를 다시 찍어 그 뒤는 어떤 timeshift로도 맞지 않는다. 측정은 `--duration 250`으로 했다(config 키가 아니라 `run_feeder` 옵션). GT 클럭 오프셋 −2.72 s.
