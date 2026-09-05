# AMvalley03

546 s, 5460 images, 4.2 km path, AGL 85 m (valley), cruise **12.0 m/s**, Armenia.

In the evaluation set since 2026-09-06. tau IS constant here: measured over 4 windows (60/180/300/420 s) at -0.0671/-0.0663/-0.0628/-0.0659, spread 4.3 ms, against the 198 ms swing that disqualified HKairport_GNSS03. Value adopted = -0.066 (R^2-weighted), which agrees to 1 ms with the -0.067 of AMtown01/03 -- same platform, same field campaign one day earlier -- so the two independent measurements corroborate each other. The two low-R^2 windows are the straight-cruise ones with little roll excitation, not a tau shift. Terrain relief is large (/height vs RTK relative altitude: corr 0.693, residual std 30.5 m, vs 5.7-10.5 m for the AMtown/HKairport bags) -- that is a valley, not a broken sensor, but it makes /height a poor z anchor and a good depth prior here. Cruise 12.0 m/s duplicates AMtown03 (11.9 m/s), so its worth is unseen-terrain generalisation, not a new speed point.

| | |
|---|---|
| rosbag2 | `datasets/AMvalley03/` |
| config | `configs/AMvalley03/config.yaml` (derived from the official calibration) |
| official calib group | `AMvalley` (official MARS-LVIG calibration) |
| image | `/camera/image_raw` 640x480 bgr8 @10 Hz |
| IMU | `/imu/data` ~208 Hz, already in m/s^2 |
| height | `/height` `sensor_msgs/Range` @10 Hz (down-looking LiDAR, baro surrogate) |
| ground truth | `/ground_truth/fix` @5 Hz (+ `/ground_truth/velocity_ned`, `/ground_truth/yaw_raw`) |
| timeshift_cam_imu | `-0.066` s (measured) |

