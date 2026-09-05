# configs/ — 시퀀스별 실행 단위

폴더 이름 = `datasets/` 아래의 rosbag2 이름.

| 사용 | 시퀀스 | calib 그룹 | timeshift | 제원 |
|---|---|---|---|---|
| ✅ | [`AMtown03`](AMtown03/) | AMtown | -0.067 | 620 s, cruise **11.9 m/s** |
| ✅ | [`AMvalley03`](AMvalley03/) | AMvalley | -0.066 | 546 s, cruise **12.0 m/s** |
| ✅ | [`HKisland03`](HKisland03/) | HKisland | -0.092 | 380 s, cruise **9.0 m/s** |
| ⛔ | [`HKairport03`](HKairport03/) | HKairport | -0.09 | 366 s, cruise **8.6 m/s** |
| ✅ | [`HKisland_GNSS03`](HKisland_GNSS03/) | HK_GNSS | -0.092 | 391 s, cruise **9.0 m/s** |
| ⛔ | [`HKairport_GNSS03`](HKairport_GNSS03/) | HK_GNSS | -0.089 | 397 s, cruise 7.1 m/s |

각 폴더에 `config.yaml` 하나와 `README.md`. 시퀀스마다 config는 하나만 둔다 — 탐색 중 생기는 변형은 다른 이름으로 만들고,
끝나면 이긴 것을 `config.yaml`로 남기고 나머지는 지운다. calib에서 유도한 초기 config는 작업 저장소의 생성기가
`config.yaml`이 없을 때만 만들어 준다.

## 변수 정의 — `AMtown03/config.yaml`

| 구분 | 변수 | 값 |
|---|---|---|
| 고정 | `min/max_features` | **120/120** (게이트 승패의 단일 키) |
| 고정 | `--delta-px` / `min_angle_deg` / `min_track_length` / `pixel_standerd_deviation` | 4 px / 2.0 / 4 / 4.0 |
| 고정 | `num_clones` | 11 (게이트와 짝) |
| 고정 | `extrinsics_std` | **0.1 × 6** (유도 extrinsic의 정직한 폭; m급 금지) |
| 고정 | 게이트 값 (`run_feeder` 기본값 = 인자 없이 실행하면 곧 레시피) | `--delta-px 4 --fire-frac 0.1 --min-inject 4 --min-ref 0 --max-dt 0.5` (옛 `--gate/--mode/--trigger` 플래그는 2026-09-04 제거) |
| 고정 | `accelerometer_random_walk` | 2e-3 |
| 기체별 측정 | calib(`T_cam_imu`·intrinsics·distortion), `timeshift_cam_imu`, gyro/accel noise density(datasheet/Allan) | 측정 후 고정 |
| 배포 기하별 | `feature_max_depth`, 창 시간 상한 | 고도·시야각에 맞게 |

튜닝 대상은 없다. 성능은 시작 오프셋 ≥6개의 중앙값+범위로만 보고한다.
⚠️ `--delta-px`(시차 주입 임계값, px)는 `timeshift_cam_imu`(초, τ)와 다른 양이다.
