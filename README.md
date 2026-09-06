# vio_node

MSCEqF 기반 단안 VIO. ROS2 Humble 패키지.

```
include/msceqf, vision, utils, types, sensors   MSCEqF 엔진 헤더
src/engine/                                      엔진 소스 → libvio_engine.so
include/vio_node/, src/gated_frontend.cpp, src/pipeline.cpp
                                                 keyframe 게이트 front-end + 파이프라인 (노드·러너 공용)
src/vio_node.cpp                                 ROS2 노드
src/run_offline.cpp, src/run_feeder.cpp          오프라인 러너 (rosbag2 직접 읽기)
launch/vio_node.launch.py
configs/<시퀀스>/config.yaml                     시퀀스별 설정 (엔진 파라미터 + 게이트 값)
```

## 의존성

Ubuntu 22.04 / ROS2 Humble / OpenCV 4 / Boost.

```bash
sudo apt install git libboost-all-dev libopencv-dev ros-humble-cv-bridge ros-humble-rosbag2-cpp ros-humble-rosbag2-storage ros-humble-rosbag2-storage-default-plugins python3-colcon-common-extensions
```

Lie++, yaml-cpp, Eigen은 빌드할 때 CMake가 받아온다 (첫 빌드에 네트워크 필요, 커밋 고정).

## 빌드

```bash
mkdir -p ~/ws/src
git clone git@github.com:j-wye/HW_VIO.git ~/ws/src/vio_node    # 또는 https://github.com/j-wye/HW_VIO.git
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`lto-wrapper: warning: using serial compilation of N LTRANS jobs`는 정상이다.
엔진이 `-march=native`로 빌드되므로 실행할 머신(Jetson)에서 직접 빌드한다.

## 노드 실행

```bash
ros2 launch vio_node vio_node.launch.py
ros2 launch vio_node vio_node.launch.py config_filepath:=/path/to/config.yaml cam_topic:=/cam0
```

launch는 `configs/AMtown03/config.yaml`을 기본값으로 쓴다(설치본 기준). `ros2 run`으로 직접 띄울 때는 `config_filepath`, `imu_topic`, `cam_topic`이 전부 필수다.

```bash
ros2 run vio_node vio_node --ros-args -p config_filepath:=... -p imu_topic:=/imu/data -p cam_topic:=/camera/image_raw
```

입력

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `config_filepath` | launch: `configs/AMtown03/config.yaml` / `ros2 run`: 필수 | 설정 yaml (아래 "설정 파일") |
| `imu_topic` | (필수; launch 기본값 `/imu/data`) | `sensor_msgs/Imu`, m/s², rad/s |
| `cam_topic` | (필수; launch 기본값 `/camera/image_raw`) | `sensor_msgs/Image` (bgr8 / rgb8 / mono8) |
| `qos_profile` | `reliable` | `reliable` 또는 `best_effort`. bag 재생은 `reliable`, best-effort로 publish하는 실기체 드라이버는 `best_effort` |
| `imu_hold_max_s` | `1.0` | 카메라가 멈췄을 때 IMU를 붙들고 있는 한계. 이 시간을 넘으면 프레임 없이도 필터에 넘긴다 |

출력

| 파라미터 | 기본 토픽 | 메시지 | 언제 |
|---|---|---|---|
| `odom_topic` | `/vio/odom` | `nav_msgs/Odometry` | `output_rate_hz`(기본 10 Hz) 고정. 마지막 갱신에서 IMU로 전파한 자세·위치·속도 |
| `pose_topic` | `/vio/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 필터 갱신마다 (게이트가 발화할 때, 기준 데이터에서 약 7 Hz) |
| `path_topic` | `/vio/path` | `nav_msgs/Path` | 갱신마다 (최근 `path_max_poses`개, 기본 2000). 매번 배열 전체를 다시 보내므로 Jetson에서는 줄이거나 구독하지 않는다 |
| `divergence_topic` | `/vio/divergence` | `std_msgs/Bool` | odom과 같은 주기 |

- `frame_id`(기본 `odom`): 필터 원점 기준 좌표계. z 위, 중력 −z. yaw는 임의다. REP-105의 `odom`(월드 고정이지만 드리프트하는 프레임)에 해당한다.
- `body_frame_id`(기본 `imu`): Odometry의 `child_frame_id`. **추정값은 IMU 프레임의 자세**이므로 `base_link`가 아니다. `base_link` 자세가 필요하면 소비자가 자기 URDF의 `base_link`→`imu` static transform을 적용한다. 이 노드는 TF를 publish하지 않는다.
- 모든 파라미터는 launch 인자로도 줄 수 있다. `qos_profile`은 launch에서 후보값이 검증되고, `ros2 run`으로 직접 줄 때는 노드가 검증한다.
- Odometry의 `twist`는 body 좌표계(ROS 관례). 공분산은 마지막 갱신 시점의 값.
- `divergence`는 `divergence_timeout_s`(2 s) 동안 갱신이 없거나, 위치 표준편차가 `divergence_pos_std_m`(100 m)을 넘거나, 상태가 유한하지 않으면 true.
- `out_csv`를 주면 갱신마다 한 행씩 `run_feeder`와 같은 형식의 CSV를 쓴다 (검증용).

처리 구조: 구독 콜백은 큐에 넣기만 하고 스레드 하나가 전부 처리한다. IMU는 도착 즉시 꺼내 고정 주기 출력에 쓰지만,
필터와 게이트에는 **프레임이 경계를 지어줄 때** 넘긴다. 스탬프 t의 프레임은 t 이후 스탬프의 IMU를 본 뒤에 처리 가능해지고
(IMU는 자기 토픽에서 순서대로 오므로 이는 t 이전 IMU가 더 남아 있지 않다는 증거다), 그때 t까지의 IMU를 먼저 전부 넣고 프레임을 넣는다.
그래서 도착 지연과 무관하게 필터는 항상 시간순으로 측정을 받고, 같은 bag을 노드로 재생하면 `run_feeder`와 같은 추정이 나온다.
무거운 연산(KLT, 검출)은 OpenCV가 `opencv_threads`만큼 병렬로 돈다.

## 오프라인 러너

rosbag2를 시간순으로 읽어 같은 파이프라인을 한 스레드에서 돌린다. 정확도 작업은 이쪽으로 한다.

```bash
ros2 run vio_node run_feeder  <bag_dir> configs/AMtown03/config.yaml out.csv   # keyframe 게이트 (노드와 같은 경로)
ros2 run vio_node run_offline <bag_dir> configs/AMtown03/config.yaml out.csv   # 엔진 내부 front-end, 매 프레임 갱신 (비교 기준)
```

`run_feeder` 옵션: `--delta-px --fire-frac --min-inject --min-ref --max-dt`(설정 파일 값 덮어쓰기), `--start S --duration D`, `--disp-log PATH`, `--track-log PATH`.
bag 토픽 이름은 `/camera/image_raw`, `/imu/data`로 고정돼 있다.

## 설정 파일

`configs/<시퀀스>/config.yaml` 하나에 엔진 파라미터(intrinsics, `T_cam_imu`, IMU 노이즈, `num_clones`, …)와 게이트 값이 같이 있다.

```yaml
frontend:          # keyframe 게이트. 없으면 기본값(아래 값과 같음)을 쓰고 로그에 남긴다
  delta_px: 4.0    # 특징점별 시차 임계값 (px)
  fire_frac: 0.1   # 임계값을 넘은 특징점 비율이 이 이상이면 갱신
  min_inject: 4    # 갱신에 필요한 최소 특징점 수
  min_ref: 0       # 참조 특징점이 이보다 적은 프레임은 버림 (N>0이면 강제 주입)
  max_dt: 0.5      # 이 시간 안에 갱신이 없으면 강제 주입 (IMU 버퍼 보호)
```

`frontend:` 안의 모르는 키는 에러다(오타가 기본값으로 조용히 떨어지지 않게). 시퀀스마다 config는 하나만 둔다.
`opencv_threads`는 1이면 결과가 비트 단위로 재현되고, 0이면 모든 코어를 쓴다(마지막 비트가 달라질 수 있음).

## 회귀 확인 (AMtown03)

MARS-LVIG AMtown03 rosbag2 기준, 새 머신에서 빌드했으면 한 번 확인한다. 지표는 내부 평가 도구로 계산한 값이다.

| | 명령 | 기대값 |
|---|---|---|
| 게이트 ON | `run_feeder <bag> configs/AMtown03/config.yaml` | injections 4443, poses 4441, `se3_rmse_m` 25.0903, `drift_1s_median_m` 1.1991 |
| 게이트 OFF | `run_offline <bag> configs/AMtown03/config.yaml` | poses 6189, `se3_rmse_m` 47.7535, `drift_1s_median_m` 1.6657 |
| 노드 | `ros2 bag play <bag> --delay 3` + `out_csv` | `run_feeder` CSV와 동일 |

같은 머신에서도 마지막 비트가 다를 때가 있으니 CSV를 바이트 단위로 비교하지 말고 지표로 본다.

노드 검증에는 `--delay 3`이 필요하다. 없으면 discovery가 끝나기 전의 앞부분을 놓친다. 실시간(1배속)에서는 메시지 유실이 없지만,
`--rate 4`처럼 빠르게 재생하면 이 데스크탑에서도 IMU가 몇 개 유실되어 결과가 달라진다(측정: 129049개 중 4개 유실, APE 25.09 → 25.92).
빠른 재생은 기동 확인용으로만 쓴다.

## MSCEqF에서 바뀐 것

엔진(`src/engine/`, `include/`)은 MSCEqF 원본에 다음 수정을 가한 것이다. 수정한 파일에는 그 사실을 적어 두었다.

| 파일 | 변경 |
|---|---|
| `src/engine/vision/camera.cpp` | radtan 왜곡 계수를 4개 고정에서 4/5/8개 가능으로 (`cv::Mat`). 5계수 calib에서 첫 이미지에 죽던 문제 |
| `src/engine/msceqf/filter/updater/updater.cpp` | clone이 없는 시각을 참조하는 트랙은 건너뜀. 비행 중간에서 시작하면 `std::out_of_range`로 죽던 문제 |
| 빌드 | ROS1·native·예제·테스트 경로 제거, Lie++·yaml-cpp 커밋 고정, 엔진만 원래 Release 플래그(-flto 등) |

원본의 ROS2 wrapper는 쓰지 않는다. 노드는 새로 썼다: 콜백 스레드마다 필터를 돌리던 구조(재생 속도에 따라 결과가 달라짐)를 시간순 단일 처리로, 타임스탬프 nanosec 변환 오류 수정, QoS 선택 파라미터, 노드·토픽 이름 `vio_node`·`/vio/…`.
그 외 우리 코드: keyframe 게이트 front-end(`gated_frontend.cpp`), 파이프라인(`pipeline.cpp`), 오프라인 러너, 10 Hz 전파 출력, divergence 플래그.

## 라이선스

엔진은 MSCEqF(Apache-2.0)를 기반으로 하며 위와 같이 수정했다. 라이선스 전문은 `LICENSE`. 빌드 시 받아오는 Lie++(Apache-2.0), yaml-cpp(MIT), Eigen(MPL-2.0)은 저장소에 포함하지 않는다. 나머지 코드는 이 패키지의 것이다.

## 상태

- 정확도 점수(m) 출력과 WGS84/NED 변환은 구현 전이다. `divergence`는 위의 단순 규칙이다.
- Jetson 실측(rate·CPU·latency)은 아직 없다.
