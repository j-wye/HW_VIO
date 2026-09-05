# vio_node

MSCEqF 기반 단안 VIO. ROS2 Humble 패키지.

```
msceqf/     MSCEqF 엔진 (Apache-2.0)
src/        vio_node (ROS2 노드), run_offline / run_feeder (오프라인 러너)
launch/     vio_node.launch.py
configs/    시퀀스별 config.yaml
scripts/    eval_vio.py, onset_eval.py, verify_bag_names.py, bag1_to_ros2.py, sort_bag_db3.py
```

## 의존성

Ubuntu 22.04 / ROS2 Humble / OpenCV 4 / Boost.

```bash
sudo apt install git libboost-all-dev libopencv-dev ros-humble-cv-bridge ros-humble-image-transport ros-humble-rosbag2-cpp ros-humble-rosbag2-storage ros-humble-rosbag2-storage-default-plugins ros-humble-visualization-msgs python3-colcon-common-extensions python3-numpy
```

Lie++, yaml-cpp, Eigen은 빌드할 때 CMake가 받아온다 (첫 빌드에 네트워크 필요, 커밋 고정).
ROS1 bag 변환(`scripts/bag1_to_ros2.py`)을 쓸 때만 `pip install rosbags`가 추가로 필요하다.

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
ros2 launch vio_node vio_node.launch.py config_filepath:=/path/to/config.yaml imu_topic:=/imu/data cam_topic:=/camera/image_raw
```

또는

```bash
ros2 run vio_node vio_node --ros-args -p config_filepath:=/path/to/config.yaml -p imu_topic:=/imu/data -p cam_topic:=/camera/image_raw
```

파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `config_filepath` | (필수) | MSCEqF config yaml |
| `imu_topic` | (필수) | `sensor_msgs/Imu` |
| `cam_topic` | (필수) | `sensor_msgs/Image` (내부에서 mono8로 변환) |
| `pose_topic` | `/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `path_topic` | `/path` | `nav_msgs/Path` |
| `image_topic` | `/tracks` | `sensor_msgs/Image`, 트랙 오버레이 |
| `extrinsics_topic` | `/extrinsics` | `geometry_msgs/PoseStamped`, cam-IMU extrinsic 추정 |
| `intrinsics_topic` | `/intrinsics` | `sensor_msgs/CameraInfo` |
| `origin_topic` | `/origin` | `geometry_msgs/PoseStamped` |
| `record` / `outbag` | `false` | 입력을 bag으로 저장 |
| `reliable_qos` | `true` | bag 재생용. 실기체에서는 `false` (best effort) |

launch 파일의 토픽 기본값은 `/imu/data`, `/camera/image_raw`, `/vio/pose`, `/vio/path`, `/vio/tracks` 등이다.
bag으로 돌려보려면 노드를 띄운 뒤 `ros2 bag play <bag_dir>`.

## 오프라인 실행

rosbag2를 시간순으로 읽어 한 스레드에서 돌린다. 같은 입력이면 같은 출력이 나오므로 정확도 비교는 이쪽으로 한다.
(노드는 콜백 스레드 구조상 재생 속도에 따라 결과가 달라진다.)

```bash
ros2 run vio_node run_offline <bag_dir> configs/AMtown03/config.yaml out.csv   # 엔진 내부 front-end, 매 프레임 갱신
ros2 run vio_node run_feeder  <bag_dir> configs/AMtown03/config.yaml out.csv   # keyframe 게이트 front-end (기본 설정)
```

`run_feeder` 옵션 (기본값이 곧 기본 설정):

```
--delta-px 4      특징점별 시차 임계값 (px)
--fire-frac 0.1   갱신을 발화시키는 ready 비율
--min-inject 4    갱신에 필요한 최소 특징점 수
--min-ref 0       참조 특징점이 N개 미만인 프레임은 주입하지 않음 (0 = 항상 버림)
--max-dt 0.5      이 시간 안에 갱신이 없으면 강제 주입 (IMU 버퍼 보호용)
--start S --duration D   구간 지정
--disp-log PATH --track-log PATH   게이트 통계 / 주입 특징점 로그
```

bag 토픽 이름은 `/camera/image_raw`, `/imu/data`로 고정돼 있다. 다르면 출력이 비어 있으니 `scripts/verify_bag_names.py`로 확인한다.

## 평가

```bash
python3 scripts/eval_vio.py --selftest
python3 scripts/eval_vio.py --est out.csv --bag <bag_dir>
```

RTK 궤적(bag의 `/ground_truth/fix`)과 비교해 SE3 RMSE, 1초 drift, 30초 국소 오차 등을 낸다. ROS를 source한 상태에서 실행한다.
`scripts/onset_eval.py`는 특정 시각을 기준으로 앵커링한 오차 곡선을 낸다.

시작 시각을 몇 프레임만 바꿔도 APE가 크게 달라지므로, 한 번 돌린 값이 아니라 `--start`를 6개 이상 바꿔 중앙값과 범위로 본다.

## 회귀 확인 (AMtown03)

MARS-LVIG AMtown03을 변환한 rosbag2 기준. 새 머신에서 빌드했으면 한 번 확인한다.

| | 명령 | 기대값 |
|---|---|---|
| 게이트 OFF | `run_offline <bag> configs/AMtown03/config.yaml` | poses 6189, `se3_rmse_m` 47.7535, `drift_1s_median_m` 1.6657 |
| 게이트 ON | `run_feeder <bag> configs/AMtown03/config.yaml` | poses 4441, `se3_rmse_m` 25.0903, `drift_1s_median_m` 1.1991 |

지표가 소수 4자리까지 맞으면 된다. 같은 머신에서도 마지막 비트가 다를 때가 있으니 CSV를 바이트 단위로 비교하지 않는다.

## 라이선스

`msceqf/`는 MSCEqF(Apache-2.0)를 기반으로 하며 일부 파일을 수정했다. 라이선스 전문은 `msceqf/LICENSE`, 수정한 파일에는 그 사실을 적어 두었다.
빌드 시 받아오는 Lie++(Apache-2.0), yaml-cpp(MIT), Eigen(MPL-2.0)은 저장소에 포함하지 않는다. 그 밖의 코드는 이 패키지의 것이다.

## 참고

- 엔진은 자체 Release 플래그(`-flto -march=native` 등)로, `src/`는 `-O3 -march=native`로만 빌드한다. `-march`가 다르면 Eigen 정렬이 달라져 엔진 헤더의 멤버 오프셋이 어긋난다.
- `configs/AMtown03/config.yaml`은 `opencv_threads: 1`이다. 0이면 KLT 결과의 마지막 비트가 부하에 따라 달라진다.
- 노드는 아직 기본 wrapper 수준이다. 정확도 점수, divergence flag, WGS84/NED 출력, 고정 10 Hz 출력은 구현 전이다.
