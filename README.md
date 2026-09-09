# vio_node

**Integration Pipeline** : Filter based VIO Algorithm *MSCEqF* 를 기반으로, frontend에서 keyframe gate 와
ROS node, offline test runner 통합

```
include/
  msceqf/ vision/ utils/ types/ sensors/   MSCEqF header
  vio_node/                                keyframe gate + pipeline header
src/
  engine/                                  engine source → libvio_engine.so
  gated_frontend.cpp  pipeline.cpp         keyframe gate + pipeline source
  vio_node.cpp                             ROS2 node
  run_feeder.cpp                           Offline runner
launch/
  vio_node.launch.py
configs/
  $DATASET/config.yaml                     시퀀스별 설정 (엔진 파라미터 + 게이트 값)
LICENSE
CMakeLists.txt
package.xml
```

## Installation

```bash
sudo apt install git python3-colcon-common-extensions python3-rosdep
mkdir -p ~/hanwha/src
git clone https://github.com/j-wye/HW_VIO.git ~/hanwha/src/vio_node
```

### Datasets
- https://drive.google.com/file/d/1p1vz40NBtruBvdrEWW66WqU1A9vXY9A_/view?usp=drive_link
- https://drive.google.com/file/d/14CVP-OpuUyURa9ks-Dhs0S61OjfhUfNx/view?usp=drive_link
- https://drive.google.com/file/d/1GEHYUk_hRmk8kg16y5KBDroBcoXceers/view?usp=drive_link
- https://drive.google.com/file/d/1uFH0lDnHZihZU1Y58YDkg43DhbGivjz0/view?usp=drive_link

```bash
mkdir -p ~/hanwha/src/datasets
cd ~/Downloads
tar -zxvf AMtown03.tar.gz        -C ~/hanwha/src/datasets
tar -zxvf AMvalley03.tar.gz      -C ~/hanwha/src/datasets
tar -zxvf HKisland03.tar.gz      -C ~/hanwha/src/datasets
tar -zxvf HKisland_GNSS03.tar.gz -C ~/hanwha/src/datasets
touch ~/hanwha/src/datasets/COLCON_IGNORE
```

### Dependency & Build
```bash
cd ~/hanwha
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -y
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
then project structure follows:
```
/hanwha/
  build/ install/ log/
  src/
    vio_node/
    datasets/
```

## Execution
```bash
ros2 launch vio_node vio_node.launch.py
ros2 launch vio_node vio_node.launch.py config_filepath:=/path/config.yaml cam_topic:=/cam0 qos_profile:=best_effort
```
<details>
<summary> I/O & Params</summary>

**Input**
| Parameter | Default | Description |
|---|---|---|
| `config_filepath` | `configs/AMtown03/config.yaml` | config.yaml |
| `imu_topic` | `/imu/data` | `sensor_msgs/Imu`. $\mathbf{a}$ (m/s²), $\boldsymbol{\omega}$ (rad/s) |
| `cam_topic` | `/camera/image_raw` | `sensor_msgs/Image`. mono8 / bgr8 / rgb8 / bgra8 / rgba8 |
| `qos_profile` | `reliable` | `reliable` \| `best_effort` publisher와 match (bag = `reliable`, 실기체 드라이버 = `best_effort`) |

**Output**

| Topic | message | Hz | Description |
|---|---|---|---|
| `/vio/odom` | `nav_msgs/Odometry` | `output_rate_hz` (10 Hz) | 마지막 filter update + IMU propagation |
| `/vio/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Filter Update | update 시점 값 그대로, propagation 없음 |
| `/vio/path` | `nav_msgs/Path` | Filter Update | Accumulated path publish |
| `/vio/divergence` | `std_msgs/Bool` | odom과 동일 | update 끊김·position std·NaN 기준의 임시 규칙 |

**Parameters**

| Parameter | Defaults | Description |
|---|---|---|
| `frame_id` | `odom` | filter origin 기준. z-up, gravity −z, yaw arbitrary |
| `body_frame_id` | `imu` | Odometry `child_frame_id`. `base_link` 아님 |
| `output_rate_hz` | 10.0 | odom 발행 주기 |
| `path_max_poses` | 5000 | Path에 담는 최근 pose 개수. `0`이면 무제한 |
| `image_queue_max` | 30 | 초과 시 오래된 frame부터 drop |
| `imu_hold_max_s` | 1.0 | 카메라가 이만큼 조용하면 frame 없이 IMU를 filter로 |
| `divergence_timeout_s` | 2.0 | 이 시간 동안 update가 없으면 divergence |
| `divergence_pos_std_m` | 100.0 | position std가 이 값을 넘으면 divergence |
| `out_csv` | — | update마다 CSV 한 행, `run_feeder`와 같은 형식 |

TF는 publish 안 한다. `base_link` pose가 필요하면 수신 측에서 `base_link`→`imu` static transform.
</details>

### Offline Runner

rosbag을 시간순으로 직접 읽어 한 스레드에서 돌린다. ROS를 거치지 않으므로 재생 속도에 따른 메시지 유실이 없고,
같은 입력이면 항상 같은 CSV가 나온다. **정확도 작업은 이걸로 진행**

```bash
ros2 run vio_node run_feeder AMtown03
```

## Rule

<details>
<summary>Workspace Structure</summary>

**반드시 데이터셋 폴더 이름과 `configs/` 아래 폴더 이름을 같게 매칭**

| | Rule |
|---|---|
| bag | `~/hanwha/src/datasets/$DATASET` |
| config | 설치된 패키지의 `configs/$DATASET/config.yaml` |
| 출력 | 현재 디렉터리의 `$DATASET.csv` |

**Keyframe Gate가 하는 일**
1. 매 프레임 Feature Tracking
2. 각 특징점은 **자기 참조 프레임 대비 시차**를 누적
3. 시차가 `delta_px`를 넘은 특징점 비율이 `fire_frac` 이상이면 그 프레임을 필터에 넣는다.
4. 넣을 때 **준비된 특징점만** 넣고, 그것들만 참조를 새로 잡는다. 나머지는 계속 누적한다.

- 기체가 거의 안 움직인 사이의 두 프레임을 넣으면 Triangulation 을 진행할 parallax가 짧아 depth error 폭증을 막음

`config.yaml`의 `frontend:` 블록:
```yaml
frontend:          # keyframe 게이트
  delta_px: 4.0    # Feature 별 parallax threshold (px)
  fire_frac: 0.1   # Threshold를 넘은 feature ratio
  min_inject: 4    # minimum feature num
  min_ref: 0       # 참조 특징점이 이보다 적은 프레임은 버린다 (N>0이면 강제 주입)
  max_dt: 0.5      # 이 시간 안에 갱신이 없으면 강제 주입 (IMU 버퍼 보호)
```
</details>

<details>
<summary>MSCEqF에서 바뀐 것</summary>

엔진(`src/engine/`, `include/`)은 MSCEqF 원본을 다음과 같이 수정한 것이다. 수정한 파일에는 그 사실을 적어 두었다.

| 파일 | 변경 | 없으면 |
|---|---|---|
| `src/engine/vision/camera.cpp` | radtan 왜곡 계수를 4개 고정에서 4/5/8개 가능으로 | 5계수 calib에서 첫 이미지에 죽는다 |
| `src/engine/msceqf/filter/updater/updater.cpp` | clone이 없는 시각을 참조하는 트랙은 건너뛴다 | 비행 중간부터 시작하면 `std::out_of_range`로 죽는다 |
| `src/engine/msceqf/filter/propagator/propagator.cpp` | IMU 버퍼가 가득 찼을 때의 propagate를 잠금 밖으로 | 같은 mutex를 재획득해 영구 정지한다 (카메라가 몇 초 멈추면 도달) |
| `include/msceqf/msceqf.hpp` | 필터 시각 접근자 추가 | 엔진이 측정을 받아들였는지 호출자가 알 수 없다 |
| 빌드 | ROS1·native·예제·테스트 경로 제거, Lie++·yaml-cpp 커밋 고정 | |
</details>

### New Dataset Setting
Sequence마다 `configs/$DATASET/config.yaml` 하나에 파라미터(intrinsics, `T_cam_imu`, IMU 노이즈, `num_clones` 등)와 frontend gate 값이 함께 들어 있다. 새 카메라·IMU·새 기체·새 데이터셋을 쓰려면 이 파일의 calib 값을 바꾼다.

## Future Work

- **Confidence Scrore**
- **WGS84/NED transfer** 현재 필터 원점 기준 좌표만 낸다.
- **TF publish**
