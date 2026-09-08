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
ros2 launch vio_node vio_node.launch.py dataset:=AMtown03
ros2 launch vio_node vio_node.launch.py dataset:=HKisland03 image_topic:=/cam0 qos_profile:=best_effort
```

`dataset:=<이름>`이 `configs/<이름>/config.yaml`을 잡는다. 다른 곳의 config를 쓰려면 `config_filepath:=`로 직접 준다.
<details>
<summary> I/O & Params</summary>

**Input**
| Parameter | Default | Description |
|---|---|---|
| `dataset` | `AMtown03` | `configs/<이름>/config.yaml`을 고른다 |
| `config_filepath` | `dataset`에서 유도 | 직접 주면 `dataset`보다 우선 |
| `imu_topic` | `/imu/data` | `sensor_msgs/Imu`. $\mathbf{a}$ (m/s²), $\boldsymbol{\omega}$ (rad/s) |
| `image_topic` | `/camera/image_raw` | `sensor_msgs/Image`. mono8 / bgr8 / rgb8 / bgra8 / rgba8 |
| `qos_profile` | `reliable` | `reliable` \| `best_effort` publisher와 match (bag = `reliable`, 실기체 드라이버 = `best_effort`) |

**Output**

| Topic | message | Hz | Description |
|---|---|---|---|
| `/vio/odom` | `nav_msgs/Odometry` | `output_rate_hz` (10 Hz) | 마지막 filter update + IMU propagation |
| `/vio/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Filter Update | update 시점 값 그대로, propagation 없음 |
| `/vio/path` | `nav_msgs/Path` | Filter Update | Accumulated path publish |

**Parameters**

| Parameter | Defaults | Description |
|---|---|---|
| `frame_id` | `odom` | filter origin 기준. z-up, gravity −z, yaw arbitrary |
| `body_frame_id` | `imu` | Odometry `child_frame_id`. `base_link` 아님 |
| `output_rate_hz` | 10.0 | odom 발행 주기 |
| `path_max_poses` | 5000 | Path에 담는 최근 pose 개수. `0`이면 무제한 |
| `image_queue_max` | 30 | 초과 시 오래된 frame부터 drop |
| `imu_hold_max_s` | 1.0 | 카메라가 이만큼 조용하면 frame 없이 IMU를 filter로 |
| `out_csv` | — | update마다 CSV 한 행, `run_feeder`와 같은 형식 |

TF는 publish 안 한다. `base_link` pose가 필요하면 수신 측에서 `base_link`→`imu` static transform.
</details>

## Offline Runner

rosbag을 시간순으로 직접 읽어 한 스레드에서 돌린다. ROS를 거치지 않으므로 재생 속도에 따른 메시지 유실이 없고,
같은 입력이면 항상 같은 CSV가 나온다. **정확도 작업은 이걸로 진행**

```bash
ros2 run vio_node run_feeder AMtown03
```

| | Rule |
|---|---|
| bag | `~/hanwha/src/datasets/$DATASET` |
| config | 설치된 패키지의 `configs/$DATASET/config.yaml` |
| 출력 | 현재 디렉터리의 `$DATASET.csv` |

**반드시 데이터셋 폴더 이름과 `configs/` 아래 폴더 이름을 같게 매칭**

**Keyframe Gate가 하는 일**
1. 매 프레임 Feature Tracking
2. 각 특징점은 **자기 참조 프레임 대비 시차**를 누적
3. 시차가 `delta_px`를 넘은 특징점 비율이 `fire_frac` 이상이면 그 프레임을 필터에 넣는다.
4. 넣을 때 **준비된 특징점만** 넣고, 그것들만 참조를 새로 잡는다. 나머지는 계속 누적한다.

- 기체가 거의 안 움직인 사이의 두 프레임을 넣으면 Triangulation 을 진행할 parallax가 짧아 depth error 폭증을 막음

## 설정

시퀀스마다 `configs/<시퀀스>/config.yaml` 하나에 엔진 파라미터(intrinsics, `T_cam_imu`, IMU 노이즈, `num_clones` 등)와
게이트 값이 함께 들어 있다. 새 카메라·IMU로 쓰려면 이 파일의 calib 값을 바꾼다.

```yaml
frontend:          # keyframe 게이트. 블록이 없으면 아래 값을 기본으로 쓰고 그 사실을 로그에 남긴다
  delta_px: 4.0    # 특징점별 시차 임계값 (px)
  fire_frac: 0.1   # 임계값을 넘은 특징점 비율이 이 이상이면 갱신
  min_inject: 4    # 갱신에 필요한 최소 특징점 수
  min_ref: 0       # 참조 특징점이 이보다 적은 프레임은 버린다 (N>0이면 강제 주입)
  max_dt: 0.5      # 이 시간 안에 갱신이 없으면 강제 주입 (IMU 버퍼 보호)
```

`frontend:` 안에 모르는 키가 있으면 에러다. 오타가 조용히 기본값으로 떨어지지 않게 하기 위한 것이다.
`opencv_threads`는 1이면 결과가 비트 단위로 재현되고, 0이면 모든 코어를 쓴다(마지막 비트가 달라질 수 있다).

## 동작 확인

빌드가 끝났으면 한 번 돌려 본다. MARS-LVIG AMtown03 rosbag2가 필요하다.

```bash
ros2 run vio_node run_feeder AMtown03
```

`AMtown03.csv`가 생기고 stderr에 처리량 한 줄이 찍히면 된다.

노드 경로는 `out_csv`를 주고 같은 bag을 1배속으로 재생해 위 CSV와 비교하면 된다.
`ros2 bag play`에는 `--delay 3`이 필요하다 — 없으면 discovery가 끝나기 전 앞부분을 놓친다.
`--rate 4`처럼 빠르게 재생하면 개발 데스크탑에서도 IMU가 몇 개 유실돼 결과가 달라지므로 기동 확인에만 쓴다.

## MSCEqF에서 바뀐 것

엔진(`src/engine/`, `include/`)은 MSCEqF 원본을 다음과 같이 수정한 것이다. 수정한 파일에는 그 사실을 적어 두었다.

| 파일 | 변경 | 없으면 |
|---|---|---|
| `src/engine/vision/camera.cpp` | radtan 왜곡 계수를 4개 고정에서 4/5/8개 가능으로 | 5계수 calib에서 첫 이미지에 죽는다 |
| `src/engine/msceqf/filter/updater/updater.cpp` | clone이 없는 시각을 참조하는 트랙은 건너뛴다 | 비행 중간부터 시작하면 `std::out_of_range`로 죽는다 |
| `src/engine/msceqf/filter/propagator/propagator.cpp` | IMU 버퍼가 가득 찼을 때의 propagate를 잠금 밖으로 | 같은 mutex를 재획득해 영구 정지한다 (카메라가 몇 초 멈추면 도달) |
| `include/msceqf/msceqf.hpp` | 필터 시각 접근자 추가 | 엔진이 측정을 받아들였는지 호출자가 알 수 없다 |
| 빌드 | ROS1·native·예제·테스트 경로 제거, Lie++·yaml-cpp 커밋 고정 | |

원본의 ROS2 wrapper는 쓰지 않는다. 노드는 새로 썼다. 원본은 콜백 스레드마다 필터를 돌려 재생 속도에 따라 결과가
달라졌고, 타임스탬프 nanosec 변환에 오류가 있었다.

우리가 새로 쓴 것: keyframe 게이트 front-end(`gated_frontend.cpp`), 파이프라인(`pipeline.cpp`), 오프라인 러너(`run_feeder.cpp`),
ROS2 노드(`vio_node.cpp`), 10 Hz 전파 출력.

## Future Work
- **Confidence Scrore**
- **WGS84/NED transfer** 현재 필터 원점 기준 좌표만 낸다.
- **TF publish**