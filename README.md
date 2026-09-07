# vio_node

카메라 한 대와 IMU로 위치·자세를 추정하는 ROS2 Humble 패키지. 필터는 MSCEqF이고, 그 앞단의 keyframe 게이트와
ROS 노드, 오프라인 러너를 이 저장소에서 구현했다.

노드는 `sensor_msgs/Imu`와 `sensor_msgs/Image`를 구독해 `nav_msgs/Odometry`를 10 Hz로 낸다.

```
CMakeLists.txt  package.xml  LICENSE
include/
  msceqf/ vision/ utils/ types/ sensors/   MSCEqF 엔진 헤더
  vio_node/                                게이트·파이프라인 헤더
src/
  engine/                                  엔진 소스 → libvio_engine.so (ROS 의존 없음)
  gated_frontend.cpp  pipeline.cpp         keyframe 게이트 + 파이프라인 (노드와 러너가 공유)
  vio_node.cpp                             ROS2 노드
  run_feeder.cpp                           오프라인 러너
launch/vio_node.launch.py
configs/<시퀀스>/config.yaml               시퀀스별 설정 (엔진 파라미터 + 게이트 값)
```

## 설치

Ubuntu 22.04 / ROS2 Humble. 디스크는 데이터셋 때문에 **45 GB 이상** 비어 있어야 한다(zip 21 GB + 압축 해제 21 GB).

```bash
sudo apt install git unzip python3-colcon-common-extensions python3-rosdep
python3 -m pip install gdown
```

**1. 워크스페이스와 패키지**

```bash
mkdir -p ~/hanwha/src
git clone https://github.com/j-wye/HW_VIO.git ~/hanwha/src/vio_node
```

**2. 데이터셋** (MARS-LVIG 시퀀스의 rosbag2 변환본, 21.4 GB)

```bash
cd ~/hanwha/src
gdown 1WpPCvj6n-_g99b8_OwjQYuYs4JMf-vKi -O datasets.zip
unzip -q datasets.zip && rm datasets.zip
touch datasets/COLCON_IGNORE
```

`COLCON_IGNORE`는 colcon이 21 GB짜리 데이터셋 트리를 매번 훑지 않게 한다.

**3. 의존성과 빌드**

의존성은 `package.xml`에 선언돼 있고 `rosdep`이 그대로 설치한다.

```bash
cd ~/hanwha
sudo rosdep init && rosdep update          # 이 머신에서 처음 한 번만
rosdep install --from-paths src --ignore-src -y
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**4. 데이터셋 위치 등록** — 러너가 시퀀스 이름만으로 bag을 찾게 한다.

```bash
echo "export VIO_DATASETS=$HOME/hanwha/src/datasets" >> ~/.bashrc && source ~/.bashrc
```

끝나면 이런 모양이 된다.

```
~/hanwha/
  build/ install/ log/        colcon 산출물
  src/
    vio_node/                 이 저장소
    datasets/<시퀀스>/         rosbag2
```

Lie++, yaml-cpp, Eigen은 CMake가 빌드 중에 받아온다 — 첫 빌드에 네트워크가 필요하다. 커밋을 고정해 두었다.

엔진이 `-march=native`로 빌드되므로 **실행할 머신(Jetson)에서 직접 빌드해야 한다.** 다른 CPU에서 만든 바이너리는 돌지 않는다.
엔진은 `-flto=auto`로 링크한다. 메모리가 빠듯한 보드에서 LTO 링크가 죽으면 `-flto`(직렬)로 바꾸거나 swap을 확보한다.

## 실행

```bash
ros2 launch vio_node vio_node.launch.py
ros2 launch vio_node vio_node.launch.py config_filepath:=/path/config.yaml cam_topic:=/cam0 qos_profile:=best_effort
```

아래 파라미터는 전부 `이름:=값`으로 덮어쓸 수 있다. launch 파일이 기본값을 채워 주므로 따로 줄 것은 없다.

**입력**

| 파라미터 | launch 기본값 | 설명 |
|---|---|---|
| `config_filepath` | `configs/AMtown03/config.yaml` | 설정 파일. 아래 "설정" 참조 |
| `imu_topic` | `/imu/data` | `sensor_msgs/Imu`. 가속도 m/s², 각속도 rad/s |
| `cam_topic` | `/camera/image_raw` | `sensor_msgs/Image`. mono8 / bgr8 / rgb8 / bgra8 / rgba8 |
| `qos_profile` | `reliable` | `reliable` \| `best_effort`. bag 재생은 `reliable`, best-effort로 내보내는 실기체 드라이버는 `best_effort` |

**출력**

| 토픽 (파라미터) | 메시지 | 주기 |
|---|---|---|
| `/vio/odom` (`odom_topic`) | `nav_msgs/Odometry` | `output_rate_hz` 고정 (기본 10 Hz) |
| `/vio/pose` (`pose_topic`) | `geometry_msgs/PoseWithCovarianceStamped` | 필터 갱신마다 (기준 데이터에서 약 7 Hz) |
| `/vio/path` (`path_topic`) | `nav_msgs/Path` | 갱신마다 |
| `/vio/divergence` (`divergence_topic`) | `std_msgs/Bool` | odom과 같은 주기 |

`/vio/odom`은 마지막 필터 갱신 위치에서 IMU로 전파한 값이라, 갱신 주기와 무관하게 일정한 주기로 나온다.
`/vio/path`는 매번 배열 전체를 다시 보낸다. 기본값 5000은 7 Hz 갱신 기준 약 12분치이고 AMtown03(4441개) 전 구간이 들어간다.
RViz에서 궤적 앞부분이 잘려 보이면 이 값이 모자란 것이다 — `path_max_poses:=0`이면 무제한이다.
대신 길수록 발행 대역이 커지므로(5000개면 한 번에 약 280 KB) Jetson에서는 줄이거나 `/vio/path`를 구독하지 않는 편이 낫다.

**좌표계.** `frame_id`(기본 `odom`)는 필터 원점 기준 좌표계다. z가 위, 중력이 −z이고 yaw는 임의다.
드리프트하는 월드 고정 프레임이므로 REP-105의 `odom`에 해당한다.
`body_frame_id`(기본 `imu`)는 Odometry의 `child_frame_id`다. **추정 대상이 IMU 프레임이라 `base_link`가 아니다.**
`base_link` 자세가 필요하면 받는 쪽에서 자기 URDF의 `base_link`→`imu` static transform을 적용한다.
이 노드는 TF를 publish하지 않는다.

**나머지 파라미터**

| | 기본값 | |
|---|---|---|
| `output_rate_hz` | 10.0 | odom·divergence 발행 주기 |
| `divergence_timeout_s` | 2.0 | 이 시간 동안 갱신이 없으면 divergence |
| `divergence_pos_std_m` | 100.0 | 위치 표준편차가 이를 넘으면 divergence |
| `path_max_poses` | 5000 | Path에 담는 최근 pose 개수. `0`이면 무제한 |
| `image_queue_max` | 30 | 처리 대기 프레임 상한. 넘으면 오래된 것부터 버린다 |
| `imu_hold_max_s` | 1.0 | 카메라가 이만큼 조용하면 프레임 없이 IMU를 필터에 넘긴다 |
| `out_csv` | (없음) | 주면 갱신마다 CSV 한 행. `run_feeder` 출력과 같은 형식 (빌드 확인용) |

`divergence`는 위 두 임계값을 넘거나 상태가 유한하지 않을 때 true다. 판정 규칙은 아직 임시다("상태" 참조).

**동작.** 구독 콜백은 큐에 넣기만 하고 스레드 하나가 전부 처리한다. IMU는 도착 즉시 꺼내 고정 주기 출력에 쓰지만,
필터와 게이트에는 프레임이 시간 경계를 지어 줄 때 넘긴다. 스탬프 t의 프레임은 t 이후 스탬프의 IMU를 본 뒤에
처리되고(IMU는 자기 토픽에서 순서대로 오므로 t 이전 IMU가 남아 있지 않다는 증거다), 그때 t까지의 IMU를 전부 넣고
프레임을 넣는다. 그래서 도착 지연과 무관하게 필터는 항상 시간순으로 측정을 받고, 같은 bag을 재생하면
`run_feeder`와 같은 추정이 나온다. 무거운 연산(KLT, 특징점 검출)은 OpenCV가 `opencv_threads`만큼 병렬로 돈다.

영상 크기가 설정의 `resolution`과 다르거나 지원하지 않는 encoding이면 그 프레임을 버리고 로그를 남긴다.
엔진이 측정을 거부하면(상태보다 오래된 프레임, propagation 실패) 갱신으로 세지 않는다.
종료할 때 처리량과 `dropped_images`·`rejected_updates`를 로그로 남긴다.

## 오프라인 러너

rosbag2를 시간순으로 직접 읽어 한 스레드에서 돌린다. ROS를 거치지 않으므로 재생 속도에 따른 메시지 유실이 없고,
같은 입력이면 항상 같은 CSV가 나온다. **정확도 작업은 이쪽으로 한다.**

노드가 아니라 그냥 실행파일이라(`--ros-args`를 받지 않는다) `ros2 run`으로 띄운다.
**시퀀스 이름만 주면 나머지 경로는 규칙으로 정해진다.**

```bash
ros2 run vio_node run_feeder AMtown03
```

| | 규칙 |
|---|---|
| bag | `$VIO_DATASETS/<시퀀스>` (설치 4단계에서 등록). 없으면 현재 디렉터리의 `datasets/<시퀀스>` |
| config | 설치된 패키지의 `configs/<시퀀스>/config.yaml` |
| 출력 | 현재 디렉터리의 `<시퀀스>.csv` |

그래서 데이터셋 폴더 이름과 `configs/` 아래 폴더 이름을 **같게 맞춰 두면** 새 시퀀스를 추가해도 명령이 그대로다.
시작할 때 실제로 어떤 경로를 골랐는지 stderr에 찍는다.

경로를 직접 주려면 세 개를 전부 준다.

```bash
ros2 run vio_node run_feeder <bag_dir> <config.yaml> <out.csv>
```

`vio_node`가 쓰는 front-end와 **같은 코드**(`gated_frontend.cpp`)를 쓴다. 그래서 이 CSV와 노드의 `out_csv`가
같으면 노드 경로가 정상이라는 뜻이다.

**keyframe 게이트가 하는 일**

1. 매 프레임 특징점을 추적한다 (엔진의 `Tracker`를 그대로 쓴다).
2. 각 특징점은 **자기 참조 프레임 대비 시차**를 누적한다. IMU 회전분을 빼서, 기체가 제자리에서 회전만 해도
   시차가 쌓이지 않게 한다 — 회전은 삼각측량에 도움이 안 되기 때문이다.
3. 시차가 `delta_px`를 넘은 특징점 비율이 `fire_frac` 이상이면 그 프레임을 필터에 넣는다.
4. 넣을 때 **준비된 특징점만** 넣고, 그것들만 참조를 새로 잡는다. 나머지는 계속 누적한다.

프레임은 10 Hz로 들어와도 필터 갱신은 약 7 Hz다(AMtown03 기준 6199프레임 → 4443회 주입).
기체가 거의 안 움직인 사이의 두 프레임을 넣으면 삼각측량 기선이 짧아 depth가 크게 틀리는데, 게이트가 그걸 막는다.

임계값은 `config.yaml`의 `frontend:` 블록에 있고, 전 시퀀스가 같은 값을 쓴다.

`run_feeder` 옵션: `--delta-px --fire-frac --min-inject --min-ref --max-dt`(설정 파일 값을 덮어쓴다),
`--start S --duration D`, `--disp-log PATH`, `--track-log PATH`. 모르는 옵션은 거부한다.

bag의 토픽 이름은 `/camera/image_raw`, `/imu/data`로 고정돼 있다. 러너는 bag의 기록 순서대로 읽으므로,
노드와 같은 결과가 나오려면 bag이 header stamp 순서로 정렬돼 있어야 한다.

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

## 빌드 확인

빌드가 제대로 됐는지 보는 절차다. 정확도를 재는 것이 아니라, **이 머신에서 빌드한 결과가 우리 것과 같은지**만 본다.
MARS-LVIG AMtown03 rosbag2가 필요하다.

```bash
ros2 run vio_node run_feeder AMtown03
```

종료할 때 stderr에 `injections=4443 poses=4441`이 찍히면 정상이다. 이 숫자는 게이트가 몇 번 발화했고 필터가 몇 번
갱신했는지라서, 빌드나 설정이 어긋나면 바로 달라진다.

노드 경로까지 보려면 `out_csv`를 주고 같은 bag을 1배속으로 재생한 뒤 위 CSV와 비교한다. 같으면 노드가 러너와 같은
계산을 한 것이다. `ros2 bag play`에는 `--delay 3`이 필요하다 — 없으면 discovery가 끝나기 전 앞부분을 놓친다.
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
ROS2 노드(`vio_node.cpp`), 10 Hz 전파 출력, divergence 플래그.

## 라이선스

엔진은 MSCEqF(Apache-2.0)를 기반으로 하며 위와 같이 수정했다. 라이선스 전문은 [`LICENSE`](LICENSE).
빌드할 때 받아오는 Lie++(Apache-2.0), yaml-cpp(MIT), Eigen(MPL-2.0)은 이 저장소에 포함하지 않는다.
나머지 코드는 이 패키지의 것이다.

## 상태

구현하지 않은 것:

- **정확도 점수(m) 출력.** pose·odom의 공분산은 필터 내부 값을 ROS 관례로 회전해 내보내지만 보정하지 않았다.
  실제 오차를 반영하는 값이 아니다.
- **WGS84/NED 변환.** 지금은 필터 원점 기준 좌표만 낸다.
- **TF publish.**
- `divergence` 판정은 위의 단순 규칙이다.
- Jetson 실측(출력 주기, CPU·GPU 점유, 지연)은 아직 없다.
