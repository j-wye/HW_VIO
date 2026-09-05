cmake_minimum_required(VERSION 3.14)
project(vio_node VERSION 0.1 LANGUAGES CXX)

# 패키지 vio_node. 구조:
#   msceqf/            MSCEqF 엔진 (Apache-2.0, msceqf/LICENSE) → libmsceqf_lib.so
#   src/ include/      우리 코드: vio_node(ROS2 노드), run_offline·run_feeder(결정적 오프라인 러너)
#   launch/            노드 launch
#   configs/           시퀀스별 config (평가용)
#   scripts/           평가·데이터 변환 스크립트 (eval_vio.py 등)
# ROS2 Humble 전용.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 엔진. 자기 Release 플래그(-flto 포함)로 빌드된다 — msceqf/CMakeLists.txt.
# 반드시 ROS find_package보다 먼저: ROS 쪽이 먼저 imported 타깃 `yaml-cpp`와 `uninstall`을 만들면
# FetchContent의 yaml-cpp/Eigen이 같은 이름을 못 만들어 configure가 깨진다. 또 아래의 우리 플래그 설정보다
# 먼저 두어, 엔진 디렉터리에는 CMake 기본값이 그대로 상속되게 한다.
add_subdirectory(msceqf)

find_package(ament_cmake REQUIRED)
set(ament_deps
  rclcpp rosbag2_cpp rosbag2_storage
  std_msgs geometry_msgs sensor_msgs nav_msgs
  cv_bridge
)
foreach(dep ${ament_deps})
  find_package(${dep} REQUIRED)
endforeach()
find_package(OpenCV 4 REQUIRED)

# 우리 코드의 컴파일 플래그는 "-O3 -march=native"로 고정한다.
#   -march=native: 엔진과 같아야 한다 — Eigen 정렬(EIGEN_MAX_ALIGN_BYTES)이 바뀌면 엔진 공개 헤더의 멤버 오프셋이
#                  어긋나 값이 틀리거나 종료 시 heap이 깨진다.
#   -flto 등 엔진의 나머지 플래그는 얹지 않는다 — 게이트(run_feeder)의 float 연산이 달라져 축적된 결과의
#                  byte-exact 재현이 깨지고, 실행파일 링크에 -flto가 들어가면 non-LTO인 libyaml-cpp.a와 섞여 링크가 깨진다.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")

## vio_node — ROS2 노드 -------------------------------------------------------------------
add_executable(vio_node src/vio_node.cpp src/msceqf_ros.cpp)
target_include_directories(vio_node PRIVATE include)
ament_target_dependencies(vio_node ${ament_deps})
target_link_libraries(vio_node msceqf_lib ${OpenCV_LIBRARIES})

## 결정적 오프라인 러너 --------------------------------------------------------------------
#   run_offline  엔진 내부 front-end, 매 프레임 주입 (= 게이트 OFF, 엔진 회귀 기준선)
#   run_feeder   keyframe 게이트 front-end (우리 코드), 시차가 정한 간격만 주입 (= 배포 레시피)
foreach(tool run_offline run_feeder)
  add_executable(${tool} src/${tool}.cpp)
  ament_target_dependencies(${tool} rclcpp rosbag2_cpp rosbag2_storage sensor_msgs cv_bridge)
  target_link_libraries(${tool} msceqf_lib ${OpenCV_LIBRARIES})
endforeach()

install(TARGETS vio_node run_offline run_feeder DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/eval_vio.py scripts/onset_eval.py scripts/verify_bag_names.py scripts/bag1_to_ros2.py scripts/sort_bag_db3.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(msceqf_lib)
ament_package()
