cmake_minimum_required(VERSION 3.14)
project(vio_node VERSION 0.1 LANGUAGES CXX)

# ROS2 Humble 전용. 구조는 README.md 참조.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# FetchContent는 반드시 ROS find_package보다 먼저: ROS가 먼저 imported 타깃 `yaml-cpp`·`uninstall`을
# 만들면 같은 이름을 못 만들어 configure가 깨진다.
include(FetchContent)
set(LIEPP_PIN   ec459c38445d986b5830162ae428b8b242ce5021)   # Eigen도 함께 가져온다
set(YAMLCPP_PIN c3f81127320662deb3e28f4da0a511f1faecd860)
FetchContent_Declare(LiePlusPlus
  GIT_REPOSITORY https://github.com/aau-cns/Lie-plusplus
  GIT_TAG        ${LIEPP_PIN}
  GIT_PROGRESS   TRUE)
set(LIEPLUSPLUS_TESTS OFF)
FetchContent_Declare(yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG        ${YAMLCPP_PIN}
  GIT_PROGRESS   TRUE)
find_package(Boost REQUIRED)
find_package(OpenCV 4 REQUIRED)
FetchContent_MakeAvailable(LiePlusPlus yaml-cpp)
# 전역 include에 넣지 말 것: Lie++의 utils/tools.hpp가 우리 것을 가린다.

message(STATUS "Boost ${Boost_VERSION}  OpenCV ${OpenCV_VERSION}  Lie++ ${LIEPP_PIN}  yaml-cpp ${YAMLCPP_PIN}")

## ROS2
find_package(ament_cmake REQUIRED)
set(ament_deps rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs sensor_msgs nav_msgs ament_index_cpp)
foreach(dep ${ament_deps})
  find_package(${dep} REQUIRED)
endforeach()

## 컴파일 플래그
# -march는 엔진과 우리 코드가 같아야 한다: Eigen 정렬이 바뀌면 엔진 헤더의 멤버 오프셋이 어긋난다.
# 반대로 엔진의 나머지 플래그를 우리 코드에 얹으면 게이트의 float 연산이 바뀌어 재현이 깨지고,
# 실행파일 링크에 -flto가 들어가면 non-LTO인 libyaml-cpp.a와 섞여 링크가 깨진다.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
set(ENGINE_FLAGS_WANTED -DNDEBUG -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno
    -funroll-loops -fno-unsafe-math-optimizations -flto=auto -march=native)
include(CheckCXXCompilerFlag)
set(ENGINE_FLAGS "")
foreach(FLAG ${ENGINE_FLAGS_WANTED})
  string(MAKE_C_IDENTIFIER "SUPPORTS${FLAG}" FLAG_VAR)
  CHECK_CXX_COMPILER_FLAG("${FLAG}" ${FLAG_VAR})
  if(${FLAG_VAR})
    list(APPEND ENGINE_FLAGS ${FLAG})
  else()
    message("${FLAG} was requested but is not supported.")
  endif()
endforeach()

## 엔진
add_library(vio_engine SHARED
  src/engine/msceqf/msceqf.cpp
  src/engine/msceqf/options/msceqf_option_parser.cpp
  src/engine/msceqf/state/state.cpp
  src/engine/msceqf/system/system.cpp
  src/engine/msceqf/symmetry/symmetry.cpp
  src/engine/msceqf/filter/propagator/propagator.cpp
  src/engine/msceqf/filter/updater/zero_velocity_updater.cpp
  src/engine/msceqf/filter/updater/updater.cpp
  src/engine/msceqf/filter/updater/updater_helper.cpp
  src/engine/msceqf/filter/checker/checker.cpp
  src/engine/msceqf/filter/initializer/static_initializer.cpp
  src/engine/vision/camera.cpp
  src/engine/vision/tracker.cpp
  src/engine/vision/track_manager.cpp
)
target_include_directories(vio_engine PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# yaml-cpp는 PRIVATE로 묶고 심볼을 감춘다. ROS(rosbag2)가 시스템 yaml-cpp 0.7.0을 같은 프로세스에
# 올리는데, 우리가 정적으로 품은 0.9.0을 밖으로 내보내면 심볼 interposition으로 rosbag2의 호출이
# 우리 구현으로 넘어온다 -- 헤더는 0.7.0, 구현은 0.9.0이라 ABI가 어긋난다.
# --exclude-libs가 정적 아카이브에서 온 심볼을 로컬로 만들어 두 버전이 서로를 보지 못하게 한다.
target_link_libraries(vio_engine
  PUBLIC  LiePlusPlus Eigen3::Eigen ${Boost_LIBRARIES} ${OpenCV_LIBRARIES}
  PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(vio_engine PRIVATE ${ENGINE_FLAGS})
set(VIO_ENGINE_SYMBOL_MAP ${CMAKE_CURRENT_SOURCE_DIR}/cmake/vio_engine_symbols.map)
target_link_options(vio_engine PRIVATE ${ENGINE_FLAGS} -Wl,--exclude-libs,ALL
  -Wl,--version-script=${VIO_ENGINE_SYMBOL_MAP})
set_target_properties(vio_engine PROPERTIES LINK_DEPENDS ${VIO_ENGINE_SYMBOL_MAP})
if("-march=native" IN_LIST ENGINE_FLAGS)
  target_compile_options(vio_engine PUBLIC -march=native)
endif()

## front-end + 파이프라인 (노드와 러너 공용)
add_library(vio_pipeline STATIC src/gated_frontend.cpp src/pipeline.cpp)
target_link_libraries(vio_pipeline vio_engine yaml-cpp::yaml-cpp ${OpenCV_LIBRARIES})

## 실행파일
add_executable(vio_node src/vio_node.cpp)
ament_target_dependencies(vio_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(vio_node vio_pipeline)
# 실행파일도 기본으로 -rdynamic이 붙어 정적 아카이브 심볼을 내보낸다 -- 같은 이유로 막는다
target_link_options(vio_node PRIVATE -Wl,--exclude-libs,ALL)

add_executable(run_feeder src/run_feeder.cpp)
ament_target_dependencies(run_feeder rclcpp rosbag2_cpp rosbag2_storage sensor_msgs ament_index_cpp)
target_link_libraries(run_feeder vio_pipeline)
target_link_options(run_feeder PRIVATE -Wl,--exclude-libs,ALL)

install(TARGETS vio_engine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS vio_node run_feeder DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch configs DESTINATION share/${PROJECT_NAME})

# downstream C++ 인터페이스는 export하지 않는다: 엔진 헤더가 Lie++를 끌어오는데 그건 설치되지 않는다.
ament_package()
