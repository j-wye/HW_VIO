# MSCEqF 엔진 (Apache-2.0, LICENSE 참조).
# 이 디렉터리는 colcon 패키지가 아니라 vio_node 패키지의 서브디렉터리다 (package.xml 없음).
# 산출물: libmsceqf_lib.so (ROS 의존 없음) + include/ 헤더 설치.

## External libraries ------------------------------------------------------------------------
include(FetchContent)

# Lie++ (Eigen도 함께 가져온다). 커밋 고정 — main을 따라가면 재빌드마다 다른 코드가 들어온다.
FetchContent_Declare(
    LiePlusPlus
    GIT_REPOSITORY  https://github.com/aau-cns/Lie-plusplus
    GIT_TAG         ec459c38445d986b5830162ae428b8b242ce5021
    GIT_PROGRESS    TRUE
)
set(LIEPLUSPLUS_TESTS OFF)
list(APPEND external LiePlusPlus)
list(APPEND libs LiePlusPlus Eigen3::Eigen)

# yaml-cpp. 커밋 고정 (위와 같은 이유).
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY  https://github.com/jbeder/yaml-cpp.git
    GIT_TAG         c3f81127320662deb3e28f4da0a511f1faecd860
    GIT_PROGRESS    TRUE
)
list(APPEND external yaml-cpp)
list(APPEND libs yaml-cpp::yaml-cpp)

find_package(Boost REQUIRED)
list(APPEND libs ${Boost_LIBRARIES})

find_package(OpenCV 4 REQUIRED)
list(APPEND libs ${OpenCV_LIBRARIES})

FetchContent_MakeAvailable(${external})
# Lie++/yaml-cpp 헤더는 타깃(LiePlusPlus, yaml-cpp::yaml-cpp)의 INTERFACE include로 들어온다.
# 전역 include에 넣지 말 것: Lie++에도 utils/tools.hpp가 있어 우리 include/utils/tools.hpp를 가린다.

message(STATUS "Eigen VER: " ${EIGEN3_VERSION})
message(STATUS "Lie++ VER: " ${LiePlusPlus_VERSION})
message(STATUS "Yaml-cpp VER: " ${YAML_CPP_VERSION})
message(STATUS "Boost VER: " ${Boost_VERSION})
message(STATUS "OpenCV VER: " ${OpenCV_VERSION})

## Compiler flags (이 디렉터리 범위에만 적용된다) ---------------------------------------------
# -march=native: 빌드한 머신의 ISA가 박힌다 → Jetson에서는 보드에서 직접 빌드할 것.
include(CheckCXXCompilerFlag)
set(RELEASE_FLAGS "-DNDEBUG" "-O3" "-fsee" "-fomit-frame-pointer" "-fno-signed-zeros" "-fno-math-errno" "-funroll-loops" "-fno-unsafe-math-optimizations" "-flto" "-march=native")
set(DEBUG_FLAGS "-O0" "-g3" "-Wall" "-Wextra" "-Werror" "-Wuninitialized" "-Wmaybe-uninitialized" "-pedantic" "-fno-omit-frame-pointer")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(FLAG_LIST ${DEBUG_FLAGS})
else()
    set(FLAG_LIST ${RELEASE_FLAGS})
endif()
foreach(FLAG ${FLAG_LIST})
    string(REPLACE "-" "" FLAG_NAME ${FLAG})
    string(REPLACE "=" "_" FLAG_NAME ${FLAG_NAME})
    string(TOUPPER ${FLAG_NAME} FLAG_VAR_NAME)
    CHECK_CXX_COMPILER_FLAG("${FLAG}" COMPILER_SUPPORTS_${FLAG_VAR_NAME})
    if (COMPILER_SUPPORTS_${FLAG_VAR_NAME})
        if (CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${FLAG}")
        else()
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${FLAG}")
        endif()
    else()
        message("${FLAG} was requested but is not supported.")
    endif()
endforeach()

## Engine library ----------------------------------------------------------------------------
list(
    APPEND lib_sources
    source/msceqf/msceqf.cpp
    source/msceqf/options/msceqf_option_parser.cpp
    source/msceqf/state/state.cpp
    source/msceqf/system/system.cpp
    source/msceqf/symmetry/symmetry.cpp
    source/msceqf/filter/propagator/propagator.cpp
    source/msceqf/filter/updater/zero_velocity_updater.cpp
    source/msceqf/filter/updater/updater.cpp
    source/msceqf/filter/updater/updater_helper.cpp
    source/msceqf/filter/checker/checker.cpp
    source/msceqf/filter/initializer/static_initializer.cpp
    source/vision/camera.cpp
    source/vision/tracker.cpp
    source/vision/track_manager.cpp
)

add_library(msceqf_lib SHARED ${lib_sources})
target_include_directories(msceqf_lib PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(msceqf_lib ${libs})

install(TARGETS msceqf_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
