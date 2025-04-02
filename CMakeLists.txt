cmake_minimum_required(VERSION 3.5)
project(slam)

SET(CMAKE_BUILD_TYPE "Release")

set(PYTHON_EXECUTABLE /usr/bin/python3)

find_package(catkin QUIET)  # ROS1 核心包
find_package(ament_cmake QUIET)  # ROS2 核心包

if(catkin_FOUND)
  set(ROS_VERSION "ROS1")
elseif(ament_cmake_FOUND)
  set(ROS_VERSION "ROS2")
endif()

# if(ROS_VERSION STREQUAL "ROS1")
#   configure_file(
#           package.xml.ros1.in
#           ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
#           @ONLY
#   )
# elseif(ROS_VERSION STREQUAL "ROS2")
#   configure_file(
#           package.xml.ros2.in
#           ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
#           @ONLY
#   )
# endif()

IF (WIN32)
    ADD_COMPILE_OPTIONS(/W4 /std:c++17 /EHsc )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14 /O2")
    # 关键配置：指定vcpkg工具链路径
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
ELSE()
    ADD_COMPILE_OPTIONS(-std=c++17)
    set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -fexceptions")
    add_compile_options(-Wno-class-memaccess)
    add_compile_options(-Wall -Wextra)
    add_compile_options(-Wno-unused-parameter)
ENDIF()

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(-DNOMINMAX)  # 所有目标生效

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

#========================
# OpenMp part
#========================
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 9)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=8)
    message("core for MP:  ${PROC_NUM}/${N}")
  elseif(N GREATER 7)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=6)
    message("core for MP:  ${PROC_NUM}/${N}")
  elseif(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  4")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
  endif()
else()
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=4)
endif()
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# 查找 Ceres Solver
find_package(Ceres REQUIRED)

#========================
# ROS lib
#========================
find_package(catkin QUIET COMPONENTS roscpp)

if(ROS_VERSION STREQUAL "ROS1")
  message(STATUS "Building with ROS1")
  add_definitions(-DUSE_ROS1)  # 定义ROS1宏
  find_package(catkin REQUIRED COMPONENTS
          geometry_msgs
          nav_msgs
          sensor_msgs
          roscpp
          std_msgs
          pcl_ros
          pcl_conversions
          tf
          message_generation
          eigen_conversions
          cv_bridge
          image_transport
          )
  include_directories(${catkin_INCLUDE_DIRS})
  catkin_package(
          CATKIN_DEPENDS nav_msgs roscpp std_msgs message_runtime cv_bridge image_transport
          INCLUDE_DIRS include
  )
elseif(ROS_VERSION STREQUAL "ROS2")
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(tf2_ros REQUIRED)  # 添加 tf2_ros 依赖
  message(STATUS "Building with ROS2")
  add_definitions(-DUSE_ROS2)  # 定义ROS2宏
endif()

#========================
# third_party lib
#========================
add_subdirectory(third_party/Sophus)
add_subdirectory(third_party/vikit_common)

#========================
# system lib
#========================
find_package(Eigen3 REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
find_package(yaml-cpp REQUIRED)

# Include dirs
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
        include)

#========================
# fast_livo lib
#========================
add_library(livo_initial src/livo_initial/initializer.cpp src/livo_initial/pre_integration.cpp src/livo_initial/global_sfm.cpp src/livo_initial/feature_tracker.cpp)
target_link_libraries(livo_initial ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} Sophus)

#========================
# fast_livo lib
#========================
add_library(fast_livo
        include/ikd-Tree/ikd_Tree.cpp
        src/lidar_selection.cpp
        src/frame.cpp
        src/point.cpp
        src/feature.cpp
        src/map.cpp
        src/IMU_Processing.cpp
        src/preprocess.cpp
        src/laserMapping.cpp
        )
target_include_directories(fast_livo
        PUBLIC
        include
        )
target_link_libraries(fast_livo
        PUBLIC
        ${PCL_LIBRARIES}
        vikit_common
        Sophus
        livo_initial
        )

if(WIN32)
  find_package(Threads REQUIRED)
  target_link_libraries(fast_livo PRIVATE Threads::Threads)
endif()

if(ROS_VERSION STREQUAL "ROS1")
  target_link_libraries(fast_livo
          PUBLIC
          ${catkin_LIBRARIES})
elseif(ROS_VERSION STREQUAL "ROS2")
  ament_target_dependencies(fast_livo
          PUBLIC
          rclcpp
          cv_bridge
          pcl_conversions
          sensor_msgs
          nav_msgs
          image_transport
          tf2_ros
          )
endif()

#========================
# ROS node
#========================
add_executable(slam_node
        node/fast_livo_slam_app_ros_node.cpp
        )
target_link_libraries(slam_node
        fast_livo
        ${YAML_CPP_LIBRARIES}
        )

if(ROS_VERSION STREQUAL "ROS1")
  # do nothing
elseif(ROS_VERSION STREQUAL "ROS2")
  ament_export_libraries(fast_livo)
  ament_export_dependencies(vikit_common)
  install(DIRECTORY include/
          DESTINATION include
          )
  install(TARGETS fast_livo)
  install(TARGETS vikit_common)

  install(TARGETS slam_node DESTINATION lib/slam)
  ament_package()
endif()
