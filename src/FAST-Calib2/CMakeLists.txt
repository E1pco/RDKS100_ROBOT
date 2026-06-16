cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Main calibration executable
add_executable(fast_calib src/main.cpp)
ament_target_dependencies(fast_calib
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  rosbag2_cpp
  livox_ros_driver2
  tf2
  tf2_ros
  cv_bridge
  image_transport
)
target_link_libraries(fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)

# LiDAR center test executable
add_executable(lidar_center_test src/lidar_center_test.cpp)
ament_target_dependencies(lidar_center_test
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  rosbag2_cpp
  livox_ros_driver2
  tf2
  tf2_ros
  cv_bridge
  image_transport
)
target_link_libraries(lidar_center_test
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)

# Multi-scene calibration executable
add_executable(multi_fast_calib src/multi_scene.cpp)
ament_target_dependencies(multi_fast_calib
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  rosbag2_cpp
  livox_ros_driver2
  tf2
  tf2_ros
  cv_bridge
  image_transport
)
target_link_libraries(multi_fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)

install(TARGETS
  fast_calib
  lidar_center_test
  multi_fast_calib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz_cfg
  scripts
  calibration_data
  img
  output
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
