cmake_minimum_required(VERSION 3.8)
project(fast_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/livox_custom_to_pointcloud2.py
  scripts/pcd_to_occupancy_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
