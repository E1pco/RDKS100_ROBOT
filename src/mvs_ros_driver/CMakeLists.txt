cmake_minimum_required(VERSION 3.10)
project(mvs_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------- MVS SDK platform-specific library path ----------
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(MVS_LIB_DIR /opt/MVS/lib/aarch64)
  message(STATUS "MVS SDK: aarch64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(MVS_LIB_DIR /opt/MVS/lib/64)
  message(STATUS "MVS SDK: x86_64")
endif()
link_directories(${MVS_LIB_DIR})

# Embed RPATH so the runtime linker can find libMvCameraControl.so
set(CMAKE_INSTALL_RPATH "${MVS_LIB_DIR}")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# ---------- ROS2 dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# ---------- Executable ----------
add_executable(grabImgWithTrigger src/grab_trigger.cpp)

target_include_directories(grabImgWithTrigger PRIVATE
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(grabImgWithTrigger ${dependencies})

target_link_libraries(grabImgWithTrigger
  ${OpenCV_LIBS}
  MvCameraControl
)

# ---------- Install ----------
install(TARGETS grabImgWithTrigger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
