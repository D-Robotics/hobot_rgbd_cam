# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(rgbd_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

OPTION(BORAD_UBUNTU    "Board is ubuntu"    ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-w)
find_package(hbm_img_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_definitions(-DRGBD -DHORIZON_PLATFORM -DCV_CONVERT_CLR)

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include
  src/TOF_SDK/include
  src/application
  src/application/camera
  src/application/common
  src/application/i2c
  src/application/tof_depth
  src/application/v4l2_capture
)

include_directories(
  ${CMAKE_SYSROOT}/usr/include/vio
)


link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
  src/TOF_SDK/libs
)

if(BORAD_UBUNTU)
include_directories(
  src/application/horizon_mpp/
)
else()
include_directories(
  src/application/horizon_mpp
  src/application/horizon_mpp/linaro/include
)
link_directories(
  src/application/horizon_mpp/linaro/libs
)
endif()

## Build the mipi camera library
add_executable(${PROJECT_NAME}
  src/rgbd_sensor.cpp
  src/rgbd_node.cpp
  src/rgbd_cam.cpp
  src/sunny_camera.cpp
  
  src/application/camera/camera_control.cpp
  src/application/camera/camera_hb_cfg.cpp
  
  src/application/horizon_mpp/horizon_mpp.cpp
  src/application/i2c/tof_i2c.cpp
  src/application/tof_depth/tof_depth_process.cpp
)

set(BASE_LIBRARIES dl rt pthread tof_mod_sdk tof_rgbd_sdk)
set(X3_HAPI_LIB hbmedia isp vio alog cam ion gdcbin hbmem iar tinyalsa)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} ${X3_HAPI_LIB} yaml-cpp
  # ${ROS_LIB}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  hbm_img_msgs
  OpenCV
  yaml_cpp_vendor
  std_srvs
  )

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TOF_SDK/libs/libtof_mod_sdk.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TOF_SDK/libs/libtof_rgbd_sdk.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorlib/libgc2053.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorlib/libgc2053_linear.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorlib/libirs2381c.so
  DESTINATION lib/)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TOF_SDK/parameter
  DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
