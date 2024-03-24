English| [简体中文](./README_cn.md)

# Getting Started with rgbd_sensor Node
---
# Intro
---
By reading this document, users can easily capture the video stream data of the MIPI depth camera on the Horizon X3 development board and publish depth map data/ grayscale image data/ RGB image data/ camera intrinsics that meet ROS standards, as well as computed point cloud data on the ROS platform for other ROS Nodes to subscribe and view real-time effects on rviz. Supports shared memory publishing.

# Build
---
## Dependency

Dependencies:
ROS package:
- sensor_msgs
- hbm_img_msgs

The hbm_img_msgs package is a custom image message format in hobot_msgs, used for image transmission in shared memory scenarios.

## Development Environment
- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/ Linaro GCC 9.3.0
## Package Description
---
The source code includes the rgbd_sensor package. After compiling rgbd_sensor, the header files, dynamic libraries, and dependencies are installed in the install/rgbd_sensor path.

## Compilation
Supports two methods for compilation: compiling on an X3 Ubuntu system and cross-compiling using docker on a PC, and supports controlling the dependencies and functions of the compiled pkg through compilation options.

### Compilation on X3 Ubuntu System
1. Compilation Environment Verification
- X3 Ubuntu system is installed on the board.
- The current compilation terminal has set up the TogetherROS environment variables: `source PATH/setup.bash`, where PATH is the installation path of TogetherROS.
- ROS2 build tool colcon is installed, installation command: `pip install -U colcon-common-extensions`
- Dependendencies are installed, detailed in the Dependency section

2. Compilation:
   `colcon build --packages-select rgbd_sensor`.

### Docker Cross-Compilation
1. Compilation Environment Verification
- Compilation in docker, with tros already installed in docker. Details on docker installation, cross-compilation instructions, tros compilation, and deployment can be found in the README.md in the robot development platform robot_dev_config repo.
- hbm_img_msgs package has been compiled (compilation method in the Dependency section)

2. Compilation- Compilation Command:

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select rgbd_sensor \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  ```

# Usage

## Current Parameter List:

| Parameter Name   | Meaning              | Value Options                                   | Default Value        |
| ---------------- | -------------------- | ---------------------------------------------- | --------------------- |
| sensor_type      | Device Type          | String, currently only supports Sunplus CP3AM  | CP3AM                 |
| io_method        | Output Data Transfer Method | ros/shared_mem                              | ros                   |
| color_width      | Module Output Image Width     | 1920                                           | 1920                  |
| color_height     | Module Output Image Height    | 1080                                           | 1080                  |
| color_fps        | Module Output Image Frame Rate | 10                                           | 10                    |
| enable_color     | Enable Image Publishing        | True/False                                     | True                  |
| depth_width      | Module Output Depth Image Width | 224                                          | 224                   |
| depth_height     | Module Output Depth Image Height | 129                                          | 129                   |
| depth_fps        | Module Output Depth Image Frame Rate | 10                                      | 10                    |
| enable_depth     | Enable Depth Image Publishing   | True/False                                     | True                  |
| enable_pointcloud | Enable Point Cloud Publishing  | True/False                                     | True                  |
| enable_aligned_pointcloud | Enable Aligned Point Cloud Publishing | True/False                        | True                  |
| infra_width      | Module Output Infrared Image Width | 224                                          | 224                   |
| infra_height     | Module Output Infrared Image Height | 108                                          | 108                   |
| infra_fps        | Module Output Infrared Image Frame Rate | 10                                      | 10                    |
| enable_infra     | Enable Infrared Image Publishing | True/False                                    | True                  |
| camera_calibration_file_path | Path to Camera Calibration File | Set according to the actual path of the camera calibration file | /opt/tros/${TROS_DISTRO}/lib/rgbd_sensor/config/CP3AM_calibration.yaml |

Currently, Sunplus modules can only output 1080P calibration, so the image parameters have no practical effect, all using default values.
Published topics include:
```
#ros
#depth image
``````
/rgbd_CP3AM/depth/image_rect_raw
# Point Cloud
/rgbd_CP3AM/depth/color/points
# Calibrated Point Cloud
/rgbd_CP3AM/aligned_depth_to_color/color/points
# Grayscale Image
/rgbd_CP3AM/infra/image_rect_raw
# Color Image
/rgbd_CP3AM/color/image_rect_raw
# Camera Parameters
/rgbd_CP3AM/color/camera_info
# shared mem:
# Color Image
hbmem_img
# Depth Image
hbmem_depth
# Grayscale Image
hbmem_infra

## Note:
1: In the current directory cp -r install/${PKG_NAME}/lib/${PKG_NAME}/parameter/ ., where ${PKG_NAME} is the specific package name.

2: The calibration library install/lib/libgc2053_linear.so needs to be copied to: /lib/sensorlib/

3: If reading camera parameters file fails, a warning about unable to publish camera parameters will appear, but it does not affect other functions of rgbd_sensor.

4: The default path for camera calibration file is: /opt/tros/${TROS_DISTRO}/lib/rgbd_sensor/config/CP3AM_calibration.yaml. Please make sure the file path is correct, and the topic name for camera parameters publication is: /rgbd_CP3AM/color/camera_info

## X3 Ubuntu System
Users can simply start by calling ros2 command:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh

ros2 run rgbd_sensor rgbd_sensor
```

Passing arguments example:
`ros2 run rgbd_sensor rgbd_sensor --ros-args --log-level info --ros-args -p io_method:=ros`

Another way to run is using launch file:
`ros2 launch install/share/rgbd_sensor/launch/rgbd_sensor.launch.py`

## X3 Linaro System
Copy the install directory compiled in docker to the Linaro system, for example: /userdata
First, specify the path for dependent libraries, for example:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`
```Please modify the path of ROS_LOG_DIR, otherwise it will be created under the /home directory, you need to execute "mount -o remount,rw /" to be able to create logs under /home.
`export ROS_LOG_DIR=/userdata/`

Run rgbd_sensor
```
// Default parameter method
/userdata/install/lib/rgbd_sensor/rgbd_sensor
// Parameter passing method
#/userdata/install/lib/rgbd_sensor/rgbd_sensor --ros-args -p io_method:=ros

```
# Result analysis
## X3 result display
If the camera runs normally and successfully publishes camera parameters, the following information will be output
```
[INFO] [1662470301.451981102] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.461459373] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.528991765] [rgbd_node]: publish camera info.

[INFO] [1662470301.533941164] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.543212650] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.608683161] [rgbd_node]: publish camera info.

[INFO] [1662470301.613038915] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.621678967] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.713615364] [rgbd_node]: publish camera info.

[INFO] [1662470301.717811416] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.726191436] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.818197996] [rgbd_node]: publish camera info.

[INFO] [1662470301.822495336] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.831028892] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.909490084] [rgbd_node]: publish camera info.

[INFO] [1662470301.913704051] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.922111820] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470302.014477201] [rgbd_node]: publish camera info.
```

## View the effect in rviz2
Install: apt install ros-foxy-rviz-common ros-foxy-rviz-default-plugins ros-foxy-rviz2