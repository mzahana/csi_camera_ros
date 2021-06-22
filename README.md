# csi_camera_ros
A ROS package that interfaces CSI cameras (e.g. Raspberry Pi camera) to ROS. Tested on Xavier NX

# Prerequisites
* ROS Melodic & Ubuntu 18
* OpenCV
* Python 2.7
* cv_bridge

# Testing
```
roslaunch csi_cam_ros run_csi_cam.launch
```
Check the launch file `run_csi_cam.launch` for configurable parameters

Tested witha RPi camera v2 on Xavier NX

# TODO
* Implement camera intrinsic params topic (CamInfo)

