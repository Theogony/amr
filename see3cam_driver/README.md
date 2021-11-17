# see3cam_driver

## Overview

This is the ROS driver for the ON Semi AR0234CS image sensor camera from E-Con systems: see3cam_24cug. This packages sit on top of [usb_cam](https://github.com/ros-drivers/usb_cam), processing the raw image. 

ROS Version: melodic

## Installation
TBD

### Building from Source
TBD

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [usb_cam](https://github.com/ros-drivers/usb_cam)
- [image_pipeline](https://github.com/ros-perception/image_pipeline)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone TBD.git
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

	$ roslaunch see3cam_driver raw_publisher.launch video_device:=/dev/video0
     ## line below is included in the first launchg file
     $ roslaunch see3cam_driver image_proc.launch

## Config files

see3cam_driver/config

* **heard_camera.yaml** estimated camera parameters using fisheye model
* **ost.yaml** estimated camera parameters using plumb model



## Launch files

* **see3cam-test.launch:** For debugging. Shows raw feed.

     Arguments

     - **`video_device`** Filepath to raw video.
     Default: `/dev/video0`.

* **raw_publisher.launch:** Publishes raw image as ROS topic.

     Arguments

     - **`video_device`** Filepath to raw video.
     Default: `/dev/video0`.

* **image_proc.launch:** Takes raw image and outputs processed image as rostopic.



## Nodes

### usb_cam 

#### Published Topics

* **`/usb_cam/camera_info`** ([sensor_msgs/CameraInfo])

* **`/usb_cam/image_raw`** ([sensor_msgs/Image])


### image_proc

#### Published Topics

* **`/usb_cam/image_color`** ([sensor_msgs/Image])

* **`/usb_cam/image_rect_color`** ([sensor_msgs/Image])

#### Subscribed Topics

* **`/usb_cam/image_raw`** ([sensor_msgs/Image])

* **`/usb_cam/camera_info`** ([sensor_msgs/CameraInfo])



## Known Issues
- Callibration should be improved by generating more accurate YAML file: https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration


rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.034 image:=/usb_cam/image_raw camera:=/usb_cam


ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
rosrun image_view image_view image:=usb_cam/image_rect_color