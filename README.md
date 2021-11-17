[![Build badge](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/workflows/Industrial%20CI%20pipeline/badge.svg?branch=master&event=push)](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/actions)

# onsemi industrial robotics 
onsemi products like; image sensors, motor control, battery charging and more are used to create reference systems for our customers.  In turn these reference systems are put together in a Autonomous Mobile Robot (AMR). This repository contains the ROS packages to control the AMR



## Acknowledgment

## How to report an issue

## How to get help


## Contents
This repository contains :

  * **amr_control**: A motor control ROS node that controls 4 independent motors for a Mecanum Wheels AMR.
  * **board_detection**: A object detection node that uses ArUco tags to determine the distance to a surface. Than control a UR3 to pickup PCBs of a specific size.
  * **see3cam_driver**: Driver for the onsemi AR Camera
  * **usb_cam**: Driver for USB interface

## Requirements
This driver requires a system setup with ROS. It is recommended to use **Ubuntu 20.04 with ROS
noetic**.

To make sure that robot control isn't affected by system latencies, it is highly recommended to use
a real-time kernel with the system. See the [real-time setup guide](ur_robot_driver/doc/real_time.md)
on information how to set this up.

## Building

**Note:** The driver consists of a [C++
library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) that abstracts the
robot's interfaces and a ROS driver on top of that. As the library can be built without ROS support,
it is not a catkin package and therefore requires a different treatment when being built inside the
workspace. See The alternative build method below if you'd like to build the library from source.

If you don't want to build the library from source, it is available as a binary package through the
ROS distribution of ROS melodic and noetic. It will be installed automatically if you
follow the steps below. If you'd like to also build the library from source, please follow the steps
explained in the [next section](#alternative-all-source-build).

```bash
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```

#### Quick start

## Troubleshooting


