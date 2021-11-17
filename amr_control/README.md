# amr_control

## Overview

Controls arm motors using ros service.

ROS Version: melodic

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone TBD.git
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

### Initialze in seperate terminals 

	 $ roscore
	 $ rosrun turtlesim turtlesim_node
     $ roslaunch amr_control amr_control.launch
	 $ rosrun turtlesim turtle_teleop_key

### Example Client
     $ roslaunch amr_control example_client.launch

## Launch files

* **amr_control.launch** Initialize amr control service

     Arguments

	 - **`motor_port_1`** Motor USB filepath.
     Default: `/dev/ttyUSB1`.

	 - **`motor_port_2`** Motor USB filepath.
     Default: `/dev/ttyUSB3`.

* **example_client.launch** Toggles on/off at 3Hz

## Nodes

### amr_driver

#### Service

* **`/amr_motor`** ([std_srvs.srv/SetBool])

## Known Issues

- Conveyor speed hardcoded to 500. May need to update and/or make configurable
