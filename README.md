# Omni Mulinex Joystick

## Overview
The package to use the joystick to pilote Omni Mulinex 1.1 Version

## Usage
Copy the `omni_mulinex_joystick` folder inside src directory, then compile and source the workspace with
```shell
colcon build --symlink-install && . install/setup.bash
```

To use the joystick, run the controller on the robot, then open a local terminal and run the following commands:
```shell
export ROS_DOMAIN_ID=#ROBOT_ID
```
```shell
ros2 launch  omni_mulinex_joystick start_joystick.launch.py
```
If the command does not run try to execute `sudo su` after the colcon command. 

REMEMBER TO LOOK AT THE PATH WHERE THE BAGS ARE SAVED AND WHERE THE CSV FILES ARE STORED.

The Matlab file for the velocity profile generation is inside the csv files folder.