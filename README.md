# Omni Mulinex Joystick

## Overview

The package to use the joystick to pilote Omni Mulinex 1.1 Version

## Usage

To use the joystick, run the controller on the robot, then open a local terminal and run the following commands:
```shell
ros2 launch  omni_mulinex_joystick start_joystick.launch.py
```
If the command does not run try to execute `sudo su` after the colcon command. 

REMEMBER TO LOOK AT THE PATH WHERE THE BAGS ARE SAVED AND WHERE THE CSV FILES ARE STORED.

The Matlab file for the velocity profile generation is inside the csv files folder.