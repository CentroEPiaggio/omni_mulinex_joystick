import os
import subprocess

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    joy_node_path = get_package_share_path("omni_mulinex_joystick")

    joy_cfg_file = os.path.join(joy_node_path,"config","joy_node.yaml")

    subprocess.check_output(
        ["ros2 control load_controller omni_controller --set-state active "]
        ,shell=True)
    
    subprocess.check_output(
        ["ros2 control load_controller state_broadcaster --set-state active"]
        ,shell=True)

    joy_event_node = Node(
        package="joy",
        executable="joy_node",
        output="screen"
    ) 

    joy_node = Node(
        package="omni_mulinex_joystick",
        executable="omni_mul_joystic_node",
        output="screen",
        parameters=[joy_cfg_file]
    )
    
    return LaunchDescription([
        joy_event_node,
        joy_node
    ])
