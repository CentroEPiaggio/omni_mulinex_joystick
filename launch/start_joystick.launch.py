import os
import subprocess
from datetime import datetime

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node



def generate_launch_description():
    joy_node_path = get_package_share_path("omni_mulinex_joystick")

    joy_cfg_file = os.path.join(joy_node_path,"config","joy_node.yaml")
    # bag_name = f"bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    bag_dir = os.path.expanduser("~/mulinex_ws/bag")
    os.makedirs(bag_dir, exist_ok=True)

    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    bag_path = os.path.join(bag_dir, f"Test_joystick_{timestamp}")

    subprocess.check_output(
        ["ros2 control load_controller omni_controller --set-state active "]
        ,shell=True)
    # COmmentare i 2 controllori sotto se utilizzi interfaccia new_mulsbc_ws
    # subprocess.check_output(
    #     ["ros2 control load_controller state_broadcaster --set-state active"]
    #     ,shell=True)

    # subprocess.check_output(
    #     ["ros2 control load_controller distributor_state_broadcaster --set-state active"]
    #     ,shell=True)


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

    bag_record = ExecuteProcess(
        # cmd=["ros2", "bag", "record", "-a", "-s", "mcap", "-o", bag_name],
        cmd=["ros2", "bag", "record", "-a", "-s", "mcap", "-o", bag_path],
        output="screen"
    )
    
    return LaunchDescription([
        joy_event_node,
        joy_node,
        bag_record
    ])
