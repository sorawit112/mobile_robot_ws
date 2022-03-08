from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stage_ros2',
            namespace='',
            executable='stage_ros2',
            name='stage_sim',
            parameters = [
                {'world':'/home/trainee/ros2_ws/mobile_robot_ws/src/utility/stage_sim/world/maze2.world'},
            ],
            remappings=[
                ("/stage_sim/odom", "/odom"),
                ("/stage_sim/cmd_vel", "/cmd_vel"),
                ("/stage_sim/ranger_0", "/scan"),
                ("robot/base_footprint", "base_footprint"),
                ("robot/base_link", "base_link"),
                ("robot/ranger_0/base_scan", "laser"),
                ("robot/odom", "odom")
            ]
            )
    ])