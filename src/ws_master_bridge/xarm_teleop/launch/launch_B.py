"""
launch_B.py  —  LAPTOP B
==========================
Starts:
  1. udp_bridge_B  — ESP32 B → ROS2 (pose in, STOP in)
  2. slave_node    — replicates master pose on xArm B

Usage:
  ros2 launch xarm_teleop launch_B.py
  ros2 launch xarm_teleop launch_B.py listen_port:=9001
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument("listen_port", default_value="9001"),

        Node(
            package="xarm_teleop",
            executable="udp_bridge_B",
            name="udp_bridge_B",
            parameters=[{
                "listen_port": LaunchConfiguration("listen_port"),
            }],
            output="screen",
        ),

        Node(
            package="xarm_teleop",
            executable="slave_node",
            name="slave_node",
            output="screen",
        ),
    ])
