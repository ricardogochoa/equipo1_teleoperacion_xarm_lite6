"""
launch_A.py  —  LAPTOP A
==========================
Starts:
  1. udp_bridge_A  — ESP32 A ↔ ROS2 (START/STOP in, pose out)
  2. master_node   — circle trajectory on xArm A

Usage:
  ros2 launch xarm_teleop launch_A.py
  ros2 launch xarm_teleop launch_A.py esp32_ip:=192.168.43.XXX
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument("esp32_ip",    default_value="192.168.43.XXX",
                              description="Hotspot IP of ESP32 A"),
        DeclareLaunchArgument("listen_port", default_value="9001"),
        DeclareLaunchArgument("esp32_port",  default_value="9100"),

        Node(
            package="xarm_teleop",
            executable="udp_bridge_A",
            name="udp_bridge_A",
            parameters=[{
                "listen_port": LaunchConfiguration("listen_port"),
                "esp32_ip":    LaunchConfiguration("esp32_ip"),
                "esp32_port":  LaunchConfiguration("esp32_port"),
            }],
            output="screen",
        ),

        Node(
            package="xarm_teleop",
            executable="master_node",
            name="master_node",
            output="screen",
        ),
    ])
