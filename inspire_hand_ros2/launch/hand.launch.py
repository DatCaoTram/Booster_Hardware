"""Start one RH56 hand.

  ros2 launch inspire_hand_ros2 hand.launch.py port:=/dev/serial/by-id/usb-...   # USB-RS485 adapter
  ros2 launch inspire_hand_ros2 hand.launch.py simulate:=true                    # no hardware
  ros2 launch inspire_hand_ros2 hand.launch.py backend:=booster                  # hand on the T1's hand port
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = LaunchConfiguration
    use_booster = PythonExpression(["'", arg("backend"), "' == 'booster'"])
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="left_hand"),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution(
            [FindPackageShare("inspire_hand_ros2"), "config", "left_hand.yaml"])),
        DeclareLaunchArgument("backend", default_value="serial",
                              description="serial = our RS485 driver, booster = T1's built-in hand control"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("hand_id", default_value="1"),
        DeclareLaunchArgument("simulate", default_value="false"),
        Node(package="inspire_hand_ros2", executable="driver", namespace=arg("namespace"), output="screen",
             parameters=[arg("config"),
                         {"port": arg("port"), "hand_id": arg("hand_id"), "simulate": arg("simulate")}],
             condition=UnlessCondition(use_booster)),
        Node(package="inspire_hand_ros2", executable="booster_bridge", namespace=arg("namespace"),
             output="screen", parameters=[arg("config")], condition=IfCondition(use_booster)),
    ])
