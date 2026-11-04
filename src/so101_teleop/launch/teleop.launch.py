"""Bring up the whole teleop system with one command.

    ros2 launch so101_teleop teleop.launch.py follower_port:=COM5 leader_port:=COM6
    ros2 launch so101_teleop teleop.launch.py ... haptic:=true camera:=true hapkit_port:=COM7

The follower is written out as an example of the syntax; the rest is yours.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition  # noqa: F401  (you'll want this for optional nodes)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument("follower_port", description="COM port of the 12 V arm (find_ports)"),
        DeclareLaunchArgument("leader_port", description="COM port of the 5 V arm (find_ports)"),
        # TODO(you): arguments for haptic, camera, camera_index, hapkit_port

        # Example: the namespace turns the node's relative topics into /follower/joint_states etc.
        Node(package="so101_teleop", executable="so101_driver", name="driver", namespace="follower",
             output="screen",
             parameters=[{"port": arg("follower_port"), "can_move": True, "hold_on_start": True,
                          # teleop feel from haptic_teleop.py (LeRobot's values; P=32 overshoots)
                          "p_gain": 16, "d_gain": 32, "speed": 2000}]),

        # TODO(you):
        #   - leader driver (namespace "leader"): read-only unless haptic; with haptic it needs
        #     can_move, release_on_exit, and a soft feel (your old LEADER_P_GAIN / TORQUE_LIMIT)
        #   - teleop
        #   - camera, only if camera:=true      -> condition=IfCondition(arg("camera"))
        #   - hapkit, only if a port was given  (hint: launch.substitutions.PythonExpression)
    ])
