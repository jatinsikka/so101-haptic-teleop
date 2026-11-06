"""Touch -> real SO-101: follower driver + teleop + Touch node, one command.

    ros2 launch so101_teleop touch_teleop.launch.py                      # read-only: arm never moves
    ros2 launch so101_teleop touch_teleop.launch.py move:=true           # arm follows the Touch
    ros2 launch so101_teleop touch_teleop.launch.py move:=true enable_force:=true

port:=COM3 by default (the 12 V follower; check with `ros2 run so101_teleop find_ports`).
First motion runs use a low torque limit (50%) and 1 rad/s max joint speed.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    touch = Path(get_package_share_directory("touch_driver")) / "launch" / "touch.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="COM3"),
        DeclareLaunchArgument("move", default_value="false", description="true = the arm may move"),
        DeclareLaunchArgument("enable_force", default_value="false"),

        Node(package="so101_teleop", executable="so101_driver", name="driver", namespace="follower",
             output="screen",
             parameters=[{"port": arg("port"),
                          "can_move": ParameterValue(arg("move"), value_type=bool),
                          "hold_on_start": ParameterValue(arg("move"), value_type=bool),
                          "release_on_exit": False,   # a follower would drop: hold on Ctrl+C
                          "p_gain": 16, "d_gain": 32, "speed": 2000,  # LeRobot's teleop feel
                          "torque_limit": 500}]),     # 50% for the first runs

        Node(package="so101_teleop", executable="teleop", name="teleop", output="screen",
             parameters=[{"max_step_rad": 0.02,       # 1 rad/s at 50 Hz
                          "engage_tolerance_rad": 0.3,
                          "gripper_source": "hold"}]),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(touch)),
                                 launch_arguments={"enable_force": arg("enable_force")}.items()),
    ])
