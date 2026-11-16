"""Touch leader node + CHAI3D window.

    ros2 launch touch_driver touch.launch.py                      # read-only: only zero force is ever sent
    ros2 launch touch_driver touch.launch.py enable_force:=true   # forces on, limits from touch.yaml

The node builds its kinematics and window meshes from the SO-101 URDF.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = Path(get_package_share_directory("so101_description"))
    mine = Path(get_package_share_directory("touch_driver"))
    urdf = (desc / "urdf" / "so101.urdf").read_text(encoding="utf-8")

    return LaunchDescription([
        DeclareLaunchArgument("enable_force", default_value="false",
                              description="true = the Touch may push (limits from touch.yaml)"),
        DeclareLaunchArgument("max_joint_speed", default_value="2.0",
                              description="rad/s cap on the arm target (node clamps to 0.1-4.0)"),
        DeclareLaunchArgument("wall_mode", default_value="proxy",
                              description="start mode: proxy or cbf ([c] in the window switches)"),
        Node(
            package="touch_driver",
            executable="touch_node",
            name="touch_driver",
            output="screen",
            parameters=[
                str(mine / "config" / "touch.yaml"),
                {"robot_description": urdf,
                 "description_dir": str(desc),  # meshes for the window
                 "enable_force": ParameterValue(LaunchConfiguration("enable_force"), value_type=bool),
                 "max_joint_speed": ParameterValue(LaunchConfiguration("max_joint_speed"), value_type=float),
                 "wall.mode": LaunchConfiguration("wall_mode")},
            ],
        ),
    ])
