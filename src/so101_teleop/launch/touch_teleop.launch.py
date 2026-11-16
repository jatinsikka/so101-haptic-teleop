"""Touch -> real SO-101: follower driver + teleop + Touch node, one command.

    ros2 launch so101_teleop touch_teleop.launch.py                      # read-only: arm never moves
    ros2 launch so101_teleop touch_teleop.launch.py move:=true           # arm follows the Touch
    ros2 launch so101_teleop touch_teleop.launch.py move:=true enable_force:=true
    ros2 launch so101_teleop touch_teleop.launch.py move:=true enable_force:=true fast:=true
    ros2 launch so101_teleop touch_teleop.launch.py move:=true enable_force:=true fast:=true wall_mode:=cbf

port:=COM3 by default (the 12 V follower; check with `ros2 run so101_teleop find_ports`).
Torque limit 50% in both profiles.
  normal: 1 rad/s per joint, servo P 16 / D 32 (heavily damped: no overshoot)
  fast:   3 rad/s per joint, servo P 24 / D 16, target up to 4 rad/s (reactive, overshoots a little)
Servo gains can be overridden live (0-254):  p_gain:=64 d_gain:=0 torque_limit:=600
  P = pull toward the goal (higher = faster, more overshoot). D = braking (lower = more overshoot).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROFILES = {  # servo P/D/speed/torque, teleop step per tick (50 Hz), Touch target speed
    "normal": {"p_gain": 16, "d_gain": 32, "speed": 2000, "torque_limit": 500,
               "max_step_rad": 0.02, "max_joint_speed": "2.0"},
    "fast":   {"p_gain": 24, "d_gain": 16, "speed": 3000, "torque_limit": 500,
               "max_step_rad": 0.06, "max_joint_speed": "4.0"},
}


def nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    move = get("move") == "true"
    p = dict(PROFILES["fast" if get("fast") == "true" else "normal"])
    for k, hi in (("p_gain", 254), ("d_gain", 254), ("torque_limit", 1000)):  # live overrides
        if get(k):
            p[k] = min(max(int(get(k)), 0), hi)
    print(f"[touch_teleop] servo P {p['p_gain']} D {p['d_gain']} speed {p['speed']} "
          f"torque {p['torque_limit'] / 10:.0f}% | teleop {p['max_step_rad'] * 50:.1f} rad/s")
    touch = Path(get_package_share_directory("touch_driver")) / "launch" / "touch.launch.py"
    return [
        Node(package="so101_teleop", executable="so101_driver", name="driver", namespace="follower",
             output="screen",
             parameters=[{"port": get("port"), "can_move": move, "hold_on_start": move,
                          "release_on_exit": False,   # a follower would drop: hold on Ctrl+C
                          "p_gain": p["p_gain"], "d_gain": p["d_gain"], "speed": p["speed"],
                          "torque_limit": p["torque_limit"]}]),
        Node(package="so101_teleop", executable="teleop", name="teleop", output="screen",
             parameters=[{"max_step_rad": p["max_step_rad"], "engage_tolerance_rad": 0.3,
                          "gripper_source": "hold"}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(touch)),
                                 launch_arguments={"enable_force": get("enable_force"),
                                                   "max_joint_speed": p["max_joint_speed"],
                                                   "wall_mode": get("wall_mode")}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="COM3"),
        DeclareLaunchArgument("move", default_value="false", description="true = the arm may move"),
        DeclareLaunchArgument("enable_force", default_value="false"),
        DeclareLaunchArgument("fast", default_value="false", description="true = reactive profile (overshoots)"),
        DeclareLaunchArgument("wall_mode", default_value="proxy", description="start mode: proxy or cbf ([c] switches)"),
        DeclareLaunchArgument("p_gain", default_value="", description="servo P override, 0-254"),
        DeclareLaunchArgument("d_gain", default_value="", description="servo D override, 0-254"),
        DeclareLaunchArgument("torque_limit", default_value="", description="override, 0-1000 (= 0-100%)"),
        OpaqueFunction(function=nodes),
    ])
