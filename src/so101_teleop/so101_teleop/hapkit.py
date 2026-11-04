"""hapkit -- Arduino Hapkit as the gripper controller, with squeeze force fed back.

Serial protocol (you own both ends; this is a suggestion, see arduino/hapkit_ros_serial):
  115200 baud, one ASCII line per message
    Arduino -> PC   "P <x>\n"   handle position (~200 Hz)
    PC -> Arduino   "F <f>\n"   force to render, N

Interface:
  Publishes   /hapkit/position     std_msgs/Float64  raw handle position
              /hapkit/gripper_cmd  std_msgs/Float64  follower gripper goal, rad
  Subscribes  /hapkit/force_cmd    std_msgs/Float64  N
Parameters:   port | baud 115200 | handle_min, handle_max | gripper_closed 0.0, gripper_open 1.2 rad
              | max_force (start small, ~2 N)

Things to get right:
  - never block the executor: open with serial.Serial(port, baud, timeout=0) and read what's waiting
  - opening the port resets most Arduinos: wait ~2 s before expecting data
  - lines can arrive split across reads: keep a buffer, only parse complete lines
  - watchdog: if no force_cmd for ~0.2 s, send F 0 (teleop crashed -> the handle must go slack)
  - on shutdown, send F 0 before closing
"""

import rclpy
from rclpy.node import Node


class Hapkit(Node):
    def __init__(self):
        super().__init__("hapkit")
        # TODO(you), milestone 7: parameters, serial port, publishers, subscriber, timer (~200 Hz)
        raise NotImplementedError("Hapkit.__init__")

    def loop(self):
        # TODO(you): read + parse lines -> publish position and the mapped gripper angle;
        # write the (clamped, watchdogged) force
        pass


def main():
    rclpy.init()
    node = Hapkit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
