"""so101_driver -- one SO-101 arm as a ROS node. Run one per arm (namespaces: leader, follower).

Interface (keep this, other nodes rely on it):
  Publishes   joint_states    sensor_msgs/JointState  name=JOINTS, position rad, velocity rad/s,
                                                      effort = load -1..1
  Subscribes  joint_commands  sensor_msgs/JointState  6 positions, rad, JOINTS order;
                                                      NaN = torque OFF for that joint (limp)
  Topic names are relative ("joint_states", not "/joint_states") so the namespace applies.

Parameters
  port           "COM5" etc.                    rate_hz        50.0
  can_move       false = never write to the arm  hold_on_start  torque on at current pose at start
  p_gain, d_gain, speed, torque_limit   servo settings written at start, -1 = leave as is
  release_on_exit  torque off on Ctrl+C (leader yes; follower no, it would drop)

Design rule: this node is the ONLY thing that touches its serial port. The subscriber callback
just stores the latest command; the timer callback does all bus reads and writes. Two
callbacks writing to the bus at once would garble packets.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_teleop.arm import JOINTS, MAX_TEMP_C, Arm


class So101Driver(Node):
    def __init__(self):
        super().__init__("so101_driver")
        # TODO(you), milestone 2:
        #   - declare the parameters above (self.declare_parameter(name, default).value)
        #   - self.arm = Arm(port, can_move=...); apply servo settings with self.arm.configure(...)
        #   - if can_move and hold_on_start: self.arm.hold_here()
        #   - publisher "joint_states", subscriber "joint_commands" -> self.on_command
        #   - timer at rate_hz -> self.loop
        #   - log the port and self.arm.voltage() so you can see which arm this is
        raise NotImplementedError("So101Driver.__init__")

    def on_command(self, msg):
        # TODO(you), milestone 3: check there are 6 positions, store them (numpy array) for loop()
        pass

    def loop(self):
        # TODO(you):
        #   milestone 2: read_state() -> fill and publish a JointState (stamp the header!)
        #   milestone 3: if a new command is stored:
        #       - joints with NaN -> set_torque off (only if currently on: bus writes cost time)
        #       - the rest -> send_targets(..., joints=<finite ones>)
        #   milestone 6: every second, check temperatures() against MAX_TEMP_C; on overheat
        #       stop applying commands and log an error
        pass

    def shutdown(self):
        # TODO(you): if can_move and release_on_exit: torque off and restore torque_limit=1000
        #   (it's a RAM setting: a low limit would stay until power-off). Then close the port.
        self.arm.close()


def main():
    rclpy.init()
    node = So101Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
