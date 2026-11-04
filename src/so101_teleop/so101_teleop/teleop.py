"""teleop -- leader -> follower mapping, with optional force feedback (ROS version of haptic_teleop.py).

Interface:
  Subscribes  /leader/joint_states      operator command (SO-101 leader now; Touch + IK later,
                                        remapped onto this same topic)
              /follower/joint_states    follower position + load (effort)
              /hapkit/gripper_cmd       std_msgs/Float64 rad     (only if gripper_source = hapkit)
  Publishes   /follower/joint_commands  follower goals
              /leader/joint_commands    leader goals for force feedback; NaN = limp joint
              /hapkit/force_cmd         std_msgs/Float64 N       (only if gripper_source = hapkit)

Parameters (suggested defaults from haptic_teleop.py)
  rate_hz 50.0 | max_step_rad 0.10 | haptic false | deadband_rad 0.08 | stopped_rad 0.005
  gripper_source "leader" | hapkit_force_per_load 4.0 | stale_timeout_s 0.25

Structure: subscriber callbacks only store the latest message; a timer runs the control law.
That keeps the loop at a fixed rate whatever order messages arrive in.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from so101_teleop.arm import JOINTS, LIMITS

GRIPPER = 5  # index of the gripper in JOINTS


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")
        # TODO(you), milestone 4: parameters, subscriptions, publishers, timer -> self.loop
        # (hapkit topics only when gripper_source == "hapkit", milestone 7)
        raise NotImplementedError("Teleop.__init__")

    def loop(self):
        """TODO(you), milestone 4:
          1. wait until you have both leader and follower data
          2. safety: if the newest leader message is older than stale_timeout_s, send nothing
             (the follower then holds its last goal). Think: what happens if you DON'T do this
             and the leader node crashes?
          3. target = leader position (milestone 7: gripper from the Hapkit instead), clamp to LIMITS
          4. rate limit: goal = q_follower + clip(target - q_follower, -max_step, +max_step)
          5. publish the goal on /follower/joint_commands
          milestone 5: if haptic: contact = self.contact_mask(...); self.send_leader_feedback(contact)
          milestone 7: self.send_hapkit_feedback()
        """

    # ---- force feedback: port your own functions from haptic_teleop.py ----
    def contact_mask(self, q_cmd, q_follower, q_follower_prev):
        """Bool (6,): which follower joints are in contact.
        TODO(you), milestone 5: same idea as your contact_mask(). A joint that is lagging but still
        moving is NOT in contact; a joint that is stuck and far from its command is."""
        raise NotImplementedError

    def haptic_target(self, q_leader, q_follower, contact):
        """Leader goal (6,): your haptic_target() -- pull toward the follower by the gap beyond
        the deadband, on contact joints only."""
        raise NotImplementedError

    def send_leader_feedback(self, contact):
        """TODO(you), milestone 5: publish haptic_target() on /leader/joint_commands, with NaN for
        joints not in contact (so they coast instead of dragging). Clamp to LIMITS first."""

    def send_hapkit_feedback(self):
        """TODO(you), milestone 7: squeeze force (N) from the follower gripper's load.
        Start simple: force = gain * (load beyond a small deadband). Feel it, then improve it."""


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
