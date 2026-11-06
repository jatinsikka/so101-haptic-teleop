"""teleop -- Geomagic Touch (leader) -> SO-101 follower. IK and forces live in touch_driver.

  - forwards the Touch target ONLY while /leader/engaged is true
  - won't start if the target is > engage_tolerance_rad from the real arm (no lunges)
  - rate-limits each joint goal (max_step_rad per tick)
  - stops sending if the Touch goes quiet (stale_timeout_s): the follower holds

Subscribes  /leader/joint_states (5 joints by name), /leader/engaged (Bool),
            /follower/joint_states, /hapkit/gripper_cmd (Float64 rad, if gripper_source = hapkit)
Publishes   /follower/joint_commands  6 goals, JOINTS order

Parameters  rate_hz 50 | max_step_rad 0.02 (= 1 rad/s) | engage_tolerance_rad 0.3
            stale_timeout_s 0.25 | gripper_source "hold" or "hapkit"
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from so101_teleop.arm import JOINTS, LIMITS

GRIPPER = 5  # index of the gripper in JOINTS


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.max_step = float(self.declare_parameter("max_step_rad", 0.02).value)
        self.engage_tol = float(self.declare_parameter("engage_tolerance_rad", 0.3).value)
        self.stale_s = float(self.declare_parameter("stale_timeout_s", 0.25).value)
        self.gripper_source = self.declare_parameter("gripper_source", "hold").value

        self.leader = None        # (5,) arm target from the Touch
        self.leader_t = 0.0
        self.engaged = False
        self.follower = None      # (6,) real arm
        self.hapkit_grip = None
        self.goal = None          # (6,) last goal sent
        self.active = False       # True between an accepted engage and the next release

        self.create_subscription(JointState, "/leader/joint_states", self.on_leader, 10)
        self.create_subscription(Bool, "/leader/engaged", self.on_engaged, 10)
        self.create_subscription(JointState, "/follower/joint_states", self.on_follower, 10)
        if self.gripper_source == "hapkit":
            self.create_subscription(Float64, "/hapkit/gripper_cmd", self.on_hapkit, 10)
        self.pub = self.create_publisher(JointState, "/follower/joint_commands", 10)
        self.create_timer(1.0 / self.rate_hz, self.loop)
        self.get_logger().info(f"teleop: max {self.max_step * self.rate_hz:.1f} rad/s per joint, "
                               f"gripper from {self.gripper_source}")

    # ---- callbacks: store only
    def on_leader(self, msg):
        q = dict(zip(msg.name, msg.position))
        if all(j in q for j in JOINTS[:5]):
            self.leader = np.array([q[j] for j in JOINTS[:5]])
            self.leader_t = time.monotonic()

    def on_engaged(self, msg):
        self.engaged = msg.data

    def on_follower(self, msg):
        q = dict(zip(msg.name, msg.position))
        if all(j in q for j in JOINTS):
            self.follower = np.array([q[j] for j in JOINTS])

    def on_hapkit(self, msg):
        self.hapkit_grip = msg.data

    # ---- control law
    def loop(self):
        if self.leader is None or self.follower is None:
            return
        fresh = time.monotonic() - self.leader_t < self.stale_s
        if not (self.engaged and fresh):
            if self.active:
                self.get_logger().info("released (or Touch quiet): follower holds")
            self.active = False
            return  # send nothing: the follower holds its last goal

        if not self.active:  # engage: only if the Touch's target starts where the real arm is
            err = np.abs(self.leader - self.follower[:5])
            if err.max() > self.engage_tol:
                self.get_logger().warn(
                    f"not moving: Touch target is {err.max():.2f} rad from the real arm (joint "
                    f"{JOINTS[int(err.argmax())]}). Is the Touch node hearing /follower/joint_states?",
                    throttle_duration_sec=2.0)
                return
            self.active = True
            self.goal = self.follower.copy()
            self.get_logger().info("engaged: following the Touch")

        target = self.goal.copy()
        target[:5] = self.leader
        if self.gripper_source == "hapkit" and self.hapkit_grip is not None:
            target[GRIPPER] = self.hapkit_grip
        target = np.clip(target, LIMITS[:, 0], LIMITS[:, 1])
        # rate limit relative to the last goal: smooth even if the target jumps
        self.goal = self.goal + np.clip(target - self.goal, -self.max_step, self.max_step)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = self.goal.tolist()
        self.pub.publish(msg)


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
