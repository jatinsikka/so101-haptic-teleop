"""so101_driver -- one SO-101 arm as a ROS node (run in a namespace, e.g. follower).

Publishes   joint_states    JointState  JOINTS order; position rad, velocity rad/s, effort = load -1..1
Subscribes  joint_commands  JointState  6 positions rad, JOINTS order; NaN = torque off (limp)
Topics are relative, so the namespace applies (/follower/joint_states).

Parameters  port "COM3" | rate_hz 50 | can_move false (= never writes) | hold_on_start
            p_gain, d_gain, speed, torque_limit (-1 = leave as is) | release_on_exit

Only the timer touches the serial port; the subscriber just stores the latest command.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_teleop.arm import JOINTS, MAX_TEMP_C, Arm


class So101Driver(Node):
    def __init__(self):
        super().__init__("so101_driver")
        port = self.declare_parameter("port", "COM3").value
        rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.can_move = bool(self.declare_parameter("can_move", False).value)
        hold_on_start = bool(self.declare_parameter("hold_on_start", False).value)
        gains = {k: int(self.declare_parameter(k, -1).value)
                 for k in ("p_gain", "d_gain", "speed", "torque_limit")}
        self.release_on_exit = bool(self.declare_parameter("release_on_exit", False).value)

        self.arm = Arm(port, can_move=self.can_move)  # can_move=False: every write is a no-op
        self.cmd = None             # latest command (6,), applied once in loop()
        self.torque_on = [False] * 6
        self.overheated = False
        self.ticks = 0
        if self.can_move:
            self.arm.configure(**{k: v for k, v in gains.items() if v >= 0})
            if hold_on_start:  # goal = here, then torque on: nothing jumps
                self.arm.hold_here()
                self.torque_on = [True] * 6

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, "joint_commands", self.on_command, 10)
        self.create_timer(1.0 / rate_hz, self.loop)
        self.get_logger().info(f"{port}: {self.arm.voltage():.1f} V | "
                               f"{'CAN MOVE' if self.can_move else 'read-only'} | {rate_hz:.0f} Hz")

    def on_command(self, msg):
        if len(msg.position) != 6:
            self.get_logger().warn(f"joint_commands needs 6 positions, got {len(msg.position)}",
                                   throttle_duration_sec=2.0)
            return
        self.cmd = np.array(msg.position, dtype=float)

    def loop(self):
        # 1. read + publish
        try:
            q, dq, load = self.arm.read_state()
        except RuntimeError as e:  # a servo didn't answer: skip this tick
            self.get_logger().warn(str(e), throttle_duration_sec=2.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = q.tolist()
        msg.velocity = dq.tolist()
        msg.effort = load.tolist()
        self.pub.publish(msg)

        # 2. overheat check ~1/s -> stop obeying (servos hold their last goal)
        self.ticks += 1
        if self.can_move and self.ticks % 50 == 0 and not self.overheated:
            try:
                temps = self.arm.temperatures()
                if max(temps) >= MAX_TEMP_C:
                    self.overheated = True
                    self.get_logger().error(f"OVERHEAT {temps} C >= {MAX_TEMP_C}: ignoring commands (holding)")
            except RuntimeError:
                pass

        # 3. apply the latest command once
        if self.cmd is None or not self.can_move or self.overheated:
            return
        cmd, self.cmd = self.cmd, None
        live = [j for j in range(6) if np.isfinite(cmd[j])]
        off = [j for j in range(6) if not np.isfinite(cmd[j]) and self.torque_on[j]]
        if off:
            self.arm.set_torque(off, False)
            for j in off:
                self.torque_on[j] = False
        if live:
            self.arm.send_targets(np.where(np.isfinite(cmd), cmd, q), joints=live)  # goal first...
            on = [j for j in live if not self.torque_on[j]]
            if on:  # ...then torque, so a limp joint doesn't jump to an old goal
                self.arm.set_torque(on, True)
                for j in on:
                    self.torque_on[j] = True

    def shutdown(self):
        # follower: keep holding (it would drop). torque_limit lives in RAM, so restore it.
        if self.can_move and self.release_on_exit:
            try:
                self.arm.set_torque(range(6), False)
                self.arm.configure(torque_limit=1000)
            except RuntimeError:
                pass
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
