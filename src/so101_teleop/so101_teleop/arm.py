"""arm.py -- the only file that talks to the SO-101 servos (ROS port of sim2real/arm.py).

Same conventions as the original: joint values in and out are RADIANS, in the sim's joint
order (motor IDs 1-6), and tick 2048 = 0 rad on every joint (LeRobot calibration was done at
the sim zero pose, so the servos' EEPROM offsets already make this true).

Changes from sim2real/arm.py:
  - the port is required (Windows: "COM5" etc.), so one class serves leader and follower;
  - position/speed/load come back in ONE sync-read packet and goals go out in ONE sync-write
    packet. On Windows every USB-serial round trip costs ~1 ms+, so 12 single reads per cycle
    would cap the loop well below 50 Hz.
"""

import math

import numpy as np
from scservo_sdk import (COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler,
                         PortHandler, SCS_HIBYTE, SCS_LOBYTE)
from serial import SerialException

BAUD = 1_000_000
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
IDS = [1, 2, 3, 4, 5, 6]

TICKS_PER_RAD = 4096 / (2 * math.pi)  # 651.9
ZERO_TICK = 2048                      # tick 2048 = the sim's 0 rad on every joint

# Joint limits from the URDF (rad). Every target is clamped to these before it is sent.
LIMITS = np.array([
    [-1.920, 1.920],   # shoulder_pan
    [-1.745, 1.745],   # shoulder_lift
    [-1.690, 1.690],   # elbow_flex
    [-1.658, 1.658],   # wrist_flex
    [-2.744, 2.841],   # wrist_roll
    [-0.175, 1.745],   # gripper
])

MAX_TEMP_C = 55   # stop before the servos' ~70 C shutdown

# STS3215 register addresses
TORQUE_ENABLE, ACCELERATION, GOAL_POSITION, GOAL_SPEED = 40, 41, 42, 46
P_COEFFICIENT, D_COEFFICIENT, TORQUE_LIMIT = 21, 22, 48
PRESENT_POSITION, PRESENT_SPEED, PRESENT_LOAD = 56, 58, 60  # 2 bytes each, contiguous
PRESENT_VOLTAGE, PRESENT_TEMP = 62, 63                      # 1 byte each


def to_rad(ticks):
    return (np.asarray(ticks, dtype=float) - ZERO_TICK) / TICKS_PER_RAD


def to_ticks(rad):
    return np.round(np.asarray(rad, dtype=float) * TICKS_PER_RAD + ZERO_TICK).astype(int)


def _signed(raw, sign_bit):
    """STS registers are sign-magnitude: bit `sign_bit` is the sign."""
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if raw & (1 << sign_bit) else mag


class Arm:
    """Open the bus once, then read state and send targets.

    can_move=False makes every write a no-op, so a node can run read-only without the arm
    ever moving (the leader in plain teleop).
    """

    def __init__(self, port, can_move=False):
        self.port_name = port
        self.can_move = can_move
        self.port = PortHandler(port)
        try:  # the SDK raises (pyserial) instead of returning False when the port is missing/busy
            opened = self.port.openPort() and self.port.setBaudRate(BAUD)
        except SerialException:
            opened = False
        if not opened:
            raise RuntimeError(f"cannot open {port} -- is that arm plugged in (and not open elsewhere)?")
        self.bus = PacketHandler(0)
        # position, speed, load: 6 contiguous bytes starting at PRESENT_POSITION
        self._reader = GroupSyncRead(self.port, self.bus, PRESENT_POSITION, 6)
        for i in IDS:
            self._reader.addParam(i)
        self._writer = GroupSyncWrite(self.port, self.bus, GOAL_POSITION, 2)

    # ---- low level: one register on one servo, retried once (the bus drops a packet now and then)
    def _read(self, motor_id, addr, size):
        fn = self.bus.read1ByteTxRx if size == 1 else self.bus.read2ByteTxRx
        for _ in range(2):
            value, comm, _ = fn(self.port, motor_id, addr)
            if comm == COMM_SUCCESS:
                return value
        raise RuntimeError(f"no reply from servo {motor_id} on {self.port_name} (register {addr})")

    def _write(self, motor_id, addr, size, value):
        if not self.can_move:
            return
        fn = self.bus.write1ByteTxRx if size == 1 else self.bus.write2ByteTxRx
        for _ in range(2):
            comm, _ = fn(self.port, motor_id, addr, int(value))
            if comm == COMM_SUCCESS:
                return
        raise RuntimeError(f"write to servo {motor_id} on {self.port_name} (register {addr}) failed twice")

    # ---- what the nodes use
    def read_state(self):
        """(position rad, velocity rad/s, load -1..1), each shape (6,). One packet on the bus."""
        for _ in range(2):
            if self._reader.txRxPacket() == COMM_SUCCESS:
                pos = [self._reader.getData(i, PRESENT_POSITION, 2) for i in IDS]
                spd = [_signed(self._reader.getData(i, PRESENT_SPEED, 2), 15) for i in IDS]  # bit 15 = sign
                load = [_signed(self._reader.getData(i, PRESENT_LOAD, 2), 10) for i in IDS]  # bit 10 = sign
                return to_rad(pos), np.array(spd) / TICKS_PER_RAD, np.array(load) / 1000.0
        # sync read failed twice: slow per-servo path, positions only (raises if a servo is silent)
        pos = to_rad([self._read(i, PRESENT_POSITION, 2) for i in IDS])
        return pos, np.zeros(6), np.zeros(6)

    def send_targets(self, targets, joints=range(6)):
        """Goal positions (rad) for the joints in `joints`, in ONE packet. Returns the clamped targets.
        Writing a goal turns that servo's torque ON."""
        targets = np.clip(np.asarray(targets, dtype=float), LIMITS[:, 0], LIMITS[:, 1])  # never skip
        if not self.can_move:
            return targets
        ticks = to_ticks(targets)
        self._writer.clearParam()
        for j in joints:
            t = int(ticks[j])
            self._writer.addParam(IDS[j], [SCS_LOBYTE(t), SCS_HIBYTE(t)])
        self._writer.txPacket()
        return targets

    def configure(self, p_gain=None, d_gain=None, speed=None, accel=None, torque_limit=None):
        """Write servo-loop settings to all six servos (RAM, lost on power-off)."""
        for i in IDS:
            if p_gain is not None:
                self._write(i, P_COEFFICIENT, 1, p_gain)
            if d_gain is not None:
                self._write(i, D_COEFFICIENT, 1, d_gain)
            if speed is not None:
                self._write(i, GOAL_SPEED, 2, speed)
            if accel is not None:
                self._write(i, ACCELERATION, 1, accel)
            if torque_limit is not None:
                self._write(i, TORQUE_LIMIT, 2, torque_limit)

    def hold_here(self):
        """Torque on, holding the current pose (goal = here first, so nothing jumps)."""
        q, _, _ = self.read_state()
        self.send_targets(q)
        self.set_torque(range(6), True)
        return q

    def set_torque(self, joints, on):
        for j in joints:
            self._write(IDS[j], TORQUE_ENABLE, 1, int(on))

    def temperatures(self):
        return [self._read(i, PRESENT_TEMP, 1) for i in IDS]

    def voltage(self):
        """Supply voltage seen by servo 1, volts. ~12 V = follower, ~5 V = leader."""
        return self._read(IDS[0], PRESENT_VOLTAGE, 1) / 10.0

    def close(self):
        self.port.closePort()
