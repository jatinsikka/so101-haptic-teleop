"""Bus safety rules for the gripper PoC. Rules live here so every script shares them.

Rule 1: a read-only script opens the bus through ReadOnlyPortHandler. It inspects each outgoing
        packet at writePort, the one call that puts bytes on the wire, so a bug anywhere above
        it (ours or the SDK's) cannot send a write.
Rule 2: any write to register 40 (Torque_Enable) must first pass check_torque_enable_value().
        Only 0 and 1 are legal. Feetech's CalibrationOfs writes 128 to this same register, which
        recentres the servo's position, so a stray value here can have a lasting effect.
"""

from scservo_sdk import PortHandler

TORQUE_ENABLE_ADDR = 40

# Instruction codes from scservo_sdk/scservo_def.py.
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
READ_ONLY_INSTRUCTIONS = frozenset({INST_PING, INST_READ})

# Packet layout: FF FF <id> <length> <instruction> <params...> <checksum>
_PKT_ID = 2
_PKT_LENGTH = 3
_PKT_INSTRUCTION = 4
_PKT_PARAMS = 5
_MIN_PACKET_LENGTH = 6

# Registers 40, 42 and 48 are listed as SRAM by Feetech's SMS_STS.h and by LeRobot v0.6.1 tables.py.
GOAL_POSITION_ADDR = 42
TORQUE_LIMIT_ADDR = 48

# Values read from this gripper's registers 9, 11 and 16.
MIN_GOAL_TICK = 2035
MAX_GOAL_TICK = 3463
MAX_TORQUE_LIMIT = 500


class WriteBlockedError(RuntimeError):
    """A packet or value broke a safety rule. Nothing was sent to the servo."""


def check_torque_enable_value(value):
    """Return value if it is a legal Torque_Enable write (exactly int 0 or 1), else raise."""
    if type(value) is not int or value not in (0, 1):
        raise WriteBlockedError(
            f"register {TORQUE_ENABLE_ADDR} only accepts 0 or 1, got {value!r}"
        )
    return value


def check_goal_position(value):
    if not MIN_GOAL_TICK <= value <= MAX_GOAL_TICK:
        raise WriteBlockedError(
            f"goal {value} is outside {MIN_GOAL_TICK}-{MAX_GOAL_TICK}"
        )
    return value


def check_torque_limit(value):
    if not 0 <= value <= MAX_TORQUE_LIMIT:
        raise WriteBlockedError(f"torque limit {value} is outside 0-{MAX_TORQUE_LIMIT}")
    return value


# register: (size in bytes, value check)
_ALLOWED_WRITES = {
    TORQUE_ENABLE_ADDR: (1, check_torque_enable_value),
    GOAL_POSITION_ADDR: (2, check_goal_position),
    TORQUE_LIMIT_ADDR: (2, check_torque_limit),
}


class ReadOnlyPortHandler(PortHandler):
    """A PortHandler that only transmits PING and READ packets addressed to one servo."""

    def __init__(self, port_name, allowed_id):
        super().__init__(port_name)
        self.allowed_id = allowed_id

    def writePort(self, packet):
        try:
            self._check_packet(packet)
        except WriteBlockedError:
            # The SDK sets is_using before calling here and clears it only after a reply. Left set,
            # every later packet fails as "port busy", including the torque-off write on exit.
            self.is_using = False
            raise
        return super().writePort(packet)

    def _check_packet(self, packet):
        if len(packet) < _MIN_PACKET_LENGTH or packet[0] != 0xFF or packet[1] != 0xFF:
            raise WriteBlockedError(f"malformed packet blocked: {list(packet)}")
        if packet[_PKT_ID] != self.allowed_id:
            raise WriteBlockedError(
                f"packet to ID {packet[_PKT_ID]} blocked; only ID {self.allowed_id} is allowed"
            )
        if packet[_PKT_INSTRUCTION] not in READ_ONLY_INSTRUCTIONS:
            raise WriteBlockedError(
                f"instruction {packet[_PKT_INSTRUCTION]} blocked; this port is read-only"
            )


class GripperPortHandler(ReadOnlyPortHandler):
    """Adds the three writes the gripper loop needs. Every other write stays blocked.

    Registers 0-39 (EEPROM) and 55 (Lock) can never be written through this port. Each write must
    use the register's exact size, so a longer write cannot spill into a neighbouring register.
    """

    def _check_packet(self, packet):
        if len(packet) < _MIN_PACKET_LENGTH or packet[_PKT_INSTRUCTION] != INST_WRITE:
            super()._check_packet(packet)
            return

        if packet[0] != 0xFF or packet[1] != 0xFF or packet[_PKT_ID] != self.allowed_id:
            raise WriteBlockedError(f"write blocked, bad header or ID: {list(packet)}")
        if len(packet) != packet[_PKT_LENGTH] + 4:
            raise WriteBlockedError(f"write blocked, inconsistent length: {list(packet)}")

        addr = packet[_PKT_PARAMS]
        data = list(packet[_PKT_PARAMS + 1:-1])
        rule = _ALLOWED_WRITES.get(addr)
        if rule is None:
            raise WriteBlockedError(f"write to register {addr} blocked")
        size, check = rule
        if len(data) != size:
            raise WriteBlockedError(f"register {addr} needs {size} bytes, got {len(data)}")
        check(sum(byte << (8 * i) for i, byte in enumerate(data)))
