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
READ_ONLY_INSTRUCTIONS = frozenset({INST_PING, INST_READ})

# Packet layout: FF FF <id> <length> <instruction> <params...> <checksum>
_PKT_ID = 2
_PKT_INSTRUCTION = 4
_MIN_PACKET_LENGTH = 6


class WriteBlockedError(RuntimeError):
    """A packet or value broke a safety rule. Nothing was sent to the servo."""


def check_torque_enable_value(value):
    """Return value if it is a legal Torque_Enable write (exactly int 0 or 1), else raise."""
    if type(value) is not int or value not in (0, 1):
        raise WriteBlockedError(
            f"register {TORQUE_ENABLE_ADDR} only accepts 0 or 1, got {value!r}"
        )
    return value


class ReadOnlyPortHandler(PortHandler):
    """A PortHandler that only transmits PING and READ packets addressed to one servo."""

    def __init__(self, port_name, allowed_id):
        super().__init__(port_name)
        self.allowed_id = allowed_id

    def writePort(self, packet):
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
        return super().writePort(packet)
