"""Print the follower's joint angles and read time. Read-only (can_move=False). Ctrl+C to stop."""
import time

from so101_teleop.arm import Arm

arm = Arm("COM3")
while True:
    t = time.perf_counter()
    q, dq, load = arm.read_state()
    print(q.round(2), f"{(time.perf_counter() - t) * 1000:.1f} ms")
