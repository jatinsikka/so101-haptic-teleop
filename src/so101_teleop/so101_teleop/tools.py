"""Small command-line helpers (no ROS graph needed).

    ros2 run so101_teleop find_ports     # which COM port is which device
    ros2 run so101_teleop find_cameras   # which OpenCV index is which camera (saves snapshots)
    ros2 run so101_teleop release COM5   # torque off on that arm -- SUPPORT IT, it drops
"""

import sys
from pathlib import Path

from serial.tools import list_ports


def find_ports():
    """Tell the arms apart the same way as real_imitation/README: servo supply ~12 V = follower,
    ~5 V = leader. A port with no servo answering is something else (Hapkit, Touch, ...)."""
    from so101_teleop.arm import Arm
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        if "Touch" in p.description or "Haptic" in p.description:
            print(f"{p.device:<7} {p.description:<45} Geomagic Touch (not probed)")
            continue
        try:
            arm = Arm(p.device)
            try:
                v = arm.voltage()
                role = f"SO-101 {'FOLLOWER' if v > 8 else 'LEADER'} ({v:.1f} V)"
            except RuntimeError:
                role = "no servo answered (Hapkit? follower with 12 V supply off?)"
            arm.close()
        except RuntimeError:
            role = "could not open (in use by another program?)"
        print(f"{p.device:<7} {p.description:<45} {role}")


def find_cameras():
    import cv2
    out = Path.cwd() / "camera_snapshots"
    out.mkdir(exist_ok=True)
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        cap.release()
        if ok:
            path = out / f"camera_{i}.jpg"
            cv2.imwrite(str(path), frame)
            print(f"index {i}: {frame.shape[1]}x{frame.shape[0]}  snapshot -> {path}")
    print("open the snapshots to see which index is the wrist camera")


def release():
    from so101_teleop.arm import Arm
    if len(sys.argv) < 2:
        sys.exit("usage: ros2 run so101_teleop release COM5")
    arm = Arm(sys.argv[1], can_move=True)
    input(f"Torque OFF on {sys.argv[1]}: support the arm, then press Enter...")
    arm.set_torque(range(6), False)
    arm.configure(torque_limit=1000)
    arm.close()
    print("released")
