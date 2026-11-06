# ros2_ws: haptic teleop of the SO-101 (ROS 2 Jazzy, native Windows)

The Geomagic Touch drives the SO-101 follower. You feel the virtual walls, the reach limits and the contact.
The Hapkit drives the gripper. A wrist camera comes later.

## Packages

| package | what |
|---|---|
| `touch_driver` (C++) | Touch via CHAI3D at 1 kHz: IK, forces, 3D window. See [its README](src/touch_driver/README.md) |
| `so101_teleop` (Python) | servo bus (`arm.py`), follower driver, teleop, tools; Hapkit + camera still TODO |
| `so101_description` | SO-101 URDF + meshes (used by the Touch's IK and the window) |

## Architecture

```
Touch ─CHAI3D 1 kHz─ touch_node ── /leader/joint_states (5) ─► teleop ── /follower/joint_commands (6) ─► follower driver ─► SO-101 (COM3)
                        ▲          /leader/engaged (deadman) ─►                                              │
                        └──────────────────── /follower/joint_states ◄────────────────────────────────────────┘
Hapkit ── /hapkit/gripper_cmd ─► teleop          (not built yet)
```

- **touch_node:** stylus → wrist point (IK) + tilt/twist → 5 joint targets. Computes all the forces.
- **teleop:** forwards the targets only while button 1 is held. Refuses to engage if the target is > 0.3 rad from the real arm. Max 1 rad/s per joint. Stops if the Touch goes quiet for 0.25 s.
- **driver:** the only node on the serial port. Sync read/write at 50 Hz. NaN command = limp. Overheat stop at 55 °C.

## Run

```powershell
. .\ros2_env.ps1                                     # every terminal
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release   # after every edit

ros2 run so101_teleop find_ports                     # follower = 12 V port (COM3), Touch = COM4
ros2 launch so101_teleop touch_teleop.launch.py                      # 1. read-only: arm never moves
ros2 launch so101_teleop touch_teleop.launch.py move:=true           # 2. arm follows (50% torque)
ros2 launch so101_teleop touch_teleop.launch.py move:=true enable_force:=true   # 3. + forces
ros2 run so101_teleop release COM3                   # torque off. SUPPORT THE ARM, it drops.
```

On Ctrl+C the follower keeps holding its pose.

## Debugging

```powershell
ros2 topic hz /follower/joint_states ; ros2 topic echo /leader/engaged
ros2 param list /follower/driver
```

Windows quirks (Smart App Control, exit codes, `install\` vs `src\`) are in [PROGRESS.md](PROGRESS.md#gotchas).
