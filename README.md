# ros2_ws: haptic teleop of the SO-101 (ROS 2 Jazzy, native Windows)

The Geomagic Touch drives a real SO-101 arm. You feel a virtual wall, and can switch it between a
**proxy wall** (stops at contact) and a **CBF wall** (control barrier function: slows down before contact).
The Hapkit will drive the gripper; a wrist camera comes later.

**New PC?** Follow [SETUP.md](SETUP.md) (ROS 2, Visual Studio, CHAI3D, Touch drivers), then `.\check_setup.ps1`.

## Packages

| package | what |
|---|---|
| `touch_driver` (C++) | Touch via CHAI3D at 1 kHz: IK, walls (proxy / CBF), forces, 3D window. [README](src/touch_driver/README.md) |
| `so101_teleop` (Python) | servo bus (`arm.py`), follower driver, teleop, wall metrics, tools. Hapkit + camera: TODO |
| `so101_description` | SO-101 URDF + meshes |

## Architecture

```
Touch ─1 kHz─ touch_node ── /leader/joint_states ─► teleop ── /follower/joint_commands ─► driver ─► SO-101 (COM3)
                 ▲  │        /leader/engaged      ─►                                         │
                 │  └─ /touch/metrics ─► wall_metrics (CSV)                                  │
                 └─────────────────────────── /follower/joint_states ◄────────────────────────┘
```

- **touch_node:** stylus → arm target (IK) → wall → force at the stylus.
- **teleop:** forwards the target only while button 1 is held; refuses to start > 0.3 rad from the real arm; speed-limited.
- **driver:** the only program on the serial port. 50 Hz, overheat stop at 55 °C.
- **wall_metrics:** measures how the real arm meets the wall (overshoot, sag, approach speed, table hits).

## Run

```powershell
. .\ros2_env.ps1                                                       # every terminal
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release   # after every edit

ros2 launch so101_teleop touch_teleop.launch.py                                   # 1. read-only: arm never moves
ros2 launch so101_teleop touch_teleop.launch.py move:=true enable_force:=true     # 2. arm follows + forces
ros2 launch so101_teleop touch_teleop.launch.py move:=true enable_force:=true fast:=true wall_mode:=cbf
ros2 run so101_teleop release COM3                                     # torque off. SUPPORT THE ARM, it drops.
```

| launch option | default | meaning |
|---|---|---|
| `move` | false | true = the real arm may move |
| `enable_force` | false | true = the Touch may push |
| `fast` | false | true = 3 rad/s, servo P 24 / D 16 (normal: 1 rad/s, P 16 / D 32) |
| `wall_mode` | proxy | start in `proxy` or `cbf` (key `c` switches) |
| `p_gain`, `d_gain`, `torque_limit` | profile | servo overrides, e.g. `d_gain:=12` |

Keys in the window: `c` mode, `1`–`5` CBF α, `w` wall, `t` tap, `q` quit ([full list](src/touch_driver/README.md#run)).
On Ctrl+C the follower keeps holding its pose. Servo gains stay in the servos until the next launch or power-off.

## Demo

1. **PROXY** (red wall): move down fast. The arm hits the wall at full speed; you feel it only at contact, with a tap.
2. Release, press **`c`** → **CBF** (blue wall): same move. The arm slows down before the wall; you feel it building.
3. Hover above the wall, press **`1`** (α 2: floats down) then **`5`** (α 50: almost proxy).

## Measure it

```powershell
ros2 run so101_teleop wall_metrics --ros-args -p table_gap_mm:=3.0    # second terminal; Ctrl+C = summary
```

Do the same quick downward move ~10× per setting (PROXY, then CBF at a few α), resting ~0.5 s on the wall.
It prints one row per approach and a summary per mode / α, and saves two CSVs in the current folder.

| column | meaning |
|---|---|
| overshoot | dynamic: real tip dips below where it then settles |
| sag | static: real arm sits below its command (gravity), same in both modes |
| cmd min | lowest commanded height (≥ 0: the command never entered the wall) |
| table hits | real tip more than `table_gap_mm` below the wall |
| speed | real tip's fastest downward speed in the last 10 mm |

## Debugging

```powershell
ros2 topic hz /follower/joint_states ; ros2 topic echo /leader/engaged ; ros2 topic echo /touch/metrics
```

Windows quirks (Smart App Control, exit codes, `install\` vs `src\`) are in [PROGRESS.md](PROGRESS.md#gotchas).
