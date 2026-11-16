# ros2_ws: haptic teleop of the SO-101 (ROS 2 Jazzy, native Windows)

The Geomagic Touch drives a real SO-101 arm, with a virtual fixture (a plane above the table) after
**Bloom et al., IEEE ToH 2026**: **OFF** (no fixture), **IMPEDANCE** (spring wall) or **CBF** (control barrier
function: the arm slows down before the wall; you feel how hard it brakes), plus a "rubber band" tracking force.
The Hapkit will drive the gripper; a wrist camera comes later.

**New PC?** Follow [SETUP.md](SETUP.md) (ROS 2, Visual Studio, CHAI3D, Touch drivers), then `.\check_setup.ps1`.

## Packages

| package | what |
|---|---|
| `touch_driver` (C++) | Touch via CHAI3D at 1 kHz: IK, fixture modes (off / impedance / CBF), forces, 3D window. [README](src/touch_driver/README.md) |
| `so101_teleop` (Python) | servo bus (`arm.py`), follower driver, teleop, wall metrics, tools. Hapkit + camera: TODO |
| `so101_description` | SO-101 URDF + meshes |

## Architecture

```
Touch ─1 kHz─ touch_node ── /leader/joint_states ─► teleop ── /follower/joint_commands ─► driver ─► SO-101 (COM3)
                 ▲  │        /leader/engaged      ─►                                         │
                 │  └─ /touch/metrics ─► wall_metrics (CSV)                                  │
                 └─────────────────────────── /follower/joint_states ◄────────────────────────┘
```

- **touch_node:** stylus → arm target (tracking + CBF filter + IK) → forces at the stylus.
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
| `wall_mode` | impedance | start in `off`, `impedance` or `cbf` (key `c` cycles) |
| `p_gain`, `d_gain`, `torque_limit` | profile | servo overrides, e.g. `d_gain:=12` |

Keys in the window: `c` mode, `f` filter always, `h` rubber band, `↑↓` K, `←→` α, `1`–`5` K/α, `q` quit
([full list](src/touch_driver/README.md#run)).
On Ctrl+C the follower keeps holding its pose. Servo gains stay in the servos until the next launch or power-off.

## Demo

The paper's three conditions, then its main result. Same quick downward move each time; press `c` (released) to switch.

1. **OFF** (grey wall): no fixture. The arm goes straight down and **hits the table**. You feel only the rubber band.
2. **IMPEDANCE** (red): you feel a spring once your hand passes the wall, but **the arm still overshoots** into the
   table. That's the paper's point: an impedance wall needs a violation before it can stop you.
3. **CBF** (blue): the arm **slows down before the wall and stops on it**. You feel damping *before* contact,
   stronger the faster you come in, because the filter is braking harder.
4. **K/α, live** (eq. 35): `4` (K/α 3) force before the wall, `2` (1) at the wall, `1` (0.5) only after you're past it.
   Then `↑↓` and `←→` change K and α separately: higher K = the arm follows more tightly; higher α = it may
   approach the wall faster.
5. *(Optional)* `f`: the CBF filter in every mode, which is how the paper ran its user study. OFF and IMPEDANCE now
   also stop at the wall; only the force differs.

## Measure it

```powershell
ros2 run so101_teleop wall_metrics    # second terminal; Ctrl+C = summary. table_gap_mm defaults to 15 (wall 1.5 cm up)
```

Do the same quick downward move ~10× per setting (OFF, IMPEDANCE, then CBF at a few K/α), resting ~0.5 s at the
bottom. It prints one row per approach and a summary per setting (OFF / IMPEDANCE / CBF K.. a..), and saves two CSVs.

| column | meaning |
|---|---|
| overshoot | dynamic: real tip dips below where it then settles |
| sag | static: real arm sits below its command (gravity), the same in every mode |
| cmd min | lowest commanded height (≥ 0: the command never entered the wall) |
| table hits | real tip more than `table_gap_mm` below the wall |
| speed | real tip's fastest downward speed in the last 10 mm |

## Debugging

```powershell
ros2 topic hz /follower/joint_states ; ros2 topic echo /leader/engaged ; ros2 topic echo /touch/metrics
```

Windows quirks (Smart App Control, exit codes, `install\` vs `src\`) are in [PROGRESS.md](PROGRESS.md#gotchas).
