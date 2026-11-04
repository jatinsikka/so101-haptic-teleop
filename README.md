# ros2_ws: SO-101 teleop in ROS 2 (Windows)

Move the leader arm and the follower copies it. When the follower hits something, you feel it.
Later steps add the wrist camera, the Hapkit as the gripper control, and the Geomagic Touch as
the leader.

The package is a **skeleton** (interfaces + TODOs). The complete reference version is in
`.solution/`; open it only when you're stuck.

## How ROS communication works

- A **node** is one running program with one job. Ours: one driver per arm, `teleop`, `camera`, `hapkit`.
- A **topic** is a named channel, like `/follower/joint_states`. Nodes **publish** messages to it and
  **subscribe** to it. They never call each other directly. A publisher doesn't know who is
  listening, so you can swap one node for another (e.g. the Touch for the leader arm) without
  touching the rest.
- A **message** is a typed packet. We use `sensor_msgs/JointState` (6 joint positions, velocities,
  efforts) and `std_msgs/Float64` (one number).
- A **namespace** prefixes a node's topics. The same driver code runs twice: launched in `leader`
  it publishes `/leader/joint_states`, launched in `follower` it publishes `/follower/joint_states`.
- Inside a node, **subscriber callbacks** only store the latest message, and a **timer callback**
  does the work at a fixed rate (50 Hz).

## Architecture

```
  LEADER ARM                      TELEOP                       FOLLOWER ARM
  driver (ns: leader)             teleop                       driver (ns: follower)
      │                             │                             │
      │──── /leader/joint_states ──►│                             │   where the leader is
      │                             │── /follower/joint_commands ►│   where the follower should go
      │                             │◄── /follower/joint_states ──│   where it is + load
      │◄── /leader/joint_commands ──│                             │   force feedback (NaN = limp)

  HAPKIT  hapkit ── /hapkit/gripper_cmd ──► teleop ── /hapkit/force_cmd ──► hapkit
  CAMERA  camera ── /camera/image_raw ──► rviz2 / ros2 bag
```

| topic | type | from → to | meaning |
|---|---|---|---|
| `/leader/joint_states` | JointState | leader driver → teleop | leader pose (rad) |
| `/follower/joint_commands` | JointState | teleop → follower driver | goal pose, rate limited |
| `/follower/joint_states` | JointState | follower driver → teleop | actual pose; `effort` = servo load |
| `/leader/joint_commands` | JointState | teleop → leader driver | pull toward follower on contact joints; NaN = torque off |
| `/hapkit/gripper_cmd` | Float64 | hapkit → teleop | gripper goal (rad), replaces the leader's gripper |
| `/hapkit/force_cmd` | Float64 | teleop → hapkit | squeeze force (N) from gripper load |
| `/camera/image_raw` | Image | camera → viewers | wrist camera, 640x480 |

The driver is the only node that touches its serial port, so bus reads and writes never collide.

## Files

```
ros2_ws/
├── ros2_env.ps1                activate ROS + workspace (run in every terminal)
└── src/so101_teleop/
    ├── so101_teleop/
    │   ├── arm.py              servo bus (your sim2real/arm.py); TODO: sync read/write
    │   ├── so101_driver.py     TODO
    │   ├── teleop.py           TODO
    │   ├── camera.py           TODO
    │   ├── hapkit.py           TODO
    │   └── tools.py            find_ports, find_cameras, release (done)
    ├── launch/teleop.launch.py TODO (follower given as example)
    └── arduino/                TODO Hapkit serial sketch
```

## Daily use

```powershell
. D:\work\02-courses\2026-fall\me384r-haptics\ros2_ws\ros2_env.ps1   # every new terminal
colcon build --merge-install                                          # from ros2_ws, after every edit
ros2 launch so101_teleop teleop.launch.py follower_port:=COM5 leader_port:=COM6
```

During the build, ignore the `colcon_output` traceback and the "Cannot create a symlink" warning.
When you stop with Ctrl+C, the follower keeps holding its pose. Support it, then run
`ros2 run so101_teleop release COMx` to drop it.

## Milestones

| # | Build | Done when |
|---|---|---|
| 0 | — | `ros2 run so101_teleop find_ports` shows the follower (12 V) and the leader (5 V) |
| 1 | `arm.read_state()` | a Python loop prints joint angles that change as you move the arm |
| 2 | driver publishes | `ros2 topic echo /leader/joint_states` updates, and `ros2 topic hz` shows about 50 Hz |
| 3 | `send_targets()` + commands | one small `ros2 topic pub --once /follower/joint_commands ...` moves the follower |
| 4 | `teleop` + launch | the follower copies the leader; `rqt_plot` shows both |
| 5 | force feedback | holding the follower's elbow makes the leader resist |
| 6 | camera | `rviz2` shows `/camera/image_raw`; `ros2 bag record` works |
| 7 | Hapkit | the handle opens and closes the gripper; squeezing an object pushes back |
| 8 | Geomagic Touch | a `touch` + `ik` node publish onto `/leader/joint_states` (remap); feedback goes to the Touch |

Safety: keep a hand near the follower the first time it moves, clamp every command to `LIMITS`,
and stop sending commands when the leader data goes stale.

## Debugging

```powershell
ros2 node list ; ros2 topic list ; ros2 topic info -v /follower/joint_commands   # who talks to whom
ros2 param list /follower/driver
ros2 run rqt_plot rqt_plot /follower/joint_states/position[2] /leader/joint_states/position[2]
```
