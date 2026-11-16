# Progress: SO-101 haptic teleop

## Status (2026-11-15)

| part | status |
|---|---|
| Touch → real SO-101 teleop | **Working.** Push in = forward, tilt bends the wrist, wall 1 cm above the table |
| Walls | PROXY + CBF, switchable live (`c`), CBF α live (`1`–`5`). Harder wall (400 N/m) + contact tap |
| Metrics | `wall_metrics` logs every approach; first run analysed (below) |
| Follower | All 6 servos answer, 28–30 °C at rest. Servo 1 (pan) overheated once (72 °C) in an early run |
| Hapkit, camera | Not started (teammate's Hapkit code is separate) |

**Next:** ~10 approaches per setting (PROXY, CBF α 2/5/20), resting on the wall, for the slide.

## First wall measurements (run 2026-11-15 22:35, fast profile)

| | PROXY (29) | CBF α 10 (8) |
|---|---|---|
| dynamic overshoot | 0.2 mm | 0.2 mm |
| sag (real arm below its command) | 3.3 mm | 6.7 mm |
| approach speed at the wall | **405 mm/s** | **221 mm/s** |

- This arm barely overshoots: the target stops at the wall and the servos are position-controlled.
- "Table hits" were gravity **sag** (~1 cm below the command in free space), not overshoot.
- **The CBF's measurable effect is approach (impact) speed: about half.**
- Caveats: only 8 CBF approaches; one CBF approach commanded the tip 7 mm below the wall for an instant (not yet explained).

## Decisions

| when | decision | why |
|---|---|---|
| 11-04 | ROS 2 native on Windows, not WSL | USB serial, webcam and haptic timing are bad in WSL |
| 11-05 | Touch is the leader (no SO-101 leader arm) | not enough USB ports |
| 11-06 | CHAI3D + Cartesian IK | Jatin knows CHAI3D; the Hapkit does the gripper |
| 11-06 | CHAI3D window instead of RViz | Smart App Control blocked RViz |
| 11-06 | one IK branch, elbow ≥ 20° from straight, joint speed limit | the first test was erratic |
| 11-15 | hand drives the wrist point; dot drawn at the tip | tip control made tilt swing the whole arm (jittery) |
| 11-15 | paper: Bloom et al., *Haptic virtual fixtures using CBFs* (ToH 2026) | covers the walls and the planned camera barriers |
| 11-15 | measure approach speed + sag, not just "below the wall" | sag dominated the raw numbers |

## Gotchas
- **Every terminal:** `. .\ros2_env.ps1` (loads MSVC + ROS). Without it: `ModuleNotFoundError: so101_teleop`.
- **No symlink-install:** Python runs the copy in `install\`, so rebuild after edits.
  A running launch locks `teleop.exe`: stop it before building `so101_teleop`.
- **C++:** always build with `-DCMAKE_BUILD_TYPE=Release` (it must match the ROS binaries).
- **Smart App Control** blocked unsigned files, more over time: RViz, geometry_msgs, then cmake, yaml, rclpy and our
  own exes (`WinError 4551`, exit code 3236495362). It is now **off** on the tower.
  `ros2_env.ps1` still points colcon at Visual Studio's CMake.
- **Exit code 3221225786 on Ctrl+C** is normal (`STATUS_CONTROL_C_EXIT`).
- **One owner per device:** close Touch Smart Setup / CHAI3D demos; stop the launch before scripts use COM3.
- **No servo replies on an open port** = 12 V off, bus cable loose, or a servo in thermal cutoff.
- **Servo gains live in RAM:** they persist after Ctrl+C until the next launch or power-off.
- **CHAI3D:** `getPosition()` returns false on success. Rotation column 0 is the stylus axis (tip → back).
- **`.solution/` is gitignored.** Never push it.

## Open items
- [ ] Full metrics run (PROXY vs CBF at several α), then the slide.
- [ ] Explain the one CBF sample commanded below the wall.
- [ ] Reduce sag (torque limit / gains) or raise the wall so table hits mean overshoot.
- [ ] Hapkit gripper (`/hapkit/gripper_cmd`), then the camera barriers (CBF around detected objects).
