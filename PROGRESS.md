# Progress: SO-101 haptic teleop

## Status (2026-11-06)

| part | status |
|---|---|
| Touch node (`touch_driver`) | Runs at 1 kHz on the device. Read-only test OK. First force test: wall worked but felt gritty → fixed (30 Hz velocity LPF, reach on the IK point, `pitch_gain −1`). **Fix not yet re-felt.** |
| Follower driver + teleop | Written: `send_targets`, driver, teleop, `touch_teleop.launch.py`. **Never moved the real arm.** The driver started once (read 12.x V). |
| Follower bus (COM3) | **Last check: port opens, but none of the 6 servos answer.** Likely 12 V supply or bus cable. |
| Hapkit, camera | Not started (`hapkit.py`, `camera.py` are TODO) |

**Next**
1. Fix the servo power or cable. Check with `python state.py` (read-only).
2. `touch_teleop.launch.py`, read-only: does the orange arm in the window match the real arm, joint by joint?
3. `move:=true`, then `enable_force:=true`.

## Decisions

| when | decision | why |
|---|---|---|
| 11-04 | ROS 2 native on Windows, not WSL | USB serial, webcam and haptic timing are bad in WSL |
| 11-05 | Touch is the leader (no SO-101 leader arm) | not enough USB ports |
| 11-06 | CHAI3D + Cartesian IK, replacing raw OpenHaptics joint-space | Jatin knows CHAI3D; the Hapkit does the gripper, so the Touch only positions the tool and renders contact |
| 11-06 | CHAI3D window instead of RViz (RViz files deleted) | Smart App Control blocks RViz DLLs |
| 11-06 | Hand = wrist point, one IK branch, elbow ≥ 20° from straight, 2 rad/s limit | the first test was erratic (tilt pivoted about the tip, branches flipped) |
| 11-06 | Map signs `[1,−1,1]`, stylus yaw ignored | pull toward you = reach out; the SO-101 has 5 DOF |

## Verified
- `test_kinematics`: FK within 5 µm of KDL; meshes match KDL; IK round trip on 2671 poses; 1 mm sweep max 0.022 rad/mm.
- `test_haptic_math`: map signs, twist, wall proxy, `safe_force`, caps, NaN.
- Touch: 1000 Hz, calibrated, clean release on Ctrl+C; button 1 engages; the wall pushes back.

## Gotchas
- **Every terminal:** `. .\ros2_env.ps1` (it also loads MSVC). Without it you get `ModuleNotFoundError: so101_teleop`.
- **No symlink-install:** Python runs the copy in `install\`, so rebuild after edits.
- **C++:** always build with `-DCMAKE_BUILD_TYPE=Release` (it must match the ROS binaries).
- **Smart App Control blocks unsigned ROS DLLs.**
  - You'll see RViz `LoadLibrary error: 4551`, or a node exiting with `3236495362` (0xC0E90002).
  - Check Event Viewer → CodeIntegrity/Operational, events 3077/3033.
  - Workaround: avoid geometry_msgs and RViz. Turning it off is your call.
- **Exit code 3221225786 on Ctrl+C** is normal (`STATUS_CONTROL_C_EXIT`).
- **One owner per device:** close Touch Smart Setup and the CHAI3D demos. Stop any launch before scripts use COM3.
- **USB powers only the adapter.** No servo replies on an open port = 12 V off or bus cable loose.
- **CHAI3D:** `getPosition()` returns false on success. Rotation column 0 is the stylus axis, pointing tip → back.
- **`.solution/` is gitignored.** Never push it.

## Open items
- [ ] Re-feel the wall after the grit fix; check the tilt and twist directions.
- [ ] Real-arm read-only check, then the first motion at 50% torque.
- [ ] Hapkit gripper (`/hapkit/gripper_cmd`), then the camera.
