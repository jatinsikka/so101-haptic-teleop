# touch_driver: Geomagic Touch as the SO-101 leader (CHAI3D)

The stylus position drives the SO-101 wrist point (via IK), stylus tilt drives wrist_flex and twist drives wrist_roll.
Contact comes back as a force at the stylus. The gripper comes from the Hapkit.

## Run

```powershell
colcon build --merge-install --packages-select touch_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch touch_driver touch.launch.py                     # read-only: only zero force is sent
ros2 launch touch_driver touch.launch.py enable_force:=true  # forces on (limits in config/touch.yaml)
install\lib\touch_driver\test_haptic_math.exe ; install\lib\touch_driver\test_kinematics.exe
```

- Close Touch Smart Setup and the CHAI3D demos first: only one program can own the Touch.
- Button 1 = clutch/deadman (hold to engage).
- Keys, only while button 1 is released: `w` wall, `b` follower feedback, `q` quit.

## Files

| file | what |
|---|---|
| `include/touch_driver/haptic_math.hpp` | map, wall, safety filter, hard caps |
| `include/touch_driver/kinematics.hpp` + `src/kinematics.cpp` | analytic FK/IK; geometry and meshes from the URDF |
| `src/haptics.cpp` | CHAI3D device, 1 kHz loop, window |
| `src/touch_node.cpp` | ROS params, topics, service |
| `test/` | math tests; FK and meshes vs KDL; IK round trip; 1 mm sweep with no jumps |

## How it works

**Map (clutch).** Pressing button 1 stores the stylus pose and the *real* arm pose, so engaging never jumps:
`p_wrist = arm_anchor + 2·diag(1,−1,1)·(p_touch − touch_anchor)`.
- Pull toward you → the arm reaches out; right → right; up → up.
- Stylus yaw is ignored: the SO-101 has 5 DOF.

**IK.** A planar 3-link arm on a turntable, with one fixed solution:
- pan = `atan2` toward the wrist point;
- lift and elbow by the law of cosines; the elbow stays ≥ 20° from straight (a singularity);
- wrist_flex = tilt − lift − elbow.

Then a 2 rad/s joint rate limit.

**Forces (felt at the stylus).**
- **Wall:** a proxy on the tool tip. The tool is lifted to the surface, so the arm never enters the wall. You feel `k·depth − b·v`.
- **Reach:** a spring toward what the IK can actually reach.
- **Feedback:** a spring toward the real arm, beyond a 1 cm deadband.
- **Damping:** light, on a 30 Hz low-passed velocity (raw velocity felt like sandpaper).

**Threads.** The haptic thread runs at 1 kHz and only `try_lock`s. The window runs on the main thread, ROS on a spin thread.
Each tick: `read_touch → read_ros → deadman → update_target → compute_force → output → publish_state`.

## Safety (the Touch is expensive)

| layer | against |
|---|---|
| `enable_force:=false` by default | forces while testing |
| deadman: button 1 held, fresh press only | force or motion with no hand on the stylus |
| 0.5 s engage ramp | a kick on engage |
| follower watchdog 0.1 s | stale feedback stuck on |
| `safe_force`: NaN → fault, \|F\| clamp, slew limit | bugs, steps, buzz |
| hard caps compiled in: 1.5 N, 250 N/m, 2 N·s/m, 50 N/s | a bad YAML value |
| fault latch until `/touch/clear_fault` | bad readings |
| shutdown: 10 ticks of zero force, then close | Ctrl+C mid-contact |

CHAI3D's Phantom driver doesn't clamp forces or check errors, so all of this lives in our code.

## Bring-up checklist

1. **Read-only.**
   - Wrong axis → flip it in `map.axis_signs`.
   - Tilt inverted → `map.pitch_gain`.
   - Twist inverted → `map.roll_gain`.
2. **`enable_force:=true`, damping only:** should feel slightly thick.
3. **Reach, then the wall at z = 5 cm:** firm, no buzz. If it buzzes, raise `wall.damping`, not the stiffness.
4. **Feedback:** `b`, with teleop and the follower running.

## Topics

| name | type | meaning |
|---|---|---|
| `/leader/joint_states` | JointState | 5-joint arm target, 100 Hz |
| `/leader/engaged` | Bool | deadman: teleop moves the follower only while true |
| `/follower/joint_states` | JointState | input: real arm (anchor + feedback) |
| `/touch/force` | Float64MultiArray | [fx, fy, fz] N sent to the Touch (touch frame) |
| `/touch/clear_fault` | std_srvs/Trigger | clear a latched fault (button released) |

`/touch/force` isn't geometry_msgs because Smart App Control blocks that DLL.
