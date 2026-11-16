# touch_driver: Geomagic Touch as the SO-101 leader (CHAI3D)

The stylus moves the SO-101's gripper. Tilt bends the wrist, twist rolls it. A virtual wall comes back as
force at the stylus, in two modes: **PROXY** (stops you at contact) or **CBF** (slows you down before it).

## Run

```powershell
colcon build --merge-install --packages-select touch_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch touch_driver touch.launch.py                                      # read-only: zero force
ros2 launch touch_driver touch.launch.py enable_force:=true                   # forces on
ros2 launch touch_driver touch.launch.py enable_force:=true wall_mode:=cbf    # start in CBF mode
install\lib\touch_driver\test_haptic_math.exe ; install\lib\touch_driver\test_kinematics.exe
```

Close Touch Smart Setup and the CHAI3D demos first: only one program can own the Touch.
Click the window once so it gets the keys.

| key | does | when |
|---|---|---|
| button 1 | hold = engage (clutch / deadman) | |
| `c` | wall mode: PROXY ↔ CBF (pop-up + banner say which) | button released |
| `1`–`5` | CBF alpha = 2 / 5 / 10 / 20 / 50 per second | anytime |
| `w` / `t` / `b` | wall / contact tap / follower feedback on-off | button released |
| `q` | quit | |

## Files

| file | what |
|---|---|
| `include/touch_driver/haptic_math.hpp` | map, wall, **CBF**, contact tap, safety filter, hard caps |
| `include/touch_driver/kinematics.hpp` + `src/kinematics.cpp` | analytic FK/IK; geometry and meshes from the URDF |
| `src/haptics.cpp` | CHAI3D device, 1 kHz loop, window |
| `src/touch_node.cpp` | ROS params, topics, metrics |
| `config/touch.yaml` | every setting |
| `test/` | math + CBF tests; FK/meshes vs KDL; IK round trips and sweeps |

## How it works

**1. Map (clutch).** Pressing button 1 stores the stylus pose and the real arm pose, so engaging never jumps.
Then `arm point = anchor + 2 · signs · (stylus − stylus anchor)`. Push in → forward, right → right, up → up.
The hand drives the wrist point (`map.control_point: wrist`), so tilt bends the wrist like your own hand.
The yellow dot is drawn at the gripper tip it commands.

**2. IK.** A planar 3-link arm on a turntable, one fixed solution:
pan = `atan2` toward the target; shoulder and elbow by the law of cosines (elbow ≥ 20° from straight);
wrist_flex = tilt − shoulder − elbow. Then a joint speed limit (2 rad/s).

**3. The wall: two modes** (both act on the gripper tip, so the arm never enters the wall)

| | PROXY | CBF |
|---|---|---|
| idea | move freely; at the wall, put the target back on the surface | approach the wall at most **α × distance** |
| arm | full speed until contact, then stops | slows down as it gets close |
| you feel | the wall only after contact (+ the tap) | resistance **before** contact if you come in too fast |
| code | `wall_proxy` | `cbf_step` |

CBF in one line: with `h` = distance above the wall, keep `dh/dt ≥ −α·h`. Each tick, if the hand's step toward
the wall is bigger than `α·h·dt`, only that part is cut (sideways stays free). It can never cross the wall.

| α (keys 1–5) | 2 | 5 | 10 | 20 | 50 |
|---|---|---|---|---|---|
| max speed 1 cm from the wall | 2 cm/s | 5 cm/s | 10 cm/s | 20 cm/s | 50 cm/s |
| feel | floats down | clear braking | moderate | late | almost proxy |

In CBF mode the proxy stays underneath as a backstop.

**4. Forces** (as felt at the stylus)
- **Wall:** spring-damper `k·d − b·v` on how far your hand is past the arm target (400 N/m, 1.2 N max).
- **Contact tap** (proxy): on impact, `A·e^(−t/20 ms)·sin(2π·100 Hz·t)`, A grows with impact speed (≤ 0.4 N). Makes contact feel hard.
- **Reach:** a spring toward what the arm can actually reach.
- **Feedback** (off by default): a spring toward the real arm, beyond a 1 cm deadband.
- **Damping:** light, on a 30 Hz low-passed velocity (raw velocity feels like sandpaper).

**5. Threads.** Haptics at 1 kHz (never waits: `try_lock` only), the window on the main thread, ROS on a spin thread.

## Safety (the Touch is expensive)

| layer | against |
|---|---|
| `enable_force:=false` by default | forces while testing |
| deadman: button 1 held, fresh press only | force or motion with no hand on the stylus |
| 0.5 s engage ramp | a kick on engage |
| `safe_force`: NaN → fault, \|F\| clamp, slew limit | bugs, steps, buzz |
| hard caps compiled in: 1.5 N, 400 N/m, 2 N·s/m, 50 N/s; tap 0.5 N / 0.1 s | a bad YAML value |
| follower watchdog 0.1 s; fault latch until `/touch/clear_fault` | stale data, bad readings |
| shutdown: 10 ticks of zero force, then close | Ctrl+C mid-contact |

1.2 N is above the Touch's continuous rating (0.88 N): short presses are fine, don't lean on the wall for long.

## Topics

| name | type | meaning |
|---|---|---|
| `/leader/joint_states` | JointState | 5-joint arm target, 100 Hz |
| `/leader/engaged` | Bool | deadman: teleop moves the follower only while true |
| `/follower/joint_states` | JointState | input: real arm |
| `/touch/force` | Float64MultiArray | [fx, fy, fz] N sent to the Touch |
| `/touch/metrics` | Float64MultiArray | [t, cbf, engaged, cmd tip h, real tip h, \|F\|, α] per follower state, h = height above the wall (m) |
| `/touch/clear_fault` | std_srvs/Trigger | clear a latched fault (button released) |

No geometry_msgs: Smart App Control blocked that DLL.

## Tuning (`config/touch.yaml`, rebuild after editing)

- Wrong direction → `map.axis_signs`. Tilt inverted → `map.pitch_gain`. Twist inverted → `map.roll_gain`.
- Wall buzzes → raise `wall.damping` first, then lower `wall.stiffness`.
- Tap: `transient.gain` (strength), `freq_hz` (low = thud, high = click).
- CBF: `wall.cbf_alpha` (start value; keys 1–5 change it live).
