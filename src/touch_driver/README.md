# touch_driver: Geomagic Touch as the SO-101 leader (CHAI3D)

The stylus moves the SO-101's gripper; tilt bends the wrist, twist rolls it. A virtual fixture (a plane above the
table) is implemented after **Bloom et al., "Haptic virtual fixtures for telemanipulation using control barrier
functions", IEEE ToH 2026**, with the paper's three conditions: **OFF**, **IMPEDANCE** and **CBF**.

## Run

```powershell
colcon build --merge-install --packages-select touch_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch touch_driver touch.launch.py                                      # read-only: zero force
ros2 launch touch_driver touch.launch.py enable_force:=true                   # forces on
ros2 launch touch_driver touch.launch.py enable_force:=true wall_mode:=cbf    # start in CBF (off / impedance / cbf)
install\lib\touch_driver\test_haptic_math.exe ; install\lib\touch_driver\test_kinematics.exe
```

Close Touch Smart Setup and the CHAI3D demos first: only one program can own the Touch.
Click the window once so it gets the keys. Every change pops up on screen and prints in the terminal.

| key | does | when |
|---|---|---|
| button 1 | hold = engage (clutch / deadman) | |
| `c` | mode: **OFF → IMPEDANCE → CBF** (wall turns grey / red / blue) | button released |
| `f` | CBF filter on the arm in **every** mode (the paper's user-study setup) | button released |
| `h` | rubber band (tracking force) on/off | button released |
| `↑` / `↓` | K × / ÷ 1.25 (how fast the arm chases your hand), 1–30 /s | anytime |
| `→` / `←` | α × / ÷ 1.25 (how fast it may approach the wall), 0.5–100 /s | anytime |
| `1`–`5` | K/α = 0.5 / 1 / 2 / 3 / 5 (sets α = K / ratio) | anytime |
| `q` | quit | |

## How it works

**1. Map (clutch).** Pressing button 1 stores the stylus pose and the real arm pose, so engaging never jumps.
Then `x_hand = anchor + 2 · signs · (stylus − stylus anchor)`. Push in → forward, right → right, up → up.
The hand drives the wrist point; tilt bends the wrist. The yellow dot is drawn at the gripper tip it commands.

**2. The arm follows your hand** (paper Fig. 4, eq. 7), every 1 ms in `haptics.cpp` `update_target`:

```
u = K·(x_hand − x_cmd)      the commanded point chases the hand (K = 10 /s: 0.1 s lag)
[CBF filter on u]           CBF mode, or every mode with [f]
x_cmd += u·dt               then: floor backstop -> analytic IK -> joint speed limit -> arm
```

**3. The three modes** (all act on the gripper tip)

| | OFF | IMPEDANCE (eq. 9–10) | CBF (eq. 14–35) |
|---|---|---|---|
| arm | goes where you go: **hits the table** | **not stopped**: overshoots like OFF | filter slows it **before** the wall, never crosses |
| you feel | nothing | spring `k·d − b·v` once **your hand** is past the wall | **damping** ∝ how hard the filter brakes, then the wall |
| code | | `wall_force_mag` on `hand_depth` | `cbf_filter`, `cbf_removed` |

**CBF filter, step by step** (`haptic_math.hpp` `cbf_filter`):

| step | equation | meaning |
|---|---|---|
| barrier (eq. 16) | `h = n·(x_tip − wall)` | tip height above the wall; safe while `h ≥ 0` |
| CBF rule (eq. 14) | `n·u* ≥ −α·h` | approach at most α × height |
| filter (eq. 26, 29) | `Ψ = n·u + α·h`; if `Ψ < 0`: `u* = u − Ψ·n` | the closest safe command: only the part of `u` into the wall is cut |
| force (eq. 23, 32) | `F = D·(u* − u) = −D·Ψ·n` | the harder the filter brakes, the harder you're pushed back (D = 40 N·s/m) |

**When the CBF force starts** depends on **K/α** (eq. 35). Our test (`test_cbf`) reproduces it:

| K/α | force starts | test (hand at 5 cm/s) |
|---|---|---|
| 3 (key `4`, default) | **before** your hand reaches the wall, if you come in too fast | 9.9 mm before |
| 1 (key `2`) | **at** the wall, like the impedance wall | at the wall |
| 0.5 (key `1`) | only once your hand is **past** the wall (the paper: avoid) | 2.5 mm past |

**4. Forces** (felt at the stylus)
- **Impedance wall** (IMPEDANCE): 400 N/m spring + damping on how far your hand is past the wall.
- **CBF force** (CBF): `D × speed removed`, + the same damping. Past the wall it feels like a D × K spring (≤ 400 N/m).
- **Rubber band** (eq. 8, every mode, `h`): `Kp·(x_robot − x_hand)`, Kp = 15 N/m. Robot = the real arm if heard,
  else the commanded arm. You feel where the arm is and how far it lags.
- **Reach:** a spring toward what the arm can actually reach. **Damping:** light, on a 30 Hz low-passed velocity.

**5. Floor backstop** (not in the paper): in every mode the commanded tip never goes below the table
(`wall.floor_below` = 1.5 cm under the wall). OFF and IMPEDANCE still reach the table; the arm is just not driven into it.

**Differences from the paper (say this in the talk):**
- One plane only (no spheres, no several constraints).
- The CBF uses the closed-form solution of a one-constraint QP, in task space. The paper solves a QP over joint
  velocities with actuator limits (eq. 22); our joint limits come after it (IK clamps + speed limit). *Phase 2.*
- The barrier is on the commanded arm (kinematic model, as the paper assumes); the real arm sags ~0.5–1 cm.
- K = 10 /s instead of 1.25 /s (a fixed SO-101, not a slow mobile base). Paper's Kd ≈ 0.01 is left out of the rubber band.

**6. Threads.** Haptics at 1 kHz (never waits: `try_lock` only), the window on the main thread, ROS on a spin thread.

## Files

| file | what |
|---|---|
| `include/touch_driver/haptic_math.hpp` | map, wall, **CBF filter**, safety filter, hard caps |
| `include/touch_driver/kinematics.hpp` + `src/kinematics.cpp` | analytic FK/IK; geometry and meshes from the URDF |
| `include/touch_driver/shared.hpp` | settings, modes, data shared between threads |
| `src/haptics.cpp` | CHAI3D device, 1 kHz loop (modes, forces), window, keys |
| `src/touch_node.cpp` | ROS params, topics, metrics |
| `config/touch.yaml` | every setting |
| `test/` | math + CBF tests (incl. eq. 35); FK/meshes vs KDL; IK round trip and sweep |

## Safety (the Touch is expensive)

| layer | against |
|---|---|
| `enable_force:=false` by default | forces while testing |
| deadman: button 1 held, fresh press only | force or motion with no hand on the stylus |
| 0.5 s engage ramp | a kick on engage |
| `safe_force`: NaN → fault, \|F\| clamp, slew limit | bugs, steps, buzz (also smooths live K/α changes) |
| hard caps compiled in: 2.5 N, 400 N/m, 4 N·s/m, 200 N/s; CBF D × K ≤ 400 N/m | a bad YAML value or key press |
| floor backstop | driving the gripper into the table |
| follower watchdog 0.1 s; fault latch until `/touch/clear_fault` | stale data, bad readings |
| shutdown: 10 ticks of zero force, then close | Ctrl+C mid-contact |

2.5 N is about 3× the Touch's continuous rating (0.88 N): taps and short presses are fine, don't lean on the wall for long.
The Touch can't render stiffer than 400 N/m, so "harder" comes from the 2.5 N cap (the spring saturates at 6 mm, not
4 mm), the 200 N/s slew (full force in ~12 ms) and the wall damping 2.5 (no bounce).

## Topics

| name | type | meaning |
|---|---|---|
| `/leader/joint_states` | JointState | 5-joint arm target, 100 Hz |
| `/leader/engaged` | Bool | deadman: teleop moves the follower only while true |
| `/follower/joint_states` | JointState | input: real arm |
| `/touch/force` | Float64MultiArray | [fx, fy, fz] N sent to the Touch |
| `/touch/metrics` | Float64MultiArray | [t, mode, engaged, cmd tip h, real tip h, \|F\|, K, α, filter on] per follower state; h = height above the wall (m) |
| `/touch/clear_fault` | std_srvs/Trigger | clear a latched fault (button released) |

No geometry_msgs: Smart App Control blocked that DLL.

## Tuning (`config/touch.yaml`, rebuild after editing)

- Wrong direction → `map.axis_signs`. Tilt inverted → `map.pitch_gain`. Twist inverted → `map.roll_gain`.
- Impedance wall bounces → raise `wall.damping` (≤ 4). Buzzes → lower `wall.damping`, then `velocity_lpf_hz`.
- CBF start values: `wall.cbf_gain` (K), `wall.cbf_alpha` (α), `wall.cbf_damping` (D). Keys change K and α live.
- Rubber band too strong/weak → `tracking.stiffness`.
