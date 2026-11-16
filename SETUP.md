# SETUP: a new Windows PC for the haptic teleop demo

Everything needed to run this repo on a fresh Windows 10/11 x64 PC, in order.
**After each step, run `.\check_setup.ps1`.** It prints `[ OK ]` or `[FAIL] -> which step to redo`.

> **For a Claude agent setting this up:**
> - Do the steps in order and verify each one with `check_setup.ps1` before moving on.
> - Steps 1 and 5 need the owner: a security setting and vendor downloads that may need a login.
> - **Never** run anything with `enable_force:=true` or `move:=true`. That moves the Touch or the arm, and only the owner runs it.
> - Never commit `.solution/`.

## What gets installed

| # | what | version (known good) | where |
|---|---|---|---|
| 1 | Smart App Control **off** | | Windows Security |
| 2 | Visual Studio 2022, C++ workload (Build Tools is enough) | MSVC 14.4x | default |
| 3 | ROS 2 Jazzy, official Windows binaries, via pixi | ros2cli 0.32.8, Python 3.12, pixi 0.81 | `C:\pixi_ws` (fixed path) |
| 4 | pip packages into the ROS Python | `requirements.txt` | pixi env |
| 5 | Geomagic Touch Device Drivers (+ Touch Smart Setup), OpenHaptics | OpenHaptics 3.4 | default |
| 6 | CHAI3D 3.3.0, built Release x64 | | **next to the repo folder** |
| 7 | this repo, built | | anywhere, e.g. `C:\haptics\ros2_ws` |

Folder layout (the C++ build finds CHAI3D as the repo's sibling, or via `$env:CHAI3D_ROOT`):
```
C:\haptics\
├── ros2_ws\          <- this repo
└── chai3d-3.3.0\     <- CHAI3D (step 6)
```

## 1. Smart App Control: off (owner decides)

It blocks unsigned DLLs and EXEs: ROS (`rclpy`), CMake, and the programs we build (`WinError 4551`).
Settings → Privacy & security → Windows Security → App & browser control → Smart App Control → **Off**.
Windows can't turn it back on without a reset. Defender antivirus stays on.

## 2. Visual Studio 2022 (C++)

```powershell
winget install Microsoft.VisualStudio.2022.BuildTools --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
```
Any edition with "Desktop development with C++" works. `ros2_env.ps1` finds it on its own.

## 3. ROS 2 Jazzy in `C:\pixi_ws`

Follow the official guide: https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html. In short:
```powershell
# a. pixi
powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"
# b. the build/runtime environment (new terminal after a.)
mkdir C:\pixi_ws ; cd C:\pixi_ws
irm https://raw.githubusercontent.com/ros2/ros2/refs/heads/jazzy/pixi.toml -OutFile pixi.toml
pixi install
# c. ROS 2 itself: download the latest "ros2-jazzy-*-windows-release-amd64.zip" from
#    https://github.com/ros2/ros2/releases and extract it so this file exists:
#    C:\pixi_ws\ros2-windows\local_setup.ps1
```

## 4. pip packages (into the ROS Python)

```powershell
cd C:\haptics\ros2_ws        # after step 7's clone
. .\ros2_env.ps1
python -m pip install -r requirements.txt
```
numpy and OpenCV already come from pixi: don't pip-install them.

## 5. Geomagic Touch drivers (owner: vendor download)

From the 3D Systems support site, install:
- **Touch Device Drivers** (includes **Touch Smart Setup**);
- **OpenHaptics for Windows** (developer edition).

Check: `C:\Windows\System32\hd.dll` exists.
Plug in the Touch, open Touch Smart Setup, check it's detected, calibrate (stylus in the inkwell), then **close Touch Smart Setup**: only one program can own the Touch.

## 6. CHAI3D 3.3.0 (Release x64)

**Fastest and known to work:** copy the built `chai3d-3.3.0` folder from the original PC
(`D:\work\02-courses\2026-fall\me384r-haptics\chai3d-3.3.0`) next to the repo, by USB.

Or build from source (https://github.com/chai3d/chai3d, version 3.3.0):
```powershell
cd C:\haptics\chai3d-3.3.0
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```
Needed afterwards:
- `build\chai3dConfig.cmake`
- `build\Release\chai3d.lib`
- `build\extras\GLFW\Release\glfw.lib`
- `bin\win-x64\hdPhantom64.dll` (ships with CHAI3D; it's the Touch plugin)

Elsewhere? `setx CHAI3D_ROOT C:\path\to\chai3d-3.3.0`, then open a new terminal.

## 7. This repo

```powershell
mkdir C:\haptics ; cd C:\haptics
git clone https://github.com/jatinsikka/so101-haptic-teleop ros2_ws
cd ros2_ws
. .\ros2_env.ps1
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
install\lib\touch_driver\test_haptic_math.exe ; install\lib\touch_driver\test_kinematics.exe   # both: "passed"
.\check_setup.ps1                                                                              # all [ OK ]
```
Ignore the colcon `output_style` traceback and the "Cannot create a symlink" warning: both harmless.

## 8. Devices and first run (owner)

1. **SO-101 USB adapter:** Windows usually installs the CH343 driver by itself. If no `USB-Enhanced-SERIAL CH343 (COMx)`
   shows up in `check_setup.ps1`, install WCH's CH343SER driver.
2. **Find the arm's port:** `ros2 run so101_teleop find_ports`. The follower is the ~12 V port. Pass it as `port:=COMx`.
3. **Read-only first:** `ros2 launch so101_teleop touch_teleop.launch.py port:=COMx`. The window opens and the orange arm matches the real one.
4. **Then (owner only):** `... move:=true enable_force:=true` (see [README](README.md#run) and the [demo](README.md#demo)).

## Troubleshooting

| symptom | cause |
|---|---|
| `WinError 4551`, or `An Application Control policy has blocked this file` | Smart App Control still on (step 1) |
| `ModuleNotFoundError: so101_teleop` | forgot `. .\ros2_env.ps1` in this terminal |
| `CHAI3D file missing: ...` during the build | step 6 (wrong folder or not built in Release) |
| `no haptic device found` | Touch unplugged, drivers missing, or Touch Smart Setup still open |
| `cannot open COMx` | wrong port, or another program (a previous launch) holds it |
| all servos `NO REPLY` | arm's 12 V supply off, or bus cable loose |
| `Permission denied: ...teleop.exe` while building | a launch is still running: stop it, then build |
