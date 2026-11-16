# Checks that this PC has everything SETUP.md installs. Read-only: changes nothing.
#     powershell -ExecutionPolicy Bypass -File .\check_setup.ps1
# Every line is [ OK ] or [FAIL] + what to do. Run it again after each fix.

$fails = 0
function Check($ok, $what, $fix) {
    if ($ok) { Write-Host "[ OK ] $what" -ForegroundColor Green }
    else { Write-Host "[FAIL] $what  ->  $fix" -ForegroundColor Red; $script:fails++ }
}
$repo = $PSScriptRoot

# 1. Smart App Control: must be off, or it blocks ROS and our own programs (WinError 4551)
$sac = (Get-ItemProperty "HKLM:\SYSTEM\CurrentControlSet\Control\CI\Policy" -ErrorAction SilentlyContinue).VerifiedAndReputablePolicyState
Check ($sac -ne 1 -and $sac -ne 2) "Smart App Control off (state: $sac; 0 = off, 1 = on, 2 = evaluation)" "SETUP.md step 1 (owner's decision)"

# 2. Visual Studio 2022 with C++
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = $null
if (Test-Path $vswhere) { $vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath }
Check ($vs) "Visual Studio 2022 with C++ ($vs)" "SETUP.md step 2"

# 3. ROS 2 Jazzy in C:\pixi_ws
Check (Test-Path "$env:LOCALAPPDATA\pixi\bin\pixi.exe") "pixi installed" "SETUP.md step 3a"
Check (Test-Path "C:\pixi_ws\.pixi\envs\default\python.exe") "pixi environment (C:\pixi_ws\.pixi)" "SETUP.md step 3b: pixi install"
Check (Test-Path "C:\pixi_ws\ros2-windows\local_setup.ps1") "ROS 2 Jazzy binaries (C:\pixi_ws\ros2-windows)" "SETUP.md step 3c"

# 4. Touch drivers (hd.dll is what CHAI3D's hdPhantom64.dll loads)
Check (Test-Path "$env:WINDIR\System32\hd.dll") "Touch / OpenHaptics runtime (System32\hd.dll)" "SETUP.md step 5"
Check (Test-Path "${env:ProgramFiles}\3D Systems\Touch Device Drivers") "Touch Device Drivers (incl. Touch Smart Setup)" "SETUP.md step 5"

# 5. CHAI3D, built Release x64: next to the repo folder, or $env:CHAI3D_ROOT
$chai = $env:CHAI3D_ROOT
if (-not $chai) { $chai = Join-Path (Split-Path $repo -Parent) "chai3d-3.3.0" }
Check (Test-Path "$chai\build\chai3dConfig.cmake") "CHAI3D configured ($chai\build)" "SETUP.md step 6"
Check (Test-Path "$chai\build\Release\chai3d.lib") "CHAI3D built Release (chai3d.lib)" "SETUP.md step 6: cmake --build build --config Release"
Check (Test-Path "$chai\build\extras\GLFW\Release\glfw.lib") "CHAI3D GLFW built (glfw.lib)" "SETUP.md step 6"
Check (Test-Path "$chai\bin\win-x64\hdPhantom64.dll") "CHAI3D Touch plugin (bin\win-x64\hdPhantom64.dll)" "use the full CHAI3D 3.3.0 download"

# 6. Python packages, in the ROS environment
if (Test-Path "C:\pixi_ws\ros2-windows\local_setup.ps1") {
    . "$repo\ros2_env.ps1" *> $null
    foreach ($m in "scservo_sdk", "serial", "numpy", "cv2", "rclpy") {
        python -c "import $m" 2>$null
        Check ($LASTEXITCODE -eq 0) "python: import $m" "SETUP.md step 4: python -m pip install -r requirements.txt"
    }
}

# 7. This workspace built
Check (Test-Path "$repo\install\lib\touch_driver\touch_node.exe") "workspace built (touch_node.exe)" "SETUP.md step 7: colcon build"

# 8. Devices (information: plug them in before the demo)
Write-Host "`nSerial ports (SO-101 = USB-Enhanced-SERIAL CH343; the Touch also shows up here):"
Get-CimInstance Win32_SerialPort -ErrorAction SilentlyContinue | ForEach-Object { Write-Host "   $($_.DeviceID)  $($_.Description)" }
Get-PnpDevice -Class Ports -PresentOnly -ErrorAction SilentlyContinue | ForEach-Object { Write-Host "   $($_.FriendlyName)" }

Write-Host ""
if ($fails -eq 0) { Write-Host "All checks passed." -ForegroundColor Green }
else { Write-Host "$fails check(s) failed." -ForegroundColor Red }
