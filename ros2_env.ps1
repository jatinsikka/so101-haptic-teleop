# Activate ROS 2 Jazzy + this workspace in the current PowerShell. Dot-source it from the repo folder:
#
#     . .\ros2_env.ps1
#
# Same as the docs' `pixi shell` + `call local_setup.bat`, but in one step and in PowerShell.
# Needed once per terminal; every terminal running ros2 commands must have it.
# Expects: ROS 2 Jazzy in C:\pixi_ws (see SETUP.md), Visual Studio 2022 with C++ (any edition).

# Visual Studio 2022, any edition (Build Tools / Community / ...), found with vswhere
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = if (Test-Path $vswhere) {
    & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
}
if (-not $vs) { Write-Warning "Visual Studio 2022 with C++ not found: C++ packages won't build (see SETUP.md)" }

# MSVC compiler (cl.exe, x64) for touch_driver. Loaded FIRST so that the pixi Python and the ROS
# paths added below take priority over anything Visual Studio puts on PATH.
if ($vs -and -not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    $env:PATH += ";${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer"   # vswhere, used by the next script
    & "$vs\Common7\Tools\Launch-VsDevShell.ps1" -Arch amd64 -HostArch amd64 -SkipAutomaticLocation | Out-Null
}

Push-Location C:\pixi_ws   # the pixi env's activation uses the current dir
(& "$env:LOCALAPPDATA\pixi\bin\pixi.exe" shell-hook -s powershell) -join "`n" | Invoke-Expression
Pop-Location

# Smart App Control once blocked pixi's cmake.exe (WinError 4551): colcon uses Visual Studio's if present
$vscmake = "$vs\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if ($vs -and (Test-Path $vscmake)) { $env:CMAKE_COMMAND = $vscmake }

. C:\pixi_ws\ros2-windows\local_setup.ps1 2>$null   # silences the harmless RTI Connext warning
if (Test-Path "$PSScriptRoot\install\local_setup.ps1") {
    . "$PSScriptRoot\install\local_setup.ps1"
}
Write-Host "ROS 2 $env:ROS_DISTRO ready (workspace: $PSScriptRoot)"
