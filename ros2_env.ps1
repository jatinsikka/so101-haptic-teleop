# Activate ROS 2 Jazzy + this workspace in the current PowerShell. Dot-source it:
#
#     . D:\work\02-courses\2026-fall\me384r-haptics\ros2_ws\ros2_env.ps1
#
# Same as the docs' `pixi shell` + `call local_setup.bat`, but in one step and in PowerShell.
# Needed once per terminal; every terminal running ros2 commands must have it.

Push-Location C:\pixi_ws   # the pixi env's activation uses the current dir
(& "$env:LOCALAPPDATA\pixi\bin\pixi.exe" shell-hook -s powershell) -join "`n" | Invoke-Expression
Pop-Location

. C:\pixi_ws\ros2-windows\local_setup.ps1 2>$null   # silences the harmless RTI Connext warning
if (Test-Path "$PSScriptRoot\install\local_setup.ps1") {
    . "$PSScriptRoot\install\local_setup.ps1"
}
Write-Host "ROS 2 $env:ROS_DISTRO ready (workspace: $PSScriptRoot)"
