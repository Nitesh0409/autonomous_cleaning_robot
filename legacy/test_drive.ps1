# [Manual Physics Test - ULTIMATE ROBUST VERSION]
# This version uses a temporary script to bypass all PowerShell quoting errors.

$ws_path = "/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot"

# 1. Create a temporary Bash script for the teleop (Bypass PS parser)
$sh_content = @"
source /opt/ros/jazzy/setup.bash
cd '$ws_path'
source ./install/setup.bash
python3 robot/arrow_teleop.py
"@
$sh_content | Out-File -FilePath "c:\SOLIDWORKS Data\browser\Projects\Robotics Assignment\Robot\manual_teleop.sh" -Encoding ascii

# 2. Close existing sessions
Write-Host "🧹 Cleaning existing sessions..." -ForegroundColor Cyan
wsl -d Ubuntu-24.04 bash -c "pkill -9 -f 'gz|ros|planner|manager|teleop|arrow_teleop' || true"

# 3. Launch Simulation (Gazebo/RViz)
Write-Host "🚀 Launching Robot Environment..." -ForegroundColor Cyan
wsl -d Ubuntu-24.04 bash -c "export DISPLAY=:0 && source /opt/ros/jazzy/setup.bash && cd '$ws_path' && source ./install/setup.bash && ros2 launch robot sim_gazebo.launch.py manual:=True" &

# 4. Wait for Gazebo
Start-Sleep -s 12

# 5. Launch Arrow-Key Teleop in a NEW INTERACTIVE WINDOW
Write-Host "⌨️ POPPING UP MANUAL CONTROL WINDOW..." -ForegroundColor Yellow
# Using cmd.exe to launch WSL directly to avoid PowerShell parsing '&&'
Start-Process cmd.exe -ArgumentList "/c", "wsl -d Ubuntu-24.04 bash '/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot/manual_teleop.sh'" -WindowStyle Normal

Write-Host "----------------------------------" -ForegroundColor Green
Write-Host "1. Click on the NEW window that just appeared."
Write-Host "2. Use your ARROW KEYS to move."
Write-Host "----------------------------------"
