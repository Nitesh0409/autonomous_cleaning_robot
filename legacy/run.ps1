param (
    [switch]$headless
)

Write-Host "--- Initializing Robotics Simulation ---" -ForegroundColor Cyan
if ($headless) {
    Write-Host "MODE: HEADLESS (Background)" -ForegroundColor Yellow
    $launch_args = "headless:=True"
} else {
    Write-Host "MODE: HEADED (GUI Visible)" -ForegroundColor Green
    $launch_args = "headless:=False"
}

# 1. STOP ALL STALE PROCESSES (The Nuke)
Write-Host "--- Purging Ghost Processes ---" -ForegroundColor Cyan
wsl -d Ubuntu-24.04 bash -c "pkill -9 -f 'gz|ros|planner|manager' || true"
wsl -d Ubuntu-24.04 bash -c "rm -rf ~/.gz/sim/*" # Clear Gazebo Cache

# 2. Build and Launch within WSL
wsl -d Ubuntu-24.04 bash -c "export DISPLAY=:0 && export QT_X11_NO_MITSHM=1 && export PYTHONUNBUFFERED=1 && export ROS_DOMAIN_ID=42 && source /opt/ros/jazzy/setup.bash && cd '/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot' && colcon build --symlink-install --packages-select robot && source './install/setup.bash' && ros2 launch robot sim_gazebo.launch.py $launch_args"

Write-Host "--- Simulation Finished ---" -ForegroundColor Green
