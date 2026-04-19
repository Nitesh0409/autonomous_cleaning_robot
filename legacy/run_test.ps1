Write-Host "🧪 --- Launching Pick-and-Place Validation Suite ---" -ForegroundColor Cyan
Write-Host "This test runs HEADLESS. Telemetry will be tracked in evaluation_report.csv" -ForegroundColor Gray

# 1. Kill any existing simulations
wsl -d Ubuntu-24.04 bash -c "pkill -9 -f 'ruby|gz|ros|rviz|planner|manager|nav_monitor' || true"
wsl -d Ubuntu-24.04 bash -c "pkill -9 -f 'python3' || true"

# 2. Build and Launch inside WSL
wsl -d Ubuntu-24.04 bash -c "export PYTHONUNBUFFERED=1 && source /opt/ros/jazzy/setup.bash && cd '/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot' && colcon build --symlink-install --packages-select robot && source './install/setup.bash' && ros2 launch robot pick_and_place_test.launch.py"

# 3. Analyze Results
python scripts/analyze_test.py
