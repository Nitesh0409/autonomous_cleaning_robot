# --- Micro-Sentinel-Elite Interactive Mission Director ---

Clear-Host
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "   🚀 ROBOT MISSION CONTROL CENTER" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

# 1. Kill any existing simulations
Write-Host "Cleaning environment..." -ForegroundColor Gray
# Added nav_monitor to the cleanup list and used -9 for immediate termination
wsl -d Ubuntu-24.04 bash -c "pkill -9 -f 'ruby|gz|ros|rviz|planner|manager|nav_monitor' || true"
wsl -d Ubuntu-24.04 bash -c "pkill -9 -f 'python3' || true"

# 2. Select Test Mode
Write-Host "`nSelect Mission Category:" -ForegroundColor Yellow
Write-Host "  [1] Calibration & Evolutionary Suite" -ForegroundColor Magenta
Write-Host "  [2] Obstacle Worlds & Benchmarks"
Write-Host "  [3] Free Roam (Standard Map)"
$Category = Read-Host "Choice (1-3 default=1)"

$Planner = "apf"
if ($Category -ne "1") {
    # 3. Select Algorithm (Only for Navigation Modes)
    Write-Host "`nSelect Local Planner Algorithm:" -ForegroundColor Yellow
    Write-Host "  [1] APF (Artificial Potential Fields) - Fast, Reactive"
    Write-Host "  [2] DWA (Dynamic Window Approach) - Predictive, Smooth"
    $AlgoChoice = Read-Host "Choice (1/2 default=1)"
    if ($AlgoChoice -eq "2") { $Planner = "dwa" }
}

$World = "mini_proving_ground"
$RunMode = "Normal"
$EvoMode = ""

if ($Category -eq "2") {
    Write-Host "`nSelect Obstacle Scenario:" -ForegroundColor Yellow
    Write-Host "  [1] Static Wall Pass (Benchmark)"
    Write-Host "  [2] THE GAUNTLET (Hard Slalom Course)"
    Write-Host "  [3] Orchestrated Integration Suite (Headless)"
    $SubChoice = Read-Host "Choice (1-3 default=1)"
    if ($SubChoice -eq "1") { $World = "benchmark_obstacle"; $RunMode = "Benchmark" }
    elseif ($SubChoice -eq "2") { $World = "slalom_gauntlet"; $RunMode = "Benchmark" }
    else { $RunMode = "Suite" }
}
elseif ($Category -eq "1") {
    Write-Host "`nSelect Evolutionary Calibration Target:" -ForegroundColor Magenta
    Write-Host "  [1] Linear Motions (Optimize Forward Wheel Radius)"
    Write-Host "  [2] Rotation Only (Optimize Track Separation)"
    Write-Host "  [3] Strafe Only (Optimize Wheelbase Slip)"
    Write-Host "  [4] Full Combination (Evolve All Parameters)"
    Write-Host "  [5] Manual Step-By-Step Noise Test"
    $CalChoice = Read-Host "Choice (1-5 default=5)"
    
    if ($CalChoice -eq "1") { $RunMode = "Evolution"; $EvoMode = "--linear" }
    elseif ($CalChoice -eq "2") { $RunMode = "Evolution"; $EvoMode = "--rotation" }
    elseif ($CalChoice -eq "3") { $RunMode = "Evolution"; $EvoMode = "--strafe" }
    elseif ($CalChoice -eq "4") { $RunMode = "Evolution"; $EvoMode = "--combo" }
    else { $World = "calibration_bay"; $RunMode = "Calibration" }
}

# 3. Resolve Paths
$WslPath = ($PSScriptRoot -replace '^([A-Za-z]):', '/mnt/$1' -replace '\\', '/').ToLower()

Write-Host "`nLaunching..." -ForegroundColor Green

if ($RunMode -eq "Evolution") {
    Write-Host "`n=== LIVE EVOLUTION: Hardware Parameter Breeding ===" -ForegroundColor Magenta
    wsl -d Ubuntu-24.04 bash -c "python3 '$WslPath/scratch/live_evolution.py' $EvoMode"
}
else {
    # 4. Construct WSL Command
    $WSL_CMD = "export PYTHONUNBUFFERED=1 && source /opt/ros/jazzy/setup.bash && cd '$WslPath' && colcon build --symlink-install && source './install/setup.bash' "

    if ($RunMode -eq "Benchmark") {
        $WSL_CMD += "&& chmod +x scratch/run_benchmark_suite.sh && ./scratch/run_benchmark_suite.sh $World $Planner"
    }
    elseif ($RunMode -eq "Suite") {
        $WSL_CMD += "&& ./scratch/run_robot_test.sh"
    }
    elseif ($RunMode -eq "Calibration") {
        # Force a clean build of the package to clear the 'AndCondition' ImportError cache
        $WSL_CMD += "&& rm -rf build/robot install/robot && colcon build --packages-select robot --symlink-install && source install/setup.bash && (ros2 launch robot sim_gazebo.launch.py world_name:=$World use_mission:=false gui:=true & sleep 12 && ros2 run robot calibration_sequence --ros-args -p use_sim_time:=true)"
    }
    else {
        $WSL_CMD += "&& ros2 launch robot sim_gazebo.launch.py world_name:=$World planner:=$Planner gui:=true"
    }

    # 5. Execute
    wsl -d Ubuntu-24.04 bash -c $WSL_CMD
}

Write-Host "`nSession Completed." -ForegroundColor Cyan
