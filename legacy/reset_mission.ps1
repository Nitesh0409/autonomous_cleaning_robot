# Hot Reset Mission Script
Write-Host "--- Mission Reset: Teleporting & Reloading Brain... ---" -ForegroundColor Green
wsl -d Ubuntu-24.04 bash -c "cd '/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot' && chmod +x ./robot/reset_robot.sh && ./robot/reset_robot.sh"
