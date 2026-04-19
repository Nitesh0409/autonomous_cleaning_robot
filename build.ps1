# ROS 2 Build Script for Robotics Project
Write-Host "Starting Build Process..." -ForegroundColor Cyan

# 1. Build the specific package
colcon build --packages-select robot --symlink-install

# 2. Source the environment if build succeeded
if ($LASTEXITCODE -eq 0) {
    Write-Host "Build Successful!" -ForegroundColor Green
    if (Test-Path ".\install\setup.ps1") {
        Write-Host "Sourcing environment..." -ForegroundColor Yellow
        . .\install\setup.ps1
    }
    Write-Host "Done. You can now run: ros2 launch robot patrol_launch.py" -ForegroundColor Cyan
} else {
    Write-Host "Build Failed. Please check the errors above." -ForegroundColor Red
}
