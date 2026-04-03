# register_wsl.ps1
# Run this from an elevated (Admin) PowerShell.

$DistroName = "Ubuntu-Manual"
$InstallPath = "C:\WSL\Ubuntu_Manual"
$VHDXPath = "C:\WSL\Ubuntu2204\ext4.vhdx"

Write-Host "Registering WSL distribution '$DistroName' from $VHDXPath..." -ForegroundColor Cyan

if (-not (Test-Path $VHDXPath)) {
    Write-Error "Could not find VHDX at $VHDXPath"
    return
}

# Create installation directory if it doesn't exist
if (-not (Test-Path $InstallPath)) {
    New-Item -ItemType Directory -Path $InstallPath -Force
}

# Standard import command
# In WSL 2, this will link the VHDX.
wsl --import $DistroName $InstallPath $VHDXPath --version 2

if ($LASTEXITCODE -eq 0) {
    Write-Host "Successfully registered '$DistroName'!" -ForegroundColor Green
    Write-Host "Setting as default..."
    wsl --set-default $DistroName
    
    Write-Host "Checking for ROS inside the distribution..."
    wsl -d $DistroName -u root -- bash -c "source /opt/ros/*/setup.bash 2>/dev/null; ros2 --version || rosversion -a"
} else {
    Write-Error "Failed to register distribution. Error code: $LASTEXITCODE"
}
