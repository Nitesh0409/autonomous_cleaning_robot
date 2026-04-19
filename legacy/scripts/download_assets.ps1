# Download Assets Script for Dual Arm Robot
$meshDir = "meshes"
if (-not (Test-Path $meshDir)) { New-Item -ItemType Directory $meshDir }

$urls = @{
    "base.stl" = "https://raw.githubusercontent.com/linorobot/linorobot2_dev/galactic/linorobot2_description/meshes/mecanum_prop_base.stl"
    "mecanum_wheel.stl" = "https://raw.githubusercontent.com/linorobot/linorobot2_dev/galactic/linorobot2_description/meshes/mecanum_wheel.stl"
    "arm_link.stl" = "https://raw.githubusercontent.com/ROBOTIS-GIT/open_manipulator/main/open_manipulator_description/meshes/open_manipulator_x/link1.stl"
    "arm_upper.stl" = "https://raw.githubusercontent.com/ROBOTIS-GIT/open_manipulator/main/open_manipulator_description/meshes/open_manipulator_x/link2.stl"
    "arm_lower.stl" = "https://raw.githubusercontent.com/ROBOTIS-GIT/open_manipulator/main/open_manipulator_description/meshes/open_manipulator_x/link3.stl"
    "arm_wrist.stl" = "https://raw.githubusercontent.com/ROBOTIS-GIT/open_manipulator/main/open_manipulator_description/meshes/open_manipulator_x/link4.stl"
}



foreach ($file in $urls.Keys) {
    Write-Host "Downloading $file..."
    $url = $urls[$file]
    # Using curl.exe for native L redirection support if Invoke-WebRequest fails
    curl.exe -L $url -o "$meshDir/$file"
}

Write-Host "Assets download complete."
