# install_miniconda_admin.ps1
# Right-click this file and select "Run with PowerShell" (Admin)

$InstallerPath = "C:\Users\cclab1\AppData\Local\Temp\Miniconda3-latest.exe"
$InstallDir = "C:\ProgramData\miniconda3"

Write-Host "Starting Miniconda installation to $InstallDir..." -ForegroundColor Cyan

if (-not (Test-Path $InstallerPath)) {
    Write-Host "Installer not found. Downloading again..." -ForegroundColor Yellow
    Invoke-WebRequest -Uri "https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe" -OutFile $InstallerPath
}

# Run the installer silently
$process = Start-Process -FilePath $InstallerPath -ArgumentList "/S", "/InstallationType=AllUsers", "/AddToPath=1", "/RegisterPython=0", "/D=$InstallDir" -Wait -PassThru

if ($process.ExitCode -eq 0) {
    Write-Host "Miniconda installed successfully!" -ForegroundColor Green
    
    # Initialize for PowerShell
    Write-Host "Initializing Conda for PowerShell..."
    & "$InstallDir\Scripts\conda.exe" "init" "powershell"
    
    Write-Host "Installation complete. Please RESTART your terminal." -ForegroundColor Green
    Read-Host "Press Enter to exit"
} else {
    Write-Error "Installation failed with exit code $($process.ExitCode)."
    Read-Host "Press Enter to exit"
}
