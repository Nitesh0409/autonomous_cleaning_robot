# --- ELEVATED SETUP SCRIPT FOR SMART DUSTBIN ---
# 1. Run PowerShell as Administrator
# 2. Copy and Paste the following commands:

# Enable WSL 2 and Virtual Machine Platform
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# Set WSL 2 as default
wsl --set-default-version 2

# Install Ubuntu 22.04 (Humble compatible)
wsl --install -d Ubuntu-22.04

# --- MANIFEST ---
# This script prepares the laptop as the "Mastermind" brain.
# It enables the virtualization features required for Gazebo.
# After running, please REBOOT YOUR PC.
