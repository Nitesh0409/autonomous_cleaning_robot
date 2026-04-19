import os
import time
import subprocess

def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

def run_wsl(cmd):
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # Convert Windows path to WSL path
    wsl_root = root_dir.replace('C:', '/mnt/c').replace('\\', '/')
    full_cmd = f"wsl -d Ubuntu-24.04 bash -c \"source /opt/ros/jazzy/setup.bash; source '{wsl_root}/install/setup.bash'; {cmd}\""
    subprocess.run(full_cmd, shell=True)

def main():
    while True:
        clear()
        print("========================================================")
        print("     🛡️  MICRO-SENTINEL MISSION CONTROL (v7.5)  🛡️")
        print("========================================================")
        print(" [1] LINEAR SURGE      : 1.0m Forward propulsion test")
        print(" [2] HOLONOMIC STRAFE  : 0.5m Sideways gliding test")
        print(" [3] POLAR PIVOT       : 360-degree Zero-radius spin")
        print(" [4] DIAGONAL GLIDE    : 45-degree holonomic vector test")
        print(" [5] PICKUP CHALLENGE  : Autonomous search & store cycle")
        print(" [L] VIEW NAV LOGS     : Check the latest telemetry data")
        print(" [Q] EXIT              : Terminate control suite")
        print("========================================================")
        
        choice = input("Select Test Strategy: ").upper()
        
        if choice == '1':
            print("🚀 Launching Linear Surge...")
            run_wsl("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'; sleep 2; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}}'")
        elif choice == '2':
            print("↔️ Launching Holonomic Strafe...")
            run_wsl("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.5}}'; sleep 1; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}}'")
        elif choice == '3':
            print("🌀 Launching Polar Pivot...")
            run_wsl("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 1.0}}'; sleep 3.14; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.0}}'")
        elif choice == '4':
            print("📐 Launching Diagonal Glide...")
            run_wsl("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.5}}'; sleep 2; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}}'")
        elif choice == '5':
            print("🤖 TRIGGERING AUTONOMOUS MISSION MANAGER...")
            # This triggers the mission manager's garbage search loop
            run_wsl("ros2 topic pub --once /mission/start std_msgs/msg/Bool '{data: true}'")
        elif choice == 'L':
            print("📊 Inspecting Navigation Logs...")
            run_wsl("ls -t telemetry_log_*.csv | head -n 1 | xargs cat")
            input("\nPress Enter to return to menu...")
        elif choice == 'Q':
            break
        
        time.sleep(1)

if __name__ == "__main__":
    main()
