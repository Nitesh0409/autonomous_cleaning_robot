#!/bin/bash
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS="$(dirname "$SCRIPT_DIR")"
cd "$WS"
colcon build --symlink-install --packages-select robot
source install/setup.bash

# Launch the simulator with GUI in the background
nohup ros2 launch robot sim_gazebo.launch.py world_name:=mini_proving_ground gui:=true > /tmp/sim_gui.log 2>&1 &

echo "Gazebo GUI launched. Waiting 10 seconds for initialization before starting mission..."
sleep 10

# Trigger the mission manager
ros2 topic pub --once /mission/cmd std_msgs/msg/String "{data: 'start'}"
echo "Mission start command sent!"
