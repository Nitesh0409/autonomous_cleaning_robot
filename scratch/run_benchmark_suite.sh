#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS="$(dirname "$SCRIPT_DIR")"
source /opt/ros/jazzy/setup.bash
cd "$WS"
source ./install/setup.bash

echo "🚀 Launching Physics Simulator..."
WORLD_TARGET=${1:-benchmark_obstacle}
PLANNER_TARGET=${2:-apf}
ros2 launch robot sim_gazebo.launch.py world_name:=$WORLD_TARGET planner:=$PLANNER_TARGET headless:=false &
LAUNCH_PID=$!

echo "🏁 Starting Benchmark Python Node (INSTANT IGNITION)..."
python3 scratch/run_benchmark.py

echo "Cleaning up..."
kill $LAUNCH_PID
pkill -f "gz sim|planner_global|planner_local|cleaner_mission"
