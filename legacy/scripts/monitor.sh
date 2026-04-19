#!/bin/bash
# Gazebo Real-Time Coordinate Monitor
# Usage: ./monitor.sh

echo "--- GAZEBO REAL-TIME COORDINATE MONITOR ---"
echo "Format: [Timestamp] [Model Name] [X] [Y] [Z]"
echo "Press Ctrl+C to stop."
echo "------------------------------------------"

source /opt/ros/jazzy/setup.bash

# Monitor the garbage block and the robot directly from Gazebo
while true; do
  echo "BLOCK: $(gz topic -e -t /model/garbage_block/pose -n 1 | grep -A 3 'translation' | tr '\n' ' ')"
  echo "ROBOT: $(gz topic -e -t /model/robot/pose -n 1 | grep -A 3 'translation' | tr '\n' ' ')"
  echo "------------------------------------------"
  sleep 2
done
