#!/bin/bash
# High-Speed Reset Script for Gazebo & ROS 2
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

echo "--- Stopping Navigation Brain ---"
pkill -f 'planner_local_apf' || true

echo "--- Teleporting Robot to (0,0) ---"
gz service -s /world/basic_world/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --req 'name: "robot", position: {x: 0.0, y: 0.0, z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'

echo "--- Relaunching Navigation (Live Code Active) ---"
export PYTHONUNBUFFERED=1
ros2 run robot planner_local_apf
