#!/bin/bash
# Full Robot Integration Test — D* Lite + Pure Pursuit
# World: mini_proving_ground | Robot: robot (dual_arm URDF)
# Monitors for 60 seconds then prints a summary.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS="$(dirname "$SCRIPT_DIR")"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG="/tmp/robot_test_live.log"

# Aggressive Cleanup
echo "🧹 Cleaning up previous processes..."
pkill -9 -f 'ruby|gz|ros|rviz|planner|manager|nav_monitor' || true
pkill -9 -f 'python3' || true
sleep 1
SUMMARY="/tmp/robot_test_summary.log"

source /opt/ros/jazzy/setup.bash
cd "$WS"
source ./install/setup.bash

echo ""
echo "============================================================"
echo "  LIVE ROBOT TEST — D* Lite + Pure Pursuit"
echo "  World : mini_proving_ground"
echo "  Timeout: 60 seconds"
echo "============================================================"
echo ""

# Launch the full stack headless in background, pipe all stderr+stdout to log
ros2 launch robot sim_gazebo.launch.py headless:=True \
    > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "▶ Launch PID: $LAUNCH_PID — waiting 8s for Gazebo to initialise..."
sleep 8

# Tail the log, filtering for meaningful events
echo ""
echo "--- Live Event Stream (50s window) ---"
timeout 50 tail -F "$LOG" 2>/dev/null | grep --line-buffered \
    -E "(ERROR|WARN|D\* status|ARRIVED|DRIVE|New Target|waypoints|converged|iter cap|ODOM HEARTBEAT|Path Received|Rail Updated|OMEGA|PICKUP|SUCCESS|Garbage|GRIP|IK|PURE PURSUIT ACTIVE|D\* Lite Global)" \
    | while IFS= read -r line; do
        echo "$(date +%H:%M:%S)  $line"
    done

# Collect summary stats
echo ""
echo "============================================================"
echo "  TEST SUMMARY"
echo "============================================================"

# Using head -n 1 to prevent potential multi-line expansion if LOG corresponds to multiple files or aliases
ODOM_COUNT=$(grep -c "ODOM HEARTBEAT" "$LOG" 2>/dev/null | head -n 1)
PATH_COUNT=$(grep -c "Rail Updated"   "$LOG" 2>/dev/null | head -n 1)
ITER_CAP=$(grep -c "iter cap"        "$LOG" 2>/dev/null | head -n 1)
CONVERGED=$(grep -c "waypoints="     "$LOG" 2>/dev/null | head -n 1)
ARRIVED=$(grep -c "ARRIVED"          "$LOG" 2>/dev/null | head -n 1)
SUCCESS=$(grep -c "SUCCESS\|Garbage Deposited" "$LOG" 2>/dev/null | head -n 1)
ERRORS=$(grep -c "^.*\[ERROR\]"      "$LOG" 2>/dev/null | head -n 1)

# Ensure defaults
ODOM_COUNT=${ODOM_COUNT:-0}
PATH_COUNT=${PATH_COUNT:-0}
ITER_CAP=${ITER_CAP:-0}
CONVERGED=${CONVERGED:-0}
ARRIVED=${ARRIVED:-0}
SUCCESS=${SUCCESS:-0}
ERRORS=${ERRORS:-0}

echo "  Odometry pulses    : $ODOM_COUNT"
echo "  Paths published    : $PATH_COUNT"
echo "  D* iter-cap hits   : $ITER_CAP  (0 = good)"
echo "  Planning cycles    : $CONVERGED"
echo "  Local goal arrivals: $ARRIVED"
echo "  Mission successes  : $SUCCESS"
echo "  ROS Errors         : $ERRORS"
echo ""

if [ "$SUCCESS" -gt 0 ] 2>/dev/null; then
    echo "  🎉 RESULT: MISSION COMPLETE"
elif [ "$ARRIVED" -gt 0 ] 2>/dev/null; then
    echo "  ✅ RESULT: NAVIGATION WORKING (mission may be incomplete)"
elif [ "$PATH_COUNT" -gt 0 ] 2>/dev/null; then
    echo "  🔶 RESULT: PATH FOUND but robot not arriving — check local planner"
elif [ "$ODOM_COUNT" -gt 0 ] 2>/dev/null; then
    echo "  🔴 RESULT: ODOM OK but D* not producing paths — check planner"
else
    echo "  ❌ RESULT: NO ODOMETRY — bridge or physics stall"
fi
echo "============================================================"
echo ""
echo "Full log: $LOG"

# Kill the launch
kill $LAUNCH_PID 2>/dev/null
sleep 2
pkill -f "gz sim|planner_global|planner_local|cleaner_mission" 2>/dev/null || true
