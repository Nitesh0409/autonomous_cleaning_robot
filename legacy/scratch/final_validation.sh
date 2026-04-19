#!/usr/bin/env bash
# final_validation.sh  — run via: wsl bash /path/final_validation.sh
set -e
BASE="/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot"
cd "$BASE"

echo "=== PYTHON SYNTAX ==="
PY_FILES=(
  "robot/planner_global_astar.py"
  "robot/planner_global_dstar.py"
  "robot/planner_local_apf.py"
  "robot/planner_local_dwa.py"
  "robot/patrol_mission_v2.py"
  "robot/obstacle_tracker.py"
  "robot/garbage_spawner.py"
  "robot_ui.py"
  "launch/sim_gazebo.launch.py"
  "launch/patrol_launch.py"
)
all_ok=1
for f in "${PY_FILES[@]}"; do
  if python3 -m py_compile "$f" 2>&1; then
    echo "  OK  $f"
  else
    echo "  FAIL $f"
    all_ok=0
  fi
done

echo ""
echo "=== XML VALIDATION ==="
XML_FILES=(
  "worlds/patrol_arena.sdf"
  "worlds/arm_test.sdf"
  "worlds/static_obstacles.sdf"
  "worlds/dynamic_obstacles.sdf"
  "models/robot_dual_arm.urdf"
  "models/garbage.urdf"
)
for f in "${XML_FILES[@]}"; do
  if xmllint --noout "$f" 2>&1; then
    echo "  OK  $f"
  else
    echo "  FAIL $f"
    all_ok=0
  fi
done

echo ""
echo "=== MESH FILE ==="
if [ -f "meshes/mecanum_wheel.dae" ]; then
  echo "  OK  meshes/mecanum_wheel.dae"
else
  echo "  MISSING meshes/mecanum_wheel.dae"
  all_ok=0
fi

echo ""
echo "=== MESH URI (must be package://, not model://) ==="
pkg_count=$(grep -c 'package://robot' models/robot_dual_arm.urdf || true)
model_count=$(grep -c 'model://' models/robot_dual_arm.urdf || true)
echo "  package:// references: $pkg_count"
echo "  model://   references: $model_count (must be 0)"
[ "$model_count" -eq 0 ] || all_ok=0

echo ""
echo "=== WALL CORNER FIX (E/W walls must be 5.85 not 6.3) ==="
for f in worlds/patrol_arena.sdf worlds/static_obstacles.sdf worlds/dynamic_obstacles.sdf; do
  bad=$(grep -c '0.15 6.3 0.12' "$f" || true)
  if [ "$bad" -eq 0 ]; then
    echo "  OK  $f (no interlacing walls)"
  else
    echo "  FAIL $f ($bad interlacing wall(s) found)"
    all_ok=0
  fi
done

echo ""
if [ "$all_ok" -eq 1 ]; then
  echo "=== ALL CHECKS PASSED ==="
else
  echo "=== SOME CHECKS FAILED — see above ==="
fi
