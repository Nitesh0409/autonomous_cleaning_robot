#!/usr/bin/env python3
"""Patch planner_global_dstar.py: remove interior obstacle pre-marking."""
import re, sys

path = "/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot/robot/planner_global_dstar.py"

with open(path, "r", encoding="utf-8") as f:
    src = f.read()

# Replace the entire pre_populate_static_map body (lines 67-97)
old_pattern = re.compile(
    r'(    def pre_populate_static_map\(self\):\r?\n)'
    r'.*?'
    r'(            "Static map: perimeter walls \+ patrol_arena interior pre-loaded"\)\r?\n)',
    re.DOTALL
)

new_body = (
    '    def pre_populate_static_map(self):\n'
    '        """\n'
    '        Pre-mark the 6x6 arena perimeter walls in the D* Lite grid.\n'
    '        Interior obstacles are NOT pre-marked here -- they are discovered\n'
    '        via LiDAR at runtime, so this works correctly in all worlds.\n'
    '        """\n'
    '        INF  = 3.0    # arena half-size (m)\n'
    '        T    = 0.15   # wall thickness (m)\n'
    '        INFL = 0.20   # extra inflation for robot clearance (m)\n'
    '\n'
    '        # North / South walls (span full outer width)\n'
    '        self.add_static_box(0,     INF,  INF + T/2,  T/2 + INFL)\n'
    '        self.add_static_box(0,    -INF,  INF + T/2,  T/2 + INFL)\n'
    '        # East / West walls (span inner height, flush to N/S wall faces)\n'
    '        self.add_static_box( INF,  0,    T/2 + INFL, INF - T/2)\n'
    '        self.add_static_box(-INF,  0,    T/2 + INFL, INF - T/2)\n'
    '\n'
    '        self.get_logger().info("Static map: perimeter walls pre-loaded")\n'
)

match = old_pattern.search(src)
if not match:
    print("ERROR: pattern not found", file=sys.stderr)
    sys.exit(1)

new_src = src[:match.start()] + new_body + src[match.end():]
with open(path, "w", encoding="utf-8") as f:
    f.write(new_src)

print("PATCHED OK")
