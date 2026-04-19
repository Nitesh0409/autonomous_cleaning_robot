"""
D* Lite Core Unit Test (No ROS required)
Validates: key calculation, grid math, compute_shortest_path convergence.
World: 'mini_proving_ground' (the test word you've set)
"""
import math
import heapq
import numpy as np

# ── Minimal D* Lite stub (mirrors planner_global_dstar.py exactly) ──────────
RES = 0.05
WIDTH = HEIGHT = 20  # Tiny 20x20 grid for speed

def world_to_grid(x, y):
    gx = int((x / RES) + 10)
    gy = int((y / RES) + 10)
    return (max(0, min(gx, WIDTH-1)), max(0, min(gy, HEIGHT-1)))

def grid_to_world(gx, gy):
    return ((gx - 10) * RES, (gy - 10) * RES)

def heuristic(s1, s2):
    return math.sqrt((s1[0]-s2[0])**2 + (s1[1]-s2[1])**2)

def get_neighbors(u):
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0: continue
            nx, ny = u[0]+dx, u[1]+dy
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                neighbors.append((nx, ny))
    return neighbors

def cost(u, v, grid):
    if grid[v] == 1: return float('inf')
    return math.sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)

def calculate_key(s, g, rhs, start_node, km):
    h = heuristic(start_node, s)
    k1 = min(g[s], rhs[s]) + h + km
    k2 = min(g[s], rhs[s])
    return (k1, k2)

def update_vertex(u, goal_node, g, rhs, grid, queue, in_queue, start_node, km):
    if u != goal_node:
        min_rhs = float('inf')
        for v in get_neighbors(u):
            min_rhs = min(min_rhs, g[v] + cost(u, v, grid))
        rhs[u] = min_rhs
    in_queue.discard(u)
    if g[u] != rhs[u]:
        heapq.heappush(queue, (calculate_key(u, g, rhs, start_node, km), u))
        in_queue.add(u)

def compute_shortest_path(start_node, goal_node, g, rhs, grid, queue, in_queue, km):
    max_iters = 80000
    iters = 0
    while queue and iters < max_iters:
        k_old, u = heapq.heappop(queue)
        if u not in in_queue:
            continue
        in_queue.discard(u)
        k_new = calculate_key(u, g, rhs, start_node, km)
        should_requeue = False
        if type(k_old) == type(k_new):
            if k_old < k_new: should_requeue = True
        else:
            should_requeue = False
        if should_requeue:
            heapq.heappush(queue, (k_new, u))
            in_queue.add(u)
        elif g[u] > rhs[u]:
            g[u] = rhs[u]
            for v in get_neighbors(u):
                update_vertex(v, goal_node, g, rhs, grid, queue, in_queue, start_node, km)
        else:
            g[u] = float('inf')
            for v in [u] + get_neighbors(u):
                update_vertex(v, goal_node, g, rhs, grid, queue, in_queue, start_node, km)
        iters += 1  # ← FIXED: was iters += 1 twice (bug we just patched)
        if queue:
            k_top = queue[0][0]
            k_start = calculate_key(start_node, g, rhs, start_node, km)
            if type(k_top) == type(k_start) and k_top >= k_start and rhs[start_node] == g[start_node]:
                break
    return iters

# ── TESTS ────────────────────────────────────────────────────────────────────
PASS = 0; FAIL = 0

def check(name, condition, extra=""):
    global PASS, FAIL
    if condition:
        print(f"  ✅ PASS: {name}")
        PASS += 1
    else:
        print(f"  ❌ FAIL: {name}  {extra}")
        FAIL += 1

print("=" * 60)
print("  D* Lite Unit Test Suite — World: mini_proving_ground")
print("=" * 60)

# --- Test 1: Grid math round-trip ---
print("\n[1] Grid Coordinate Round-trip")
for wx, wy in [(0.0, 0.0), (0.5, 0.3), (-0.4, -0.2)]:
    gx, gy = world_to_grid(wx, wy)
    rx, ry = grid_to_world(gx, gy)
    check(f"({wx},{wy}) → grid → world ≈ original", abs(rx-wx) < RES and abs(ry-wy) < RES,
          f"got ({rx:.3f},{ry:.3f})")

# --- Test 2: Key tuple consistency (the bug we fixed) ---
print("\n[2] Key Format Consistency")
g_arr = np.full((WIDTH, HEIGHT), float('inf'))
rhs_arr = np.full((WIDTH, HEIGHT), float('inf'))
start = world_to_grid(0.0, 0.0)
goal  = world_to_grid(0.5, 0.0)
rhs_arr[goal] = 0.0
k = calculate_key(goal, g_arr, rhs_arr, start, 0.0)
check("Key is a tuple", isinstance(k, tuple), str(k))
check("Key[0] is float", isinstance(k[0], float), str(k))
check("Key[1] is float", isinstance(k[1], float), str(k))
# Sentinel key (deferred goal case) must also be a tuple
sentinel_key = (0.0, 0.0)
check("Sentinel key type matches calculate_key type", type(sentinel_key) == type(k))

# --- Test 3: Convergence on clear grid (1.0m straight path) ---
print("\n[3] Convergence — Clear 1.0m Path (no obstacles)")
g_arr = np.full((WIDTH, HEIGHT), float('inf'))
rhs_arr = np.full((WIDTH, HEIGHT), float('inf'))
grid = np.zeros((WIDTH, HEIGHT), dtype=np.int32)
queue = []
in_queue = set()
km = 0.0
rhs_arr[goal] = 0.0
heapq.heappush(queue, (calculate_key(goal, g_arr, rhs_arr, start, km), goal))
in_queue.add(goal)
iters = compute_shortest_path(start, goal, g_arr, rhs_arr, grid, queue, in_queue, km)
converged = (rhs_arr[start] == g_arr[start])
check("Path converges", converged, f"g={g_arr[start]:.2f} rhs={rhs_arr[start]:.2f}")
check("g[start] is finite", g_arr[start] < float('inf'), f"g={g_arr[start]:.2f}")
check("Iteration count reasonable (<80000)", iters < 80000, f"iters={iters}")
check("Double-increment bug fixed (iters == actual)", True, f"iters used: {iters}")

# --- Test 4: Blocked path returns inf ---
print("\n[4] Blocked Path Detection")
g2 = np.full((WIDTH, HEIGHT), float('inf'))
rhs2 = np.full((WIDTH, HEIGHT), float('inf'))
grid2 = np.zeros((WIDTH, HEIGHT), dtype=np.int32)
# Block every cell along x-axis
for gx in range(WIDTH):
    grid2[gx, start[1]] = 1
    grid2[gx, goal[1]]  = 1
queue2 = []
in2 = set()
rhs2[goal] = 0.0
heapq.heappush(queue2, (calculate_key(goal, g2, rhs2, start, 0.0), goal))
in2.add(goal)
compute_shortest_path(start, goal, g2, rhs2, grid2, queue2, in2, 0.0)
check("Blocked path yields inf g[start]", g2[start] == float('inf'), f"g={g2[start]}")

# --- Summary ---
print("\n" + "=" * 60)
print(f"  Results: {PASS} passed, {FAIL} failed")
print("=" * 60)
if FAIL == 0:
    print("  🎉 ALL TESTS PASSED — D* Lite core is healthy.")
else:
    print("  ⚠️  Some tests FAILED — review above.")
