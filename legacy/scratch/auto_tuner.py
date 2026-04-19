#!/usr/bin/env python3
import math
import numpy as np
import json
import os
import random
import copy
import time
import sys

# ==============================================================
# 1. OBSTACLE DATABASE (Channel Gauntlet)
# ==============================================================
OBSTACLES = [
    {"name": "bot_wall", "cx": 6.0, "cy": 0.5, "sx": 12.0, "sy": 0.2},
    {"name": "top_wall", "cx": 6.0, "cy": 5.5, "sx": 12.0, "sy": 0.2},
    {"name": "b1_a", "cx": 2.0, "cy": 2.0, "sx": 0.4, "sy": 3.0}, 
    {"name": "b1_b", "cx": 2.0, "cy": 5.5, "sx": 0.4, "sy": 1.0},
    {"name": "b2_a", "cx": 4.0, "cy": 4.0, "sx": 0.4, "sy": 3.0}, 
    {"name": "b2_b", "cx": 4.0, "cy": 0.5, "sx": 0.4, "sy": 1.0},
    {"name": "b3_a", "cx": 6.0, "cy": 2.0, "sx": 0.4, "sy": 3.0}, 
    {"name": "b3_b", "cx": 6.0, "cy": 5.5, "sx": 0.4, "sy": 1.0},
    {"name": "b4_a", "cx": 8.0, "cy": 4.5, "sx": 0.4, "sy": 2.5}, 
    {"name": "b4_b", "cx": 8.0, "cy": 1.5, "sx": 0.4, "sy": 2.5},
]

START_X, START_Y = 0.0, 3.0
GOAL_X, GOAL_Y = 10.0, 3.0

# ==============================================================
# 2. PARAMETER SPACE
# ==============================================================
PARAM_SPACE = {
    "max_speed":         {"min": 0.4, "max": 0.9, "step": 0.1},
    "collision_radius":  {"min": 0.18, "max": 0.35, "step": 0.05},
    "dist_weight":       {"min": 5.0, "max": 25.0, "step": 1.0},
    "yaw_weight":        {"min": 1.0, "max": 15.0, "step": 1.0},
    "progress_weight":   {"min": 5.0, "max": 20.0, "step": 1.0},
    "strafe_penalty":    {"min": 0.0, "max": 12.0, "step": 1.0},
    "spin_weight":       {"min": 0.0, "max": 12.0, "step": 1.0},
    "inflation_radius":  {"min": 0.10, "max": 0.30, "step": 0.02},
}

def random_params():
    return {k: round(v["min"] + random.randint(0, int((v["max"]-v["min"])/v["step"])) * v["step"], 3) for k, v in PARAM_SPACE.items()}

def mutate(params, rate=0.3):
    p = copy.deepcopy(params)
    for k, v in PARAM_SPACE.items():
        if random.random() < rate:
            p[k] = round(max(v["min"], min(v["max"], p[k] + random.choice([-1, 1]) * v["step"])), 3)
    return p

def crossover(p1, p2):
    return {k: (p1[k] if random.random() < 0.5 else p2[k]) for k in PARAM_SPACE}

# ==============================================================
# 3. UTILITIES
# ==============================================================
def min_clearance_to_obstacles(px, py):
    min_d = float('inf')
    for obs in OBSTACLES:
        dx = max(0, abs(px - obs["cx"]) - obs["sx"] / 2.0)
        dy = max(0, abs(py - obs["cy"]) - obs["sy"] / 2.0)
        d = math.sqrt(dx**2 + dy**2)
        if d < min_d: min_d = d
    return min_d

# ==============================================================
# 4. SIMULATION ENGINE (ULTRA-SPEED)
# ==============================================================
def simulate_run(params):
    rx, ry, yaw = START_X, START_Y, 0.0
    dt, predict_time, max_steps = 0.1, 2.0, 500
    collisions, min_clearance, stall_counter = 0, float('inf'), 0
    
    # Pre-calculate velocity search space
    v_s, w_s = 5, 5
    VX = np.linspace(0.1, params["max_speed"], v_s)
    VY = np.linspace(-params["max_speed"], params["max_speed"], v_s)
    WZ = np.linspace(-1.5, 1.5, w_s)
    n_predict = int(predict_time / dt)
    
    for step_idx in range(max_steps):
        if math.sqrt((rx - GOAL_X)**2 + (ry - GOAL_Y)**2) < 0.5:
            return True, collisions, min_clearance, step_idx
        
        clr = min_clearance_to_obstacles(rx, ry)
        min_clearance = min(min_clearance, clr)
        if clr < 0.18: return False, collisions, min_clearance, step_idx
        
        best_v, best_score = (0.0, 0.0, 0.0), -float('inf')
        for tvx in VX:
            for tvy in VY:
                for twz in WZ:
                    # In-line projection and collision check
                    is_collidable = False
                    tx, ty, tyaw = rx, ry, yaw
                    for _ in range(n_predict):
                        tx += (tvx * math.cos(tyaw) - tvy * math.sin(tyaw)) * dt
                        ty += (tvx * math.sin(tyaw) + tvy * math.cos(tyaw)) * dt
                        tyaw += twz * dt
                        if _ % 5 == 0:
                            if min_clearance_to_obstacles(tx, ty) < params["collision_radius"]:
                                is_collidable = True; break
                    
                    if is_collidable: score = -2000.0
                    else:
                        d = math.sqrt((tx-GOAL_X)**2 + (ty-GOAL_Y)**2)
                        ye = math.atan2(GOAL_Y-ty, GOAL_X-tx) - tyaw
                        while ye > math.pi: ye -= 2*math.pi
                        while ye < -math.pi: ye += 2*math.pi
                        
                        strafe_cost = abs(tvy) * params.get("strafe_penalty", 1.0)
                        spin_cost = abs(twz) * params.get("spin_weight", 2.0) if tvx < 0.2 else 0
                        
                        score = -(d * params["dist_weight"]) - (abs(ye) * params["yaw_weight"]) + \
                                ((tx-rx) * params["progress_weight"]) - strafe_cost - spin_cost
                    
                    if score > best_score: best_score, best_v = score, (tvx, tvy, twz)
        
        vx, vy, wz = best_v
        if abs(vx) < 0.05 and abs(vy) < 0.05 and abs(wz) < 0.05:
            stall_counter += 1
        else:
            stall_counter = 0
            
        if stall_counter > 30: return False, collisions, min_clearance, step_idx
        
        rx += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        ry += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw += wz * dt
        if rx > 9.5: return True, collisions, min_clearance, step_idx
        
    return False, collisions, min_clearance, 500

# ==============================================================
# 5. EVOLUTIONARY LOOP
# ==============================================================
def fitness(params):
    success, collisions, clearance, steps = simulate_run(params)
    score = (2000.0 + (500 - steps) * 10.0) if success else 0.0
    score -= collisions * 500.0
    score += (clearance * 20.0)
    return score, {"success": success, "collisions": collisions, "steps": steps}

def run_evolution(pop_size=20, generations=30):
    print("AUTO-TUNER: Evolutionary Optimizer v3 (NITRO-SPEED)")
    population = [random_params() for _ in range(pop_size)]
    best_p, best_s = None, -float('inf')
    
    for gen in range(generations):
        print(f"Gen {gen+1:02d} | Training: ", end=""); sys.stdout.flush()
        scored = []
        for p in population:
            sc, meta = fitness(p)
            scored.append((sc, p, meta))
            print(".", end=""); sys.stdout.flush()
        print("")
        
        scored.sort(key=lambda x: x[0], reverse=True)
        if scored[0][0] > best_s: best_s, best_p = scored[0][0], scored[0][1]
        
        successes = sum(1 for x in scored if x[2]["success"])
        print(f"  Best: {scored[0][0]:.1f} | Success: {successes}/{pop_size}")
        
        if successes == pop_size and gen > 10: break
        
        survivors = [x[1] for x in scored[:pop_size//2]]
        population = survivors + [mutate(crossover(random.choice(survivors), random.choice(survivors))) for _ in range(pop_size - len(survivors))]
        
    return best_p

def apply_params(p):
    dwa_path = "robot/planner_local_dwa.py"
    dstar_path = "robot/planner_global_dstar.py"
    import re
    if os.path.exists(dwa_path):
        with open(dwa_path, 'r', encoding='utf-8') as f: code = f.read()
        code = re.sub(r"max_speed',\s*[\d.]+", f"max_speed', {p['max_speed']}", code)
        code = re.sub(r"dist_to_target \* [\d.]+", f"dist_to_target * {p['dist_weight']}", code)
        code = re.sub(r"abs\(yaw_error\) \* [\d.]+", f"abs(yaw_error) * {p['yaw_weight']}", code)
        code = re.sub(r"progress \* [\d.]+", f"progress * {p['progress_weight']}", code)
        with open(dwa_path, 'w', encoding='utf-8') as f: f.write(code)
    if os.path.exists(dstar_path):
        with open(dstar_path, 'r', encoding='utf-8') as f: code = f.read()
        code = re.sub(r"self\.inflation = [\d.]+", f"self.inflation = {p['inflation_radius']}", code)
        with open(dstar_path, 'w', encoding='utf-8') as f: f.write(code)
    print("\n✅ PARAMETERS PATCHED. READY FOR GAUNTLET.")

if __name__ == "__main__":
    best = run_evolution()
    apply_params(best)
