#!/usr/bin/env python3
import os
import time
import subprocess
import copy
import random
import glob
import re

# ==============================================================
# LIVE EVOLUTIONARY KINEMATICS ENGINE
# ==============================================================

POPULATION_SIZE = 4
GENERATIONS = 5

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
urdf_path = os.path.join(root_dir, 'models', 'robot_dual_arm.urdf')
log_dir = os.path.join(root_dir, 'loggers', 'logs')

def get_latest_log_time():
    logs = glob.glob(os.path.join(log_dir, "calibration_*.txt"))
    return max(os.path.getctime(f) for f in logs) if logs else 0

def kill_gazebo():
    print("🧹 NUCLEAR CLEANUP: Purging all ghost processes...")
    # Using pkill -9 -f to ensure bridges and hidden gazebo servers are caught
    subprocess.run("pkill -9 -f 'gz|rviz2|robot_state_publisher|ros_gz_bridge|static_transform_publisher|calibration_sequence'", shell=True, stderr=subprocess.DEVNULL)
    time.sleep(3)

def set_urdf_params(wheelbase, wheel_sep, wheel_rad):
    with open(urdf_path, 'r') as f:
        urdf = f.read()
    
    urdf = re.sub(r'<wheelbase>[\d.]+</wheelbase>', f'<wheelbase>{wheelbase:.4f}</wheelbase>', urdf)
    urdf = re.sub(r'<wheel_separation>[\d.]+</wheel_separation>', f'<wheel_separation>{wheel_sep:.4f}</wheel_separation>', urdf)
    urdf = re.sub(r'<wheel_radius>[\d.]+</wheel_radius>', f'<wheel_radius>{wheel_rad:.4f}</wheel_radius>', urdf)

    with open(urdf_path, 'w') as f:
        f.write(urdf)

def run_simulation(individual):
    # 1. Inject DNA into the physical robot firmware
    set_urdf_params(individual['base'], individual['sep'], individual['rad'])
    
    # 2. Build changes
    print(f"🧬 Booting DNA: Base={individual['base']:.3f}, Sep={individual['sep']:.3f}, Rad={individual['rad']:.3f}")
    res = subprocess.run(["colcon", "build", "--packages-select", "robot"], cwd=root_dir)
    if res.returncode != 0:
        print("❌ BUILD FAILED. Check your ROS installation inside WSL.")
        return 10000.0
    
    # 3. Launch the World (Removed -s to keep it visible!)
    print(f"🌍 Spawning Simulation World & Robot...")
    launch_cmd = "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot sim_gazebo.launch.py gz_args:='-r'"
    sim_proc = subprocess.Popen(["bash", "-c", launch_cmd], cwd=root_dir)
    
    time.sleep(15) # Increased to let physics settle
    
    import sys
    mode_arg = ""
    if "--rotation" in sys.argv: mode_arg = "--rotation"
    if "--linear" in sys.argv: mode_arg = "--linear"
    if "--strafe" in sys.argv: mode_arg = "--strafe"

    # 4. Launch Auto-Calibration
    print(f"🤖 Commencing Auto-Calibration Test...")
    calib_cmd = f"source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run robot calibration_sequence --auto {mode_arg}"
    cal_proc = subprocess.Popen(["bash", "-c", calib_cmd], cwd=root_dir)
    
    # Wait for the new log to drop
    timeout = 60
    start = time.time()
    drift_fwd, drift_turn, drift_strafe = 5000.0, 5000.0, 5000.0
    success = False
    
    start_logs = glob.glob(os.path.join(log_dir, "calibration_*.txt"))
    start_log_count = len(start_logs)
    
    while time.time() - start < timeout:
        logs = glob.glob(os.path.join(log_dir, "calibration_*.txt"))
        if len(logs) > start_log_count:
            latest = max(logs, key=os.path.getctime)
            time.sleep(2) # let it finish writing
            try:
                with open(latest, 'r') as f:
                    content = f.read()
                    if "CALIBRATION FINAL REPORT" in content:
                        for line in content.split('\n'):
                            m = re.search(r'([-+]\d+\.\d+)%', line)
                            if m:
                                if "Forward" in line: drift_fwd = float(m.group(1))
                                if "Turn" in line: drift_turn = float(m.group(1))
                                if "Strafe" in line: drift_strafe = float(m.group(1))
                        success = True
                        break
            except Exception:
                pass
        time.sleep(2)
        print(".", end="", flush=True)
        
    print("")
    kill_gazebo()
    
    # Calculate Fitness (0 error is perfect)
    if success:
        if mode_arg == "--rotation": score = abs(drift_turn)
        elif mode_arg == "--linear": score = abs(drift_fwd)
        elif mode_arg == "--strafe": score = abs(drift_strafe)
        else: score = abs(drift_fwd) + abs(drift_turn) + abs(drift_strafe)
        
        print(f"🎯 SCORE: {score:.1f}% Total Isolated Error (F:{drift_fwd}%, T:{drift_turn}%, S:{drift_strafe}%)")
        return score
    else:
        print("❌ Test Failed / Timeout")
        return float('inf')

def run_evolution():
    # Dynamically inject the most recently saved parameters from URDF
    cur_urdf = ""
    with open(URDF_PATH, "r") as f:
        cur_urdf = f.read()
    b_match = re.search(r'<wheelbase>([\d.]+)</wheelbase>', cur_urdf)
    s_match = re.search(r'<wheel_separation>([\d.]+)</wheel_separation>', cur_urdf)
    r_match = re.search(r'<wheel_radius>([\d.]+)</wheel_radius>', cur_urdf)
    
    cb = float(b_match.group(1)) if b_match else 0.20
    cs = float(s_match.group(1)) if s_match else 0.24
    cr = float(r_match.group(1)) if r_match else 0.03
    
    print(f"🧬 Inherited Base DNA -> Base: {cb}, Sep: {cs}, Rad: {cr}")

    # Baseline - Dynamically branches out from the current URDF
    population = [
        {"base": cb, "sep": cs, "rad": cr},
        {"base": cb*0.9, "sep": cs*0.9, "rad": cr},
        {"base": cb*1.1, "sep": cs*1.1, "rad": cr},
        {"base": cb*0.8, "sep": cs*1.2, "rad": cr},
    ]
    
    best_dna = None
    best_score = float('inf')
    
    for gen in range(GENERATIONS):
        print("\n" + "="*50)
        print(f"🧬 GENERATION {gen+1} / {GENERATIONS}")
        print("="*50)
        
        scored = []
        for i, ind in enumerate(population):
            print(f"\n--- Testing Subject {i+1}/{len(population)} ---")
            score = run_simulation(ind)
            scored.append((score, ind))
            
        scored.sort(key=lambda x: x[0])
        
        if scored[0][0] < best_score:
            best_score = scored[0][0]
            best_dna = scored[0][1]
            
        print(f"\n🏆 Generation {gen+1} Winner: {best_score:.1f}% Error")
        
        # Evolve (Elitism + Random Mutation)
        survivors = [x[1] for x in scored[:2]]
        
        population = [copy.deepcopy(survivors[0]), copy.deepcopy(survivors[1])]
        
        for _ in range(POPULATION_SIZE - 2):
            parent = copy.deepcopy(random.choice(survivors))
            import sys
            if "--rotation" in sys.argv:
                parent['sep'] = max(0.05, parent['sep'] * random.uniform(0.70, 1.30)) 
            elif "--linear" in sys.argv:
                parent['rad'] = max(0.01, parent['rad'] * random.uniform(0.85, 1.15))
            elif "--strafe" in sys.argv:
                parent['base'] = max(0.05, parent['base'] * random.uniform(0.70, 1.30))
            else: # combo
                parent['base'] = max(0.05, parent['base'] * random.uniform(0.85, 1.15))
                parent['sep'] = max(0.05, parent['sep'] * random.uniform(0.85, 1.15))
                parent['rad'] = max(0.01, parent['rad'] * random.uniform(0.95, 1.05))
            population.append(parent)
            
    print("\n" + "="*50)
    print(f"🎉 EVOLUTION COMPLETE!")
    print(f"Best DNA: {best_dna}")
    
    if best_dna is not None:
        print(f"To lock it in, URDF is already patched to the final mutation!")
        # Inject best
        set_urdf_params(best_dna['base'], best_dna['sep'], best_dna['rad'])
    else:
        print("No successful DNA to patch.")


if __name__ == "__main__":
    kill_gazebo()
    run_evolution()
