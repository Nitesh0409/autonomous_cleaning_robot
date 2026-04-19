#!/usr/bin/env python3
import os
import glob
import re
import argparse

def evaluate_calibration_logs(log_dir):
    logs = glob.glob(os.path.join(log_dir, "calibration_*.txt"))
    if not logs:
        print("No calibration logs found.")
        return
    
    # Get the latest log
    latest_log = max(logs, key=os.path.getctime)
    print(f"[{latest_log}]\n")
    
    drift_fwd, drift_turn, drift_strafe = None, None, None
    with open(latest_log, 'r') as f:
        content = f.read()
        print(content)
        
        # Parse floats
        for line in content.split('\n'):
            if "Forward" in line:
                m = re.search(r'([-+]\d+\.\d+)%', line)
                if m: drift_fwd = float(m.group(1))
            elif "Turn" in line:
                m = re.search(r'([-+]\d+\.\d+)%', line)
                if m: drift_turn = float(m.group(1))
            elif "Strafe" in line:
                m = re.search(r'([-+]\d+\.\d+)%', line)
                if m: drift_strafe = float(m.group(1))

    print("\n" + "="*50)
    print("🤖 KINEMATIC OPTIMIZER ANALYSIS")
    print("="*50)
    
    # Analyze Forward
    if drift_fwd is not None:
        if abs(drift_fwd) > 2.0:
            correction = (1.0 + (drift_fwd / 100.0))
            print(f"👉 FORWARD ERROR: The wheels are over/under-rotating. ")
            print(f"   SOLUTION: Multiply your URDF plugin <wheel_radius> by {correction:.4f}")
        else:
            print("✅ FORWARD: Wheel Radius is optimally tuned.")

    # Analyze Turn
    if drift_turn is not None:
        if abs(drift_turn) > 2.0:
            # Turning relies on separation and wheelbase
            correction = (1.0 + (drift_turn / 100.0))
            print(f"👉 TURN ERROR: The chassis is rotating at the wrong rate. ")
            print(f"   SOLUTION: Multiply your URDF plugin <wheel_separation> by {correction:.4f}")
        else:
            print("✅ TURN: Chassis Wheel Separation is optimally tuned.")

    # Analyze Strafe
    if drift_strafe is not None:
        if abs(drift_strafe) > 10.0:
            print(f"👉 STRAFE ERROR (CRITICAL): Enormous slip detected!")
            print(f"   SOLUTION: Mecanum sideways motion is physically locked. Check roller friction (mu1/mu2) and fdir1!")
            print("   (Note: Antigravity already fixed this in the previous step, so re-run the calibration first!)")
        elif abs(drift_strafe) > 2.0:
            correction = (1.0 + (drift_strafe / 100.0))
            print(f"👉 STRAFE ERROR: Kinematic footprint is misaligned.")
            print(f"   SOLUTION: Multiply your URDF plugin <wheelbase> by {correction:.4f}")
        else:
            print("✅ STRAFE: Wheelbase and slip parameters are optimal.")
            
    # --- AUTO-APPLY ---
    print("\n" + "-"*50)
    apply = input("Apply these mathematical corrections directly to the URDF now? (y/n): ")
    if apply.lower().strip() == 'y':
        urdf_path = os.path.join(root_dir, 'models', 'robot_dual_arm.urdf')
        with open(urdf_path, 'r') as f:
            urdf = f.read()
            
        if drift_fwd and abs(drift_fwd) > 2.0:
            m = re.search(r'<wheel_radius>([\d.]+)</wheel_radius>', urdf)
            if m:
                new_rad = float(m.group(1)) * (1.0 + drift_fwd/100.0)
                urdf = urdf.replace(m.group(0), f"<wheel_radius>{new_rad:.4f}</wheel_radius>")
                print(f"Updated wheel_radius -> {new_rad:.4f}")
                
        if drift_turn and abs(drift_turn) > 2.0:
            m = re.search(r'<wheel_separation>([\d.]+)</wheel_separation>', urdf)
            if m:
                new_sep = float(m.group(1)) * (1.0 + drift_turn/100.0)
                urdf = urdf.replace(m.group(0), f"<wheel_separation>{new_sep:.4f}</wheel_separation>")
                print(f"Updated wheel_separation -> {new_sep:.4f}")
                
        if drift_strafe and 2.0 < abs(drift_strafe) <= 10.0:
            m = re.search(r'<wheelbase>([\d.]+)</wheelbase>', urdf)
            if m:
                new_base = float(m.group(1)) * (1.0 + drift_strafe/100.0)
                urdf = urdf.replace(m.group(0), f"<wheelbase>{new_base:.4f}</wheelbase>")
                print(f"Updated wheelbase -> {new_base:.4f}")
                
        with open(urdf_path, 'w') as f:
            f.write(urdf)
        print("✅ URDF Patched.")

if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    log_dir = os.path.join(root_dir, 'loggers', 'logs')
    evaluate_calibration_logs(log_dir)

