import pandas as pd
import glob
import os
import sys

def analyze():
    # Find latest telemetry file
    files = glob.glob("mission_telemetry_*.csv")
    if not files:
        print("❌ No telemetry found.")
        return
    
    latest = max(files, key=os.path.getctime)
    df = pd.read_csv(latest)
    
    print("\n" + "="*50)
    print(f"📊 EVALUATION REPORT: {latest}")
    print("="*50)
    
    min_dist = df['dist_to_block'].min()
    final_x = df['robot_x'].iloc[-1]
    final_y = df['robot_y'].iloc[-1]
    
    print(f"🔹 Closest Approach to Block: {min_dist:.4f}m")
    print(f"🔹 Final Robot Position: ({final_x:.2f}, {final_y:.2f})")
    
    if min_dist < 0.05:
        print("\n✅ GRADE: PLATINUM (Physical Contact Verified)")
    elif min_dist < 0.15:
        print("\n✅ GRADE: GOLD (Within Manipulation Range)")
    else:
        print("\n❌ GRADE: FAIL (Missed Target)")
        
    print("="*50 + "\n")

if __name__ == "__main__":
    analyze()
