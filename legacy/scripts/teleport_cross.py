import subprocess
import sys
import math
import platform

def teleport(x, y, yaw_deg, scale):
    yaw_rad = math.radians(yaw_deg)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    
    delete_cmd = "gz service -s /world/master_calibration/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"calib_cross\"'"
    
    # FORCE-CENTERED RIG: Explicitly offsetting the bars to center them on (0,0,0)
    spawn_sdf = f"""
    <sdf version='1.8'>
      <model name='calib_cross'>
        <static>true</static>
        <pose>{x} {y} 0.1 0 0 {yaw_rad}</pose>
        
        <link name='link'>
          <!-- THE ABSOLUTE CENTER (YELLOW) -->
          <visual name='center'>
            <pose>0 0 0 0 0 0</pose>
            <geometry><sphere><radius>0.08</radius></sphere></geometry>
            <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
          </visual>

          <!-- X AXIS (RED) - Centered -->
          <visual name='x'>
            <pose>0 0 0 0 0 0</pose> <!-- Forced Center -->
            <geometry><box><size>{1.0 * scale} 0.04 0.04</size></box></geometry>
            <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
          </visual>

          <!-- Y AXIS (GREEN) - Centered -->
          <visual name='y'>
            <pose>0 0 0 0 0 0</pose> <!-- Forced Center -->
            <geometry><box><size>0.04 {1.0 * scale} 0.04</size></box></geometry>
            <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
          </visual>
        </link>
      </model>
    </sdf>
    """
    
    sdf_path = '/tmp/cross.sdf'
    with open(sdf_path, 'w') as f:
        f.write(spawn_sdf)

    spawn_cmd = f"gz service -s /world/master_calibration/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"{sdf_path}\"'"
    
    print(f"🚀 Deploying SYNCED RIG: X={x}, Y={y}, Scale={scale}")
    
    full_cmd = f"source /opt/ros/jazzy/setup.bash && {delete_cmd} && {spawn_cmd}"
    if platform.system() == "Windows":
        subprocess.run(['wsl', '-d', 'Ubuntu-24.04', 'bash', '-c', full_cmd], capture_output=True)
    else:
        subprocess.run(['bash', '-c', full_cmd], capture_output=True)
        
    print("✅ SYNC COMPLETE.")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        pass
    else:
        try:
            x, y = float(sys.argv[1]), float(sys.argv[2])
            yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            scale = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
            teleport(x, y, yaw, scale)
        except: pass
