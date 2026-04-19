import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Path
import os
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('path_plotter')

    print('Waiting for D* Lite path...')
    path_msg = None
    
    def cb(msg):
        nonlocal path_msg
        path_msg = msg
        
    sub = node.create_subscription(Path, 'plan', cb, 10)
    
    # Wait up to 10 seconds
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if path_msg is not None:
            break
            
    if path_msg is None:
        print("No path received.")
        sys.exit(1)

    xs = [p.pose.position.x for p in path_msg.poses]
    ys = [p.pose.position.y for p in path_msg.poses]

    plt.figure(figsize=(8,8))
    plt.plot(xs, ys, 'b.-', label='Waypoints')
    plt.plot(xs[0], ys[0], 'go', markersize=10, label='Start')
    plt.plot(xs[-1], ys[-1], 'ro', markersize=10, label='Goal')
    plt.title('D* Lite Planned Path')
    plt.xlabel('World X (m)')
    plt.ylabel('World Y (m)')
    plt.axis('equal')
    plt.grid()
    plt.legend()
    
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(root_dir, 'artifacts')
    os.makedirs(out_dir, exist_ok=True)
    plt.savefig(os.path.join(out_dir, 'dstar_path.png'))
    print('Path plotted to artifacts/dstar_path.png')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
