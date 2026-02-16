#!/usr/bin/env python3
# debug_path.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathDebugger(Node):
    def __init__(self):
        super().__init__('path_debugger')
        self.create_subscription(Path, '/piec/path', self.path_cb, 10)
        self.get_logger().info("Path debugger started")
        
    def path_cb(self, msg):
        self.get_logger().info(f"\n=== New Path Received ===")
        self.get_logger().info(f"Path has {len(msg.poses)} waypoints")
        for i, pose_stamped in enumerate(msg.poses):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.get_logger().info(f"  Waypoint {i}: x={x:.3f}, y={y:.3f}")

def main():
    rclpy.init()
    node = PathDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
