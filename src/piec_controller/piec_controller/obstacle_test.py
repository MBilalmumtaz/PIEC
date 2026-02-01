#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

class ObstacleTest(Node):
    def __init__(self):
        super().__init__('obstacle_test')
        
        # Use same QoS as controller
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.create_subscription(LaserScan, '/scan_fixed', self.scan_callback, qos_profile)
        self.get_logger().info("Listening to laser scan...")
        
    def scan_callback(self, msg):
        if len(msg.ranges) == 0:
            return
            
        # Analyze different sectors
        front_distances = []
        left_distances = []
        right_distances = []
        
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            if msg.range_min < distance < 10.0:
                if abs(angle) < 0.785:  # 45 degrees front
                    front_distances.append(distance)
                elif angle < -0.785:  # Left side
                    left_distances.append(distance)
                elif angle > 0.785:  # Right side
                    right_distances.append(distance)
                    
        if front_distances:
            min_front = min(front_distances)
            avg_front = np.mean(front_distances)
            
            min_left = min(left_distances) if left_distances else float('inf')
            min_right = min(right_distances) if right_distances else float('inf')
            
            self.get_logger().info(
                f"Front: min={min_front:.2f}m, avg={avg_front:.2f}m | "
                f"Left: {min_left:.2f}m | Right: {min_right:.2f}m"
            )
            
            if min_front < 0.8:
                self.get_logger().warn(f"⚠️ OBSTACLE DETECTED at {min_front:.2f}m!")
                
def main():
    rclpy.init()
    node = ObstacleTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
