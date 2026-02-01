#!/usr/bin/env python3
"""
Quick test magnetometer data
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math

class MagnetometerTester(Node):
    def __init__(self):
        super().__init__('magnetometer_tester')
        
        self.mag_sub = self.create_subscription(
            Vector3, '/magnetic', self.mag_callback, 10
        )
        
        self.sample_count = 0
        self.max_samples = 10
        
        self.get_logger().info("🧭 Testing magnetometer...")
        self.get_logger().info("Rotate robot slowly and check values")
    
    def mag_callback(self, msg):
        if self.sample_count < self.max_samples:
            magnitude = math.sqrt(msg.x**2 + msg.y**2 + msg.z**2)
            
            self.get_logger().info(f"Sample {self.sample_count+1}:")
            self.get_logger().info(f"  X: {msg.x:7.1f} μT")
            self.get_logger().info(f"  Y: {msg.y:7.1f} μT")
            self.get_logger().info(f"  Z: {msg.z:7.1f} μT")
            self.get_logger().info(f"  Magnitude: {magnitude:7.1f} μT")
            self.get_logger().info("")
            
            self.sample_count += 1
            
            if self.sample_count == self.max_samples:
                self.get_logger().info("✅ Test complete")
                self.destroy_node()
                rclpy.shutdown()

def main():
    rclpy.init()
    node = MagnetometerTester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
