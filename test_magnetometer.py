#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import math

class TestMagnetometer(Node):
    def __init__(self):
        super().__init__('test_magnetometer')
        
        self.mag_sub = self.create_subscription(
            MagneticField,
            '/openzen/mag',
            self.mag_callback,
            10
        )
        
        self.sample_count = 0
        self.get_logger().info("🧲 Testing LPMS IG1 Magnetometer")
        self.get_logger().info("Listening to /openzen/mag...")
    
    def mag_callback(self, msg):
        self.sample_count += 1
        
        # Calculate magnitude
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y
        mag_z = msg.magnetic_field.z
        magnitude = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2)
        
        # Earth's magnetic field is typically 25-65 microTesla
        if self.sample_count % 10 == 0:  # Print every 10th sample
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"Sample #{self.sample_count}")
            self.get_logger().info(f"  X: {mag_x:12.9f} T  ({mag_x*1e6:8.3f} µT)")
            self.get_logger().info(f"  Y: {mag_y:12.9f} T  ({mag_y*1e6:8.3f} µT)")
            self.get_logger().info(f"  Z: {mag_z:12.9f} T  ({mag_z*1e6:8.3f} µT)")
            self.get_logger().info(f"  Magnitude: {magnitude*1e6:.3f} µT")
            
            # Check if values are reasonable
            mag_uT = magnitude * 1e6
            if 20.0 < mag_uT < 70.0:
                self.get_logger().info("  ✅ Magnitude is within Earth's field range")
            else:
                self.get_logger().warn(f"  ⚠️  Magnitude outside typical range (25-65 µT)")

def main():
    rclpy.init()
    node = MagnetometerTester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
