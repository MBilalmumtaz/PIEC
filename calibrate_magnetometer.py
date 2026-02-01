#!/usr/bin/env python3
"""
Calibrate magnetometer for IMU - FIXED VERSION for Vector3 messages
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
import math

class MagnetometerCalibratorFixed(Node):
    def __init__(self):
        super().__init__('magnetometer_calibrator_fixed')
        
        self.mag_sub = self.create_subscription(
            Vector3, '/magnetic', self.mag_callback, 10
        )
        
        self.samples_x = []
        self.samples_y = []
        self.samples_z = []
        self.sample_count = 0
        self.max_samples = 500
        
        self.get_logger().info("🧲 Magnetometer Calibration (Fixed for Vector3)")
        self.get_logger().info("Slowly rotate robot in ALL directions for 30 seconds")
        self.get_logger().info("Tilt, roll, and yaw the robot in full circles")
        self.get_logger().info(f"Collecting {self.max_samples} samples...")
        
        self.timer = self.create_timer(0.1, self.check_completion)
    
    def mag_callback(self, msg):
        if self.sample_count < self.max_samples:
            self.samples_x.append(msg.x)
            self.samples_y.append(msg.y)
            self.samples_z.append(msg.z)
            self.sample_count += 1
            
            if self.sample_count % 50 == 0:
                self.get_logger().info(f"Collected {self.sample_count}/{self.max_samples} samples")
    
    def check_completion(self):
        if self.sample_count >= self.max_samples:
            self.calculate_calibration()
            self.timer.cancel()
    
    def calculate_calibration(self):
        x = np.array(self.samples_x)
        y = np.array(self.samples_y)
        z = np.array(self.samples_z)
        
        # Calculate statistics
        x_mean, x_std = np.mean(x), np.std(x)
        y_mean, y_std = np.mean(y), np.std(y)
        z_mean, z_std = np.mean(z), np.std(z)
        
        # Calculate min/max for hard iron calibration
        x_min, x_max = np.min(x), np.max(x)
        y_min, y_max = np.min(y), np.max(y)
        z_min, z_max = np.min(z), np.max(z)
        
        # Hard iron bias (offset)
        bias_x = (x_max + x_min) / 2.0
        bias_y = (y_max + y_min) / 2.0
        bias_z = (z_max + z_min) / 2.0
        
        # Scale factors
        avg_delta = ((x_max - x_min) + (y_max - y_min) + (z_max - z_min)) / 3.0
        scale_x = avg_delta / (x_max - x_min)
        scale_y = avg_delta / (y_max - y_min)
        scale_z = avg_delta / (z_max - z_min)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🧲 MAGNETOMETER CALIBRATION RESULTS (Vector3)")
        self.get_logger().info("="*60)
        
        self.get_logger().info(f"\n📊 Statistics (raw values):")
        self.get_logger().info(f"  X: mean={x_mean:.1f}, std={x_std:.1f}, range=[{x_min:.1f}, {x_max:.1f}]")
        self.get_logger().info(f"  Y: mean={y_mean:.1f}, std={y_std:.1f}, range=[{y_min:.1f}, {y_max:.1f}]")
        self.get_logger().info(f"  Z: mean={z_mean:.1f}, std={z_std:.1f}, range=[{z_min:.1f}, {z_max:.1f}]")
        
        self.get_logger().info(f"\n🔧 Hard Iron Bias (offset to subtract):")
        self.get_logger().info(f"  X: {bias_x:.1f}")
        self.get_logger().info(f"  Y: {bias_y:.1f}")
        self.get_logger().info(f"  Z: {bias_z:.1f}")
        
        self.get_logger().info(f"\n📐 Scale Factors (multiply after bias correction):")
        self.get_logger().info(f"  X: {scale_x:.6f}")
        self.get_logger().info(f"  Y: {scale_y:.6f}")
        self.get_logger().info(f"  Z: {scale_z:.6f}")
        
        # Calculate corrected values
        corrected_x = (x - bias_x) * scale_x
        corrected_y = (y - bias_y) * scale_y
        corrected_z = (z - bias_z) * scale_z
        
        strengths = np.sqrt(corrected_x**2 + corrected_y**2 + corrected_z**2)
        avg_strength = np.mean(strengths)
        std_strength = np.std(strengths)
        
        self.get_logger().info(f"\n🧭 Magnetic Field Strength (corrected):")
        self.get_logger().info(f"  Average: {avg_strength:.1f} μT")
        self.get_logger().info(f"  Std Dev: {std_strength:.1f} μT")
        self.get_logger().info(f"  Variation: {(std_strength/avg_strength*100):.1f}%")
        
        # Earth's magnetic field is typically 25-65 μT
        if 20 < avg_strength < 70:
            self.get_logger().info(f"  ✅ Good: Within Earth's magnetic field range (25-65 μT)")
        else:
            self.get_logger().info(f"  ⚠️  Warning: Outside typical Earth field range")
        
        # Save calibration
        with open('/tmp/magnetometer_calibration_vector3.yaml', 'w') as f:
            f.write("magnetometer_calibration:\n")
            f.write(f"  # Raw values are in μT (microtesla)\n")
            f.write(f"  mag_bias_x: {bias_x:.1f}\n")
            f.write(f"  mag_bias_y: {bias_y:.1f}\n")
            f.write(f"  mag_bias_z: {bias_z:.1f}\n")
            f.write(f"  mag_scale_x: {scale_x:.6f}\n")
            f.write(f"  mag_scale_y: {scale_y:.6f}\n")
            f.write(f"  mag_scale_z: {scale_z:.6f}\n")
            f.write(f"  # For imu_filter_madgwick (in Tesla, multiply by 1e-6)\n")
            f.write(f"  imu_filter_bias_x: {bias_x * 1e-6:.6f}\n")
            f.write(f"  imu_filter_bias_y: {bias_y * 1e-6:.6f}\n")
            f.write(f"  imu_filter_bias_z: {bias_z * 1e-6:.6f}\n")
        
        self.get_logger().info(f"\n✅ Calibration saved to /tmp/magnetometer_calibration_vector3.yaml")
        self.get_logger().info(f"\n🎯 Update your IMU filter parameters:")
        self.get_logger().info(f"  mag_bias_x: {bias_x * 1e-6:.6f}")
        self.get_logger().info(f"  mag_bias_y: {bias_y * 1e-6:.6f}")
        self.get_logger().info(f"  mag_bias_z: {bias_z * 1e-6:.6f}")
        
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = MagnetometerCalibratorFixed()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
