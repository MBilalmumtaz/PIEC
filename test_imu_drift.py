#!/usr/bin/env python3
"""
Test IMU drift after magnetometer calibration
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import numpy as np

class IMUDriftTestFixed(Node):
    def __init__(self):
        super().__init__('imu_drift_test_fixed')
        
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.yaw_samples = []
        self.start_time = time.time()
        self.test_duration = 30.0
        
        self.get_logger().info("🧪 Testing IMU Drift (with magnetometer)")
        self.get_logger().info(f"Keep robot stationary for {self.test_duration} seconds")
        self.get_logger().info("Checking for yaw drift...")
        
        self.timer = self.create_timer(1.0, self.check_progress)
    
    def quat_to_yaw(self, q):
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 0.0001:
            return 0.0
        
        x = q.x / norm
        y = q.y / norm
        z = q.z / norm
        w = q.w / norm
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def imu_callback(self, msg):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed < self.test_duration:
            yaw = self.quat_to_yaw(msg.orientation)
            self.yaw_samples.append((current_time, yaw))
    
    def check_progress(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= self.test_duration:
            self.analyze_results()
            self.destroy_node()
            rclpy.shutdown()
        
        # Progress update
        remaining = self.test_duration - elapsed
        if remaining > 0:
            self.get_logger().info(f"Testing... {int(elapsed)}s elapsed, {int(remaining)}s remaining")
    
    def analyze_results(self):
        if len(self.yaw_samples) < 10:
            self.get_logger().error("Not enough IMU samples!")
            return
        
        times = [t for t, _ in self.yaw_samples]
        yaws = [y for _, y in self.yaw_samples]
        
        # Convert to degrees for easier interpretation
        yaws_deg = [math.degrees(y) for y in yaws]
        
        start_yaw = yaws_deg[0]
        end_yaw = yaws_deg[-1]
        total_drift = end_yaw - start_yaw
        total_time = times[-1] - times[0]
        drift_rate = total_drift / total_time if total_time > 0 else 0
        
        # Calculate standard deviation
        yaw_std = np.std(yaws_deg)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🧪 IMU DRIFT TEST RESULTS (with magnetometer)")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Test duration: {total_time:.1f} seconds")
        self.get_logger().info(f"Samples collected: {len(self.yaw_samples)}")
        self.get_logger().info(f"Start yaw: {start_yaw:.2f}°")
        self.get_logger().info(f"End yaw: {end_yaw:.2f}°")
        self.get_logger().info(f"Total drift: {total_drift:.2f}°")
        self.get_logger().info(f"Drift rate: {drift_rate:.4f} °/s")
        self.get_logger().info(f"Yaw stability (std dev): {yaw_std:.3f}°")
        
        # Evaluation
        if abs(drift_rate) < 0.01:  # < 0.57 °/s
            self.get_logger().info("✅ EXCELLENT: Magnetometer is working! Minimal drift.")
        elif abs(drift_rate) < 0.1:  # < 5.7 °/s
            self.get_logger().info("✅ GOOD: Acceptable drift with magnetometer.")
        elif abs(drift_rate) < 1.0:  # < 57 °/s
            self.get_logger().info("⚠️  ACCEPTABLE: Some drift, better than before.")
        else:
            self.get_logger().info("❌ POOR: Drift still significant. Check magnetometer calibration.")
        
        # Log first and last 5 samples for verification
        self.get_logger().info(f"\nFirst 5 yaw samples (degrees):")
        for i in range(min(5, len(yaws_deg))):
            self.get_logger().info(f"  {i+1}: {yaws_deg[i]:.2f}°")
        
        self.get_logger().info(f"\nLast 5 yaw samples (degrees):")
        for i in range(max(0, len(yaws_deg)-5), len(yaws_deg)):
            self.get_logger().info(f"  {i+1}: {yaws_deg[i]:.2f}°")

def main():
    rclpy.init()
    node = IMUDriftTestFixed()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
