#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class TestMagIMU(Node):
    def __init__(self):
        super().__init__('test_mag_imu')
        
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.start_time = time.time()
        self.last_yaw = None
        self.yaw_samples = []
        self.test_duration = 30.0
        
        self.get_logger().info("🧪 Testing LPMS IG1 IMU with Magnetometer")
        self.get_logger().info(f"Keep robot stationary for {self.test_duration} seconds")
        self.get_logger().info("Checking for yaw drift...")
        
        self.timer = self.create_timer(1.0, self.check_drift)
    
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
            
            if self.last_yaw is None:
                self.last_yaw = yaw
            else:
                drift = yaw - self.last_yaw
                self.last_yaw = yaw
    
    def check_drift(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= self.test_duration:
            self.analyze_drift()
            self.destroy_node()
            rclpy.shutdown()
    
    def analyze_drift(self):
        if len(self.yaw_samples) < 2:
            self.get_logger().error("❌ Not enough samples collected!")
            return
        
        start_time, start_yaw = self.yaw_samples[0]
        end_time, end_yaw = self.yaw_samples[-1]
        
        duration = end_time - start_time
        total_drift = end_yaw - start_yaw
        drift_rate = total_drift / duration if duration > 0 else 0
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("LPMS IG1 IMU DRIFT TEST RESULTS:")
        self.get_logger().info(f"Duration: {duration:.1f}s")
        self.get_logger().info(f"Samples: {len(self.yaw_samples)}")
        self.get_logger().info(f"Start yaw: {math.degrees(start_yaw):.2f}°")
        self.get_logger().info(f"End yaw: {math.degrees(end_yaw):.2f}°")
        self.get_logger().info(f"Total drift: {math.degrees(total_drift):.2f}°")
        self.get_logger().info(f"Drift rate: {math.degrees(drift_rate):.4f} °/s")
        
        if abs(drift_rate) < 0.001:  # < 0.057 °/s
            self.get_logger().info("✅ EXCELLENT: Magnetometer is working! No significant drift.")
        elif abs(drift_rate) < 0.01:  # < 0.57 °/s
            self.get_logger().info("✅ GOOD: Minimal drift with magnetometer.")
        elif abs(drift_rate) < 0.1:  # < 5.7 °/s
            self.get_logger().info("⚠️  ACCEPTABLE: Some drift, but much better than gyro-only.")
        else:
            self.get_logger().error("❌ HIGH DRIFT: Check magnetometer calibration!")

def main():
    rclpy.init()
    node = TestMagIMU()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
