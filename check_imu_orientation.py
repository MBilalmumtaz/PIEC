#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUOrientationChecker(Node):
    def __init__(self):
        super().__init__('imu_orientation_checker')
        
        # Subscribe to OpenZen raw and filtered IMU
        self.create_subscription(Imu, '/openzen/data', self.openzen_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.openzen_orientation = None
        self.imu_orientation = None
        self.sample_count = 0
        
        self.timer = self.create_timer(1.0, self.print_orientation)
        
        self.get_logger().info("🧭 IMU Orientation Checker (OpenZen)")
        self.get_logger().info("Monitoring /openzen/data (raw) and /imu (filtered)")
    
    def quat_to_euler(self, q):
        """Convert quaternion to roll, pitch, yaw in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def openzen_callback(self, msg):
        self.openzen_orientation = msg.orientation
    
    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation
    
    def print_orientation(self):
        self.sample_count += 1
        
        if self.openzen_orientation and self.imu_orientation:
            raw_roll, raw_pitch, raw_yaw = self.quat_to_euler(self.openzen_orientation)
            filt_roll, filt_pitch, filt_yaw = self.quat_to_euler(self.imu_orientation)
            
            self.get_logger().info(f"\n{'='*70}")
            self.get_logger().info(f"Sample #{self.sample_count}")
            self.get_logger().info(f"  RAW  (/openzen/data): Roll={raw_roll:7.2f}°  Pitch={raw_pitch:7.2f}°  Yaw={raw_yaw:7.2f}°")
            self.get_logger().info(f"  FILT (/imu):          Roll={filt_roll:7.2f}°  Pitch={filt_pitch:7.2f}°  Yaw={filt_yaw:7.2f}°")
        elif self.openzen_orientation:
            self.get_logger().warn("⚠️  Only raw data available")
        elif self.imu_orientation:
            self.get_logger().warn("⚠️  Only filtered data available")
        else:
            self.get_logger().warn("❌ No IMU data received yet")

def main():
    rclpy.init()
    node = IMUOrientationChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
