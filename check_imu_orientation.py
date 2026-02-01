#!/usr/bin/env python3
"""
Check IMU orientation and calculate correct transform
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np

class IMUOrientationChecker(Node):
    def __init__(self):
        super().__init__('imu_orientation_checker')
        
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.imu_raw_sub = self.create_subscription(Imu, '/imu_raw', self.imu_raw_callback, 10)
        
        self.imu_orientation = None
        self.imu_raw_orientation = None
        self.sample_count = 0
        
        self.timer = self.create_timer(1.0, self.print_orientation)
        
    def quat_to_rpy(self, q):
        """Convert quaternion to roll, pitch, yaw"""
        # Normalize
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 0.0001:
            return 0.0, 0.0, 0.0
        
        x = q.x / norm
        y = q.y / norm
        z = q.z / norm
        w = q.w / norm
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation
        self.sample_count += 1
    
    def imu_raw_callback(self, msg):
        self.imu_raw_orientation = msg.orientation
    
    def print_orientation(self):
        if self.imu_orientation is not None:
            roll, pitch, yaw = self.quat_to_rpy(self.imu_orientation)
            
            self.get_logger().info("="*60)
            self.get_logger().info(f"📊 IMU Orientation (Sample #{self.sample_count})")
            self.get_logger().info(f"  Quaternion: x={self.imu_orientation.x:.4f}, y={self.imu_orientation.y:.4f}, "
                                  f"z={self.imu_orientation.z:.4f}, w={self.imu_orientation.w:.4f}")
            self.get_logger().info(f"  Roll:  {math.degrees(roll):7.1f}°")
            self.get_logger().info(f"  Pitch: {math.degrees(pitch):7.1f}°")
            self.get_logger().info(f"  Yaw:   {math.degrees(yaw):7.1f}°")
            
            # Expected: When robot faces forward (X-axis forward), yaw should be 0°
            # If yaw is 180°, IMU is backwards
            # If yaw is 90°, IMU is rotated 90° clockwise
            # If yaw is -90°, IMU is rotated 90° counter-clockwise
            
            if abs(yaw - 0.0) < 0.1:  # ±5.7°
                self.get_logger().info("✅ IMU orientation is CORRECT (facing forward)")
            elif abs(yaw - math.pi) < 0.1:  # 180° ±5.7°
                self.get_logger().info("❌ IMU is BACKWARDS (rotated 180°)")
                self.get_logger().info("   Fix: Rotate 180° around Z axis")
            elif abs(yaw - math.pi/2) < 0.1:  # 90° ±5.7°
                self.get_logger().info("❌ IMU is SIDEWAYS (rotated 90° clockwise)")
                self.get_logger().info("   Fix: Rotate -90° around Z axis")
            elif abs(yaw + math.pi/2) < 0.1:  # -90° ±5.7°
                self.get_logger().info("❌ IMU is SIDEWAYS (rotated 90° counter-clockwise)")
                self.get_logger().info("   Fix: Rotate 90° around Z axis")
            else:
                self.get_logger().info(f"⚠️  IMU is rotated by {math.degrees(yaw):.1f}°")
            
            # Also check pitch and roll
            if abs(pitch) > 0.5:  # > 28.6°
                self.get_logger().warn(f"⚠️  IMU may be tilted: Pitch = {math.degrees(pitch):.1f}°")
            if abs(roll) > 0.5:  # > 28.6°
                self.get_logger().warn(f"⚠️  IMU may be tilted: Roll = {math.degrees(roll):.1f}°")

def main():
    rclpy.init()
    node = IMUOrientationChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
