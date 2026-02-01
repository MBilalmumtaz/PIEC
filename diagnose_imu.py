#!/usr/bin/env python3
"""
Diagnose IMU data flow issues
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
import time

class IMUDiagnostic(Node):
    def __init__(self):
        super().__init__('imu_diagnostic')
        
        # Track received messages
        self.imu_raw_received = False
        self.imu_received = False
        self.magnetic_received = False
        self.magnetic_field_received = False
        
        self.start_time = time.time()
        self.timeout = 5.0
        
        # Subscribe to all IMU-related topics
        self.create_subscription(Imu, '/imu_raw', self.imu_raw_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Vector3, '/magnetic', self.magnetic_callback, 10)
        self.create_subscription(MagneticField, '/magnetic_field', self.magnetic_field_callback, 10)
        
        self.get_logger().info("🔧 IMU Data Flow Diagnostic")
        self.get_logger().info("Checking all IMU topics for 5 seconds...")
        
        self.timer = self.create_timer(1.0, self.check_status)
    
    def imu_raw_callback(self, msg):
        self.imu_raw_received = True
    
    def imu_callback(self, msg):
        self.imu_received = True
    
    def magnetic_callback(self, msg):
        self.magnetic_received = True
    
    def magnetic_field_callback(self, msg):
        self.magnetic_field_received = True
    
    def check_status(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= self.timeout:
            self.report_results()
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().info(f"Diagnosing... {int(elapsed)}/{int(self.timeout)}s")
    
    def report_results(self):
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🔧 IMU DATA FLOW DIAGNOSTIC RESULTS")
        self.get_logger().info("="*60)
        
        self.get_logger().info(f"✅ /imu_raw: {'RECEIVED' if self.imu_raw_received else 'NOT RECEIVED'}")
        self.get_logger().info(f"✅ /imu: {'RECEIVED' if self.imu_received else 'NOT RECEIVED'}")
        self.get_logger().info(f"✅ /magnetic: {'RECEIVED' if self.magnetic_received else 'NOT RECEIVED'}")
        self.get_logger().info(f"✅ /magnetic_field: {'RECEIVED' if self.magnetic_field_received else 'NOT RECEIVED'}")
        
        # Analysis
        if not self.imu_raw_received:
            self.get_logger().error("❌ IMU driver not publishing /imu_raw")
            self.get_logger().info("  Fix: Check fdilink_imu_driver node and /dev/ttyUSB0 permissions")
        
        if self.imu_raw_received and not self.imu_received:
            self.get_logger().error("❌ IMU filter not processing /imu_raw")
            self.get_logger().info("  Fix: Check imu_filter node configuration")
        
        if not self.magnetic_received:
            self.get_logger().error("❌ IMU driver not publishing /magnetic")
            self.get_logger().info("  Fix: Check fdilink_imu_driver magnetometer configuration")
        
        if self.magnetic_received and not self.magnetic_field_received:
            self.get_logger().error("❌ Magnetometer converter not working")
            self.get_logger().info("  Fix: Check magnetometer_converter node")
        
        if all([self.imu_raw_received, self.imu_received, 
                self.magnetic_received, self.magnetic_field_received]):
            self.get_logger().info("✅ ALL SYSTEMS GO! IMU data flow is working correctly.")
        else:
            self.get_logger().error("❌ IMU SYSTEM HAS ISSUES - See fixes above")

def main():
    rclpy.init()
    node = IMUDiagnostic()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
