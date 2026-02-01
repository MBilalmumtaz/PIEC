#!/usr/bin/env python3
"""
Diagnose UKF issues
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time

class UKFDiagnostic(Node):
    def __init__(self):
        super().__init__('ukf_diagnostic')
        
        self.odom_received = False
        self.imu_received = False
        self.ukf_odom_received = False
        self.start_time = time.time()
        
        # Subscribe to all relevant topics
        self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/ukf/odom', self.ukf_callback, 10)
        
        self.get_logger().info("🔧 UKF Diagnostic Started")
        self.get_logger().info("Checking topics for 10 seconds...")
        
        self.timer = self.create_timer(1.0, self.check_status)
        self.timer2 = self.create_timer(0.1, self.log_status)
    
    def odom_callback(self, msg):
        self.odom_received = True
    
    def imu_callback(self, msg):
        self.imu_received = True
    
    def ukf_callback(self, msg):
        self.ukf_odom_received = True
    
    def log_status(self):
        elapsed = time.time() - self.start_time
        if elapsed < 10:
            status = f"Status [t={int(elapsed)}s]: "
            status += f"Odom: {'✓' if self.odom_received else '✗'}, "
            status += f"IMU: {'✓' if self.imu_received else '✗'}, "
            status += f"UKF: {'✓' if self.ukf_odom_received else '✗'}"
            self.get_logger().info(status)
    
    def check_status(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= 10:
            self.report_results()
            self.destroy_node()
            rclpy.shutdown()
    
    def report_results(self):
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🔧 UKF DIAGNOSTIC RESULTS")
        self.get_logger().info("="*60)
        
        self.get_logger().info(f"✅ /odometry received: {self.odom_received}")
        self.get_logger().info(f"✅ /imu received: {self.imu_received}")
        self.get_logger().info(f"✅ /ukf/odom published: {self.ukf_odom_received}")
        
        if not self.odom_received:
            self.get_logger().error("❌ UKF IS MISSING ODOMETRY DATA")
            self.get_logger().info("  Fix: Check if scout_base_node is running")
            self.get_logger().info("  Command: ros2 launch agilex_scout scout_robot_lidar.launch.py use_imu:=true use_base:=true")
        
        if not self.imu_received:
            self.get_logger().error("❌ UKF IS MISSING IMU DATA")
            self.get_logger().info("  Fix: IMU filter is not publishing /imu")
        
        if self.odom_received and self.imu_received and not self.ukf_odom_received:
            self.get_logger().error("❌ UKF HAS DATA BUT NOT PUBLISHING")
            self.get_logger().info("  Fix: Check UKF node parameters and initialization")
            self.get_logger().info("  Possible issues:")
            self.get_logger().info("  1. UKF waiting for more initialization data")
            self.get_logger().info("  2. UKF parameters incorrect")
            self.get_logger().info("  3. UKF debug mode might be hiding output")
        
        if all([self.odom_received, self.imu_received, self.ukf_odom_received]):
            self.get_logger().info("✅ ALL SYSTEMS GO! UKF is working correctly.")

def main():
    rclpy.init()
    node = UKFDiagnostic()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
