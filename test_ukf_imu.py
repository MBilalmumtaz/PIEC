#!/usr/bin/env python3
"""
Test UKF-IMU integration
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class UKFIMUTest(Node):
    def __init__(self):
        super().__init__('ukf_imu_test')
        
        self.ukf_odom_received = False
        self.start_time = time.time()
        self.timeout = 10.0
        
        self.create_subscription(Odometry, '/ukf/odom', self.ukf_callback, 10)
        
        self.get_logger().info("🔧 Testing UKF-IMU Integration")
        self.get_logger().info("Waiting for /ukf/odom topic for 10 seconds...")
        
        self.timer = self.create_timer(1.0, self.check_status)
    
    def ukf_callback(self, msg):
        self.ukf_odom_received = True
        self.get_logger().info(f"✅ UKF Odom received: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
    
    def check_status(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= self.timeout:
            if self.ukf_odom_received:
                self.get_logger().info("✅ SUCCESS: UKF is working with IMU!")
            else:
                self.get_logger().error("❌ FAILURE: UKF is NOT publishing /ukf/odom")
                self.get_logger().info("Possible issues:")
                self.get_logger().info("1. UKF node not receiving /imu topic")
                self.get_logger().info("2. UKF parameters incorrect")
                self.get_logger().info("3. UKF not initialized with odometry")
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().info(f"Testing... {int(elapsed)}/{int(self.timeout)}s")

def main():
    rclpy.init()
    node = UKFIMUTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
