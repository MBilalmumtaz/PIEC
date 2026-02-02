#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import time

class IMUDiagnostic(Node):
    def __init__(self):
        super().__init__('imu_diagnostic')
        
        # Track received messages for OpenZen IMU
        self.openzen_data_received = False
        self.openzen_mag_received = False
        self.imu_filtered_received = False
        
        self.start_time = time.time()
        self.timeout = 5.0
        
        # Subscribe to OpenZen topics
        self.create_subscription(Imu, '/openzen/data', self.openzen_data_callback, 10)
        self.create_subscription(MagneticField, '/openzen/mag', self.openzen_mag_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.get_logger().info("🔧 IMU Data Flow Diagnostic (OpenZen)")
        self.get_logger().info("Checking OpenZen IMU topics for 5 seconds...")
        
        self.timer = self.create_timer(1.0, self.check_status)
    
    def openzen_data_callback(self, msg):
        if not self.openzen_data_received:
            self.get_logger().info("✅ Receiving /openzen/data")
            self.get_logger().info(f"   Linear accel: x={msg.linear_acceleration.x:.3f}, y={msg.linear_acceleration.y:.3f}, z={msg.linear_acceleration.z:.3f}")
        self.openzen_data_received = True
    
    def openzen_mag_callback(self, msg):
        if not self.openzen_mag_received:
            self.get_logger().info("✅ Receiving /openzen/mag")
            self.get_logger().info(f"   Magnetic field: x={msg.magnetic_field.x:.6f}, y={msg.magnetic_field.y:.6f}, z={msg.magnetic_field.z:.6f}")
        self.openzen_mag_received = True
    
    def imu_callback(self, msg):
        if not self.imu_filtered_received:
            self.get_logger().info("✅ Receiving /imu (filtered)")
        self.imu_filtered_received = True
    
    def check_status(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= self.timeout:
            self.get_logger().info("\n" + "="*50)
            self.get_logger().info("DIAGNOSTIC SUMMARY:")
            self.get_logger().info(f"  /openzen/data: {'✅ OK' if self.openzen_data_received else '❌ NOT RECEIVED'}")
            self.get_logger().info(f"  /openzen/mag: {'✅ OK' if self.openzen_mag_received else '❌ NOT RECEIVED'}")
            self.get_logger().info(f"  /imu (filtered): {'✅ OK' if self.imu_filtered_received else '❌ NOT RECEIVED'}")
            
            if not self.openzen_data_received:
                self.get_logger().error("\n❌ OpenZen IMU data not received!")
                self.get_logger().info("Troubleshooting:")
                self.get_logger().info("1. Check permissions: sudo chmod 666 /dev/ttyUSB0")
                self.get_logger().info("2. Check if driver is running: ros2 node list | grep openzen")
                self.get_logger().info("3. Check device: ls -la /dev/ttyUSB*")
                self.get_logger().info("4. Verify baud rate: OpenZen uses 921600 by default (set baudrate=0 in launch)")
                self.get_logger().info("5. Add user to dialout group: sudo usermod -a -G dialout $USER")
            
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = IMUDiagnostic()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
