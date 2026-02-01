#!/usr/bin/env python3
"""
Convert geometry_msgs/Vector3 to sensor_msgs/MagneticField
for IMU magnetometer compatibility
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

class MagnetometerConverter(Node):
    def __init__(self):
        super().__init__('magnetometer_converter')
        
        # Parameters
        self.declare_parameter('scale_factor', 1.0e-6)  # Convert to Tesla (assuming μT)
        self.declare_parameter('input_topic', '/magnetic')
        self.declare_parameter('output_topic', '/magnetic_field')
        self.declare_parameter('frame_id', 'imu_link')
        
        scale = self.get_parameter('scale_factor').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        frame_id = self.get_parameter('frame_id').value
        
        self.scale_factor = scale
        self.frame_id = frame_id
        
        # Subscribe to raw magnetic field (Vector3)
        self.subscription = self.create_subscription(
            Vector3,
            input_topic,
            self.callback,
            10
        )
        
        # Publish converted magnetic field (MagneticField)
        self.publisher = self.create_publisher(
            MagneticField,
            output_topic,
            10
        )
        
        self.get_logger().info(f"🔧 Magnetometer Converter started")
        self.get_logger().info(f"  Input: {input_topic} (Vector3)")
        self.get_logger().info(f"  Output: {output_topic} (MagneticField)")
        self.get_logger().info(f"  Scale factor: {scale}")
        self.get_logger().info(f"  Frame ID: {frame_id}")
    
    def callback(self, msg):
        # Create MagneticField message
        mag_msg = MagneticField()
        mag_msg.header = Header()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.frame_id
        
        # Apply scale factor (convert to Tesla)
        # Raw values seem to be in microtesla (μT) based on typical Earth field ~50μT
        mag_msg.magnetic_field.x = msg.x * self.scale_factor
        mag_msg.magnetic_field.y = msg.y * self.scale_factor
        mag_msg.magnetic_field.z = msg.z * self.scale_factor
        
        # Covariance (assuming good quality sensor)
        # 1.0e-6 Tesla^2 variance (1 μT uncertainty)
        cov = 1.0e-6
        mag_msg.magnetic_field_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov
        ]
        
        self.publisher.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
