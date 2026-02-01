#!/usr/bin/env python3
"""
Diagnostic script to check all frames and transformations
"""
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
import math

class FrameDiagnostic(Node):
    def __init__(self):
        super().__init__('frame_diagnostic')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(Odometry, '/odometry', self.odom_cb, 10)
        self.create_subscription(Odometry, '/ukf/odom', self.ukf_cb, 10)
        self.create_subscription(Path, '/piec/path', self.path_cb, 10)
        
        self.timer = self.create_timer(2.0, self.check_all_transforms)
        
        self.odom_position = None
        self.ukf_position = None
        self.path_start = None
        
    def odom_cb(self, msg):
        self.odom_position = msg.pose.pose.position
        self.get_logger().info(f"📡 Odom: ({self.odom_position.x:.3f}, {self.odom_position.y:.3f})")
        
    def ukf_cb(self, msg):
        self.ukf_position = msg.pose.pose.position
        self.get_logger().info(f"📍 UKF: ({self.ukf_position.x:.3f}, {self.ukf_position.y:.3f})")
        
    def path_cb(self, msg):
        if len(msg.poses) > 0:
            self.path_start = msg.poses[0].pose.position
            self.get_logger().info(f"🛣️ Path start: ({self.path_start.x:.3f}, {self.path_start.y:.3f})")
    
    def check_all_transforms(self):
        self.get_logger().info("\n" + "="*60)
        
        # Check odom → base_link transform
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.get_logger().info(f"✅ odom→base_link: "
                                  f"x={trans.transform.translation.x:.3f}, "
                                  f"y={trans.transform.translation.y:.3f}, "
                                  f"z={trans.transform.translation.z:.3f}")
        except Exception as e:
            self.get_logger().error(f"❌ odom→base_link: {e}")
        
        # Check if positions match
        if self.odom_position and self.ukf_position:
            dx = self.ukf_position.x - self.odom_position.x
            dy = self.ukf_position.y - self.odom_position.y
            dist = math.hypot(dx, dy)
            if dist > 0.1:
                self.get_logger().error(f"❌ Odom/UKF mismatch: {dist:.3f}m")
            else:
                self.get_logger().info(f"✅ Odom/UKF aligned: {dist:.3f}m")
        
        if self.ukf_position and self.path_start:
            dx = self.path_start.x - self.ukf_position.x
            dy = self.path_start.y - self.ukf_position.y
            dist = math.hypot(dx, dy)
            if dist > 0.2:
                self.get_logger().error(f"❌ Path/UKF mismatch: {dist:.3f}m")
            else:
                self.get_logger().info(f"✅ Path starts at robot: {dist:.3f}m")

def main():
    rclpy.init()
    node = FrameDiagnostic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
