#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestController(Node):
    def __init__(self):
        super().__init__('test_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Testing controller...")
        
        # Test sequence
        self.create_timer(2.0, self.test_forward)
        self.create_timer(4.0, self.test_turn)
        self.create_timer(6.0, self.test_stop)
        self.create_timer(8.0, self.shutdown)
        
    def test_forward(self):
        msg = Twist()
        msg.linear.x = 0.3
        self.publisher.publish(msg)
        self.get_logger().info("Moving forward...")
        
    def test_turn(self):
        msg = Twist()
        msg.angular.z = 0.5
        self.publisher.publish(msg)
        self.get_logger().info("Turning...")
        
    def test_stop(self):
        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info("Stopping...")
        
    def shutdown(self):
        self.get_logger().info("Test complete")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = TestController()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
