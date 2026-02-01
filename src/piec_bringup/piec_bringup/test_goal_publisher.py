#!/usr/bin/env python3
# test_goal_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class TestGoalPublisher(Node):
    def __init__(self):
        super().__init__('test_goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(2.0, self.publish_goal)
        self.goal_counter = 0
        self.goals = [
            (2.0, 0.0),   # Forward
            (2.0, 2.0),   # Diagonal
            (0.0, 2.0),   # Left
            (0.0, 0.0),   # Back home
        ]
        self.get_logger().info("Test goal publisher started")
        
    def publish_goal(self):
        if self.goal_counter >= len(self.goals):
            self.get_logger().info("All goals published. Stopping.")
            self.timer.cancel()
            return
            
        x, y = self.goals[self.goal_counter]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.pub.publish(msg)
        self.get_logger().info(f"Published goal {self.goal_counter+1}: x={x:.1f}, y={y:.1f}")
        self.goal_counter += 1

def main():
    rclpy.init()
    node = TestGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
