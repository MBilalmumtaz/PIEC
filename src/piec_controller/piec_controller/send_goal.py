#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Send first goal after 5 seconds
        self.create_timer(5.0, self.send_goal)
        self.goal_count = 0
        
    def send_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        
        # Different goals for testing
        if self.goal_count == 0:
            # Goal 1: Straight ahead
            goal.pose.position.x = 3.0
            goal.pose.position.y = 0.0
            self.get_logger().info("Sending Goal 1: (3.0, 0.0)")
            
        elif self.goal_count == 1:
            # Goal 2: To the right
            goal.pose.position.x = 2.0
            goal.pose.position.y = 2.0
            self.get_logger().info("Sending Goal 2: (2.0, 2.0)")
            
        elif self.goal_count == 2:
            # Goal 3: Far point
            goal.pose.position.x = 5.0
            goal.pose.position.y = 5.0
            self.get_logger().info("Sending Goal 3: (5.0, 5.0)")
            
        else:
            # Stop after 3 goals
            self.get_logger().info("All goals sent. Stopping.")
            self.destroy_timer(self._timers[0])
            return
        
        goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        self.goal_count += 1

def main():
    rclpy.init()
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
