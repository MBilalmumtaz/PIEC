#!/usr/bin/env python3
# manual_goal_setter.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class ManualGoalSetter(Node):
    def __init__(self):
        super().__init__('manual_goal_setter')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Get current robot pose first
        self.current_x = 0.0
        self.current_y = 0.0
        self.create_subscription(
            PoseStamped,
            '/ukf/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info("Manual Goal Setter Ready!")
        self.get_logger().info("Enter goals in format: x y")
        self.set_goals()
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
    def set_goals(self):
        import sys
        import select
        
        while rclpy.ok():
            print(f"\nCurrent robot position: ({self.current_x:.2f}, {self.current_y:.2f})")
            print("Enter goal (x y) or 'q' to quit: ", end='', flush=True)
            
            # Non-blocking input
            if select.select([sys.stdin], [], [], 1)[0]:
                line = sys.stdin.readline().strip()
                if line.lower() == 'q':
                    break
                    
                try:
                    x_str, y_str = line.split()
                    x = float(x_str)
                    y = float(y_str)
                    
                    # Publish goal
                    goal = PoseStamped()
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.header.frame_id = 'odom'  # Use odom frame!
                    goal.pose.position.x = x
                    goal.pose.position.y = y
                    goal.pose.orientation.w = 1.0
                    
                    self.pub.publish(goal)
                    self.get_logger().info(f"Published goal: ({x:.2f}, {y:.2f})")
                    
                except:
                    print("Invalid input! Use format: x y")
                    
            rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    node = ManualGoalSetter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
