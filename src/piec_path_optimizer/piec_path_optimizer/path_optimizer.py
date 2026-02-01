#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
import math

class PathOptimizer(Node):
    def __init__(self):
        super().__init__('path_optimizer')

        self.odom = None
        self.goal = None

        self.create_subscription(Odometry, '/ukf/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        
        # Reliable QoS for path publication
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        
        self.path_pub = self.create_publisher(Path, '/piec/path', qos_profile)
        self.create_timer(3.0, self.generate_simple_path)

    def odom_cb(self, msg):
        self.odom = msg

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def generate_simple_path(self):
        if self.odom is None or self.goal is None:
            self.get_logger().warn("Waiting for odometry and goal...")
            return

        start = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
        goal = self.goal

        self.get_logger().info(f"Generating simple path from {start} to {goal}")

        # Create a simple straight line path
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        # Generate waypoints along a straight line
        num_waypoints = 20
        for i in range(num_waypoints + 1):
            t = i / float(num_waypoints)
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.yaw_to_quaternion(0.0)
            path.poses.append(pose)

        # Publish multiple times for reliability
        for i in range(3):
            self.path_pub.publish(path)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        
        self.get_logger().info(f"Published simple path with {len(path.poses)} waypoints")

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main():
    rclpy.init()
    node = PathOptimizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
