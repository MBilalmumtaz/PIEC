#!/usr/bin/env python3
"""
MPC Trajectory Visualizer for Debugging
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import numpy as np

class MPCVisualizer(Node):
    def __init__(self):
        super().__init__('mpc_visualizer')
        
        # Publisher for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/mpc_debug/trajectory', 10)
        self.path_pub = self.create_publisher(Path, '/mpc_debug/path', 10)
        
        # Timer for periodic visualization
        self.create_timer(0.5, self.visualize_mpc)
        
        self.get_logger().info("MPC Visualizer started")
    
    def visualize_mpc(self):
        """Visualize MPC predictions"""
        # This would typically subscribe to MPC prediction topics
        # For now, create a placeholder visualization
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Predicted trajectory (green)
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0
        marker.color.g = 1.0  # Green
        
        # Add points (example - would come from actual MPC)
        for i in range(10):
            point = Point()
            point.x = i * 0.1
            point.y = 0.0
            point.z = 0.0
            marker.points.append(point)
        
        marker_array.markers.append(marker)
        
        # Control actions (red arrows)
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_controls"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Shaft diameter
        marker.scale.y = 0.2  # Head diameter
        marker.scale.z = 0.1  # Head length
        marker.color.a = 1.0
        marker.color.r = 1.0  # Red
        
        marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = MPCVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
