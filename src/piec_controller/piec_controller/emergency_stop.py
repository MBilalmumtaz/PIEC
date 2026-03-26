#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
import numpy as np

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        
        # Parameters
        self.declare_parameter('stop_distance', 0.90)   # well above LiDAR range_min of 0.6 m
        self.declare_parameter('slow_distance', 1.20)   # wider slow zone
        self.declare_parameter('enable_emergency_stop', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_piec')
        self.declare_parameter('output_topic', '/cmd_vel')
        
        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # State
        self.emergency_active = False
        self.last_cmd = Twist()
        self.min_obstacle_distance = 10.0
        self.obstacle_persist_count = 0
        
        # Use BEST_EFFORT QoS to match the publisher
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        
        # Timer to ensure continuous publishing
        self.create_timer(0.05, self.publish_safe_cmd)  # 20Hz
        
        self.get_logger().info(f"🛑 Emergency Stop ready:")
        self.get_logger().info(f"  Input: {self.cmd_vel_topic}")
        self.get_logger().info(f"  Output: {self.output_topic}")
        self.get_logger().info(f"  Scan: {self.scan_topic} (BEST_EFFORT)")
        self.get_logger().info(f"  Stop distance: {self.stop_distance}m")
    
    def cmd_callback(self, msg):
        self.last_cmd.linear.x = msg.linear.x
        self.last_cmd.linear.y = msg.linear.y
        self.last_cmd.linear.z = msg.linear.z
        self.last_cmd.angular.x = msg.angular.x
        self.last_cmd.angular.y = msg.angular.y
        self.last_cmd.angular.z = msg.angular.z
    
    def scan_callback(self, msg):
        if not self.get_parameter('enable_emergency_stop').value:
            self.min_obstacle_distance = 10.0
            return
            
        ranges = np.array(msg.ranges)
        
        # Front sector (-45 to +45 degrees)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        front_indices = []
        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            if -0.785 <= angle <= 0.785:
                front_indices.append(i)

        if not front_indices:
            self.min_obstacle_distance = 10.0
            self.obstacle_persist_count = 0
            return

        front_ranges = ranges[front_indices]

        # Collect valid readings; treat readings at or below range_min as very close
        valid_ranges = []
        for r in front_ranges:
            if np.isfinite(r) and r > msg.range_min and r < msg.range_max:
                valid_ranges.append(r)

        if not valid_ranges:
            # No valid readings in forward sector – sensor blind-spot or all readings
            # below range_min; treat conservatively as an obstacle very close.
            self.min_obstacle_distance = msg.range_min if msg.range_min > 0.0 else 0.5
        else:
            self.min_obstacle_distance = float(np.min(valid_ranges))
        
        # Track persistence
        if self.min_obstacle_distance < self.stop_distance:
            self.obstacle_persist_count += 1
        else:
            self.obstacle_persist_count = max(0, self.obstacle_persist_count - 1)
        
        # Adaptive threshold
        adaptive_stop_distance = self.stop_distance
        if self.obstacle_persist_count > 10:
            adaptive_stop_distance = max(0.40, self.stop_distance * 0.67)
        
        was_emergency = self.emergency_active
        if self.min_obstacle_distance < adaptive_stop_distance:
            self.emergency_active = True
            if not was_emergency:
                self.get_logger().error(f"🚨 EMERGENCY STOP! Obstacle at {self.min_obstacle_distance:.2f}m")
        else:
            if self.emergency_active:
                self.emergency_active = False
                self.get_logger().info(f"✅ Emergency cleared - obstacle now at {self.min_obstacle_distance:.2f}m")
    
    def publish_safe_cmd(self):
        safe_cmd = Twist()
        if self.emergency_active:
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
        elif self.min_obstacle_distance < self.slow_distance:
            safe_cmd.linear.x = self.last_cmd.linear.x * 0.5
            safe_cmd.angular.z = self.last_cmd.angular.z
        else:
            safe_cmd.linear.x = self.last_cmd.linear.x
            safe_cmd.linear.y = self.last_cmd.linear.y
            safe_cmd.linear.z = self.last_cmd.linear.z
            safe_cmd.angular.x = self.last_cmd.angular.x
            safe_cmd.angular.y = self.last_cmd.angular.y
            safe_cmd.angular.z = self.last_cmd.angular.z
        
        self.cmd_pub.publish(safe_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Emergency stop node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
