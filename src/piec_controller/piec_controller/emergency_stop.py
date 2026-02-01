#!/usr/bin/env python3
"""
Emergency Stop Node - Pass-through with safety override
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        
        # Parameters
        self.declare_parameter('stop_distance', 0.3)  # Match launch file default
        self.declare_parameter('slow_distance', 0.6)  # Match launch file default
        self.declare_parameter('enable_emergency_stop', True)
        self.declare_parameter('scan_topic', '/scan_fixed')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_piec')
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        
        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # State
        self.emergency_active = False
        self.last_cmd = Twist()
        self.min_obstacle_distance = 10.0
        self.obstacle_persist_count = 0  # Track obstacle persistence for adaptive thresholds
        
        # Subscribers
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        
        # Publisher - CRITICAL: Always publish even if no obstacles
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        
        # Timer to ensure continuous publishing
        self.create_timer(0.05, self.publish_safe_cmd)  # 20Hz
        
        self.get_logger().info(f"🛑 Emergency Stop ready:")
        self.get_logger().info(f"  Input: {self.cmd_vel_topic}")
        self.get_logger().info(f"  Output: {self.output_topic}")
        self.get_logger().info(f"  Scan: {self.scan_topic}")
        self.get_logger().info(f"  Stop distance: {self.stop_distance}m")
    
    def cmd_callback(self, msg: Twist):
        """Store latest command (deep copy for thread safety)"""
        self.last_cmd.linear.x = msg.linear.x
        self.last_cmd.linear.y = msg.linear.y
        self.last_cmd.linear.z = msg.linear.z
        self.last_cmd.angular.x = msg.angular.x
        self.last_cmd.angular.y = msg.angular.y
        self.last_cmd.angular.z = msg.angular.z
    
    def scan_callback(self, msg: LaserScan):
        """Monitor laser scan for emergency conditions"""
        if not self.get_parameter('enable_emergency_stop').value:
            self.min_obstacle_distance = 10.0
            return
            
        ranges = np.array(msg.ranges)
        
        # Check front sector ONLY (-45 to +45 degrees) - Wider angle to better detect obstacles
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        front_indices = []
        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            if -0.785 <= angle <= 0.785:  # ±45 degrees
                if np.isfinite(ranges[i]) and msg.range_min < ranges[i] < msg.range_max:
                    front_indices.append(i)
        
        if not front_indices:
            self.min_obstacle_distance = 10.0
            self.obstacle_persist_count = 0
            return
            
        front_ranges = ranges[front_indices]
        self.min_obstacle_distance = np.min(front_ranges)
        
        # Track obstacle persistence
        if self.min_obstacle_distance < self.stop_distance:
            self.obstacle_persist_count += 1
        else:
            self.obstacle_persist_count = max(0, self.obstacle_persist_count - 1)
        
        # Update emergency state with adaptive threshold
        adaptive_stop_distance = self.get_adaptive_stop_distance()
        was_emergency = self.emergency_active
        
        if self.min_obstacle_distance < adaptive_stop_distance:
            self.emergency_active = True
            if not was_emergency:
                self.get_logger().error(f"🚨 EMERGENCY STOP! Obstacle at {self.min_obstacle_distance:.2f}m")
        else:
            if self.emergency_active:
                self.emergency_active = False
                self.get_logger().info(f"✅ Emergency cleared - obstacle now at {self.min_obstacle_distance:.2f}m")
    
    def get_adaptive_stop_distance(self):
        """Adjust stop distance based on recovery state"""
        base_distance = self.stop_distance
        
        # If obstacle persists for long time (robot might be stuck), allow closer approach
        if self.obstacle_persist_count > 10:
            return max(0.20, base_distance * 0.67)  # Reduce threshold when stuck
        
        return base_distance
    
    def publish_safe_cmd(self):
        """Publish safe command - either pass-through or stop"""
        safe_cmd = Twist()
        
        if self.emergency_active:
            # Emergency stop - zero velocity
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
        elif self.min_obstacle_distance < self.slow_distance:
            # Slow down - reduce to 50%
            safe_cmd.linear.x = self.last_cmd.linear.x * 0.5
            safe_cmd.angular.z = self.last_cmd.angular.z
        else:
            # Pass through - copy values explicitly
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
