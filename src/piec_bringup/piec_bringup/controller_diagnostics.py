#!/usr/bin/env python3
"""
Controller Diagnostics Tool
Validates controller behavior, especially for lateral goals
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import sys


class ControllerDiagnostics(Node):
    def __init__(self):
        super().__init__('controller_diagnostics')
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ukf/odom',
            self.odom_callback,
            qos_reliable
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/piec/path',
            self.path_callback,
            qos_reliable
        )
        
        # State variables
        self.odom = None
        self.path = None
        self.last_check_time = self.get_clock().now()
        
        # Create timer for periodic checks
        self.create_timer(2.0, self.diagnostic_check)
        
        self.get_logger().info("🔍 Controller Diagnostics Node Started")
        self.get_logger().info("   Monitoring /ukf/odom and /piec/path")
    
    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def odom_callback(self, msg):
        """Store latest odometry"""
        self.odom = msg
    
    def path_callback(self, msg):
        """Store latest path"""
        self.path = msg
    
    def diagnostic_check(self):
        """Perform diagnostic checks"""
        if self.odom is None:
            self.get_logger().warn("⚠️  No odometry data received yet")
            return
        
        # Extract current pose
        current_pose = self.odom.pose.pose
        x = current_pose.position.x
        y = current_pose.position.y
        yaw = self.quat_to_yaw(current_pose.orientation)
        
        self.get_logger().info("━" * 60)
        self.get_logger().info(f"📍 Current Robot Pose:")
        self.get_logger().info(f"   Position: ({x:.3f}, {y:.3f})")
        self.get_logger().info(f"   Yaw: {math.degrees(yaw):.1f}° ({yaw:.3f} rad)")
        
        # Check velocity
        vx = self.odom.twist.twist.linear.x
        vy = self.odom.twist.twist.linear.y
        w = self.odom.twist.twist.angular.z
        v_total = math.hypot(vx, vy)
        
        self.get_logger().info(f"🚗 Current Velocity:")
        self.get_logger().info(f"   Linear: {v_total:.3f} m/s")
        self.get_logger().info(f"   Angular: {w:.3f} rad/s ({math.degrees(w):.1f}°/s)")
        
        # Check path if available
        if self.path is not None and len(self.path.poses) > 0:
            self.get_logger().info(f"🛤️  Path Information:")
            self.get_logger().info(f"   Points: {len(self.path.poses)}")
            
            # Check if path start is close to robot
            start_pose = self.path.poses[0].pose.position
            start_dist = math.hypot(start_pose.x - x, start_pose.y - y)
            self.get_logger().info(f"   Start distance: {start_dist:.3f} m")
            
            if start_dist > 0.5:
                self.get_logger().warn(
                    f"   ⚠️  Path start is far from robot! ({start_dist:.3f}m)"
                )
            
            # Check goal position and bearing
            goal_pose = self.path.poses[-1].pose.position
            goal_x = goal_pose.x
            goal_y = goal_pose.y
            
            dx = goal_x - x
            dy = goal_y - y
            distance = math.hypot(dx, dy)
            
            # Calculate bearing to goal
            target_bearing = math.atan2(dy, dx)
            angle_error = self.wrap_angle(target_bearing - yaw)
            
            self.get_logger().info(f"🎯 Goal Information:")
            self.get_logger().info(f"   Goal: ({goal_x:.3f}, {goal_y:.3f})")
            self.get_logger().info(f"   Distance: {distance:.3f} m")
            self.get_logger().info(f"   Target bearing: {math.degrees(target_bearing):.1f}° ({target_bearing:.3f} rad)")
            self.get_logger().info(f"   Angle error: {math.degrees(angle_error):.1f}° ({angle_error:.3f} rad)")
            
            # Diagnose goal direction
            if abs(angle_error) > math.radians(135):
                self.get_logger().info("   📍 Goal is BEHIND robot")
            elif abs(angle_error) > math.radians(45):
                if angle_error > 0:
                    self.get_logger().info("   📍 Goal is to the LEFT side")
                else:
                    self.get_logger().info("   📍 Goal is to the RIGHT side")
            else:
                self.get_logger().info("   📍 Goal is AHEAD of robot")
            
            # Expected angular velocity sign
            if abs(angle_error) > math.radians(2):
                expected_w_sign = "positive (CCW)" if angle_error > 0 else "negative (CW)"
                self.get_logger().info(f"   Expected w sign: {expected_w_sign}")
                
                if w != 0.0:
                    actual_w_sign = "positive (CCW)" if w > 0 else "negative (CW)"
                    self.get_logger().info(f"   Actual w sign: {actual_w_sign}")
                    
                    # Check if signs match
                    if (angle_error > 0 and w < 0) or (angle_error < 0 and w > 0):
                        self.get_logger().error(
                            "   ❌ SIGN MISMATCH! Controller turning wrong direction!"
                        )
                    else:
                        self.get_logger().info("   ✅ Turn direction is correct")
        else:
            self.get_logger().info("🛤️  No path received yet")
        
        self.get_logger().info("━" * 60)


def main(args=None):
    rclpy.init(args=args)
    
    node = ControllerDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
