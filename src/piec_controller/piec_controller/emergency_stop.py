#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy

_BLIND_SPOT_DISTANCE = 0.5


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')

        self.declare_parameter('stop_distance', 0.80)
        self.declare_parameter('slow_distance', 0.90)
        self.declare_parameter('hysteresis_margin', 0.12)
        self.declare_parameter('clear_count_threshold', 5)
        self.declare_parameter('enable_emergency_stop', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_piec')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('body_cone_deg', 35.0)          # ±35° for 60cm body

        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.hysteresis_margin = self.get_parameter('hysteresis_margin').value
        self.clear_count_threshold = self.get_parameter('clear_count_threshold').value
        self.body_cone_rad = math.radians(self.get_parameter('body_cone_deg').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.emergency_active = False
        self.last_cmd = Twist()
        self.min_obstacle_distance = 10.0
        self.obstacle_direction = 0.0
        self.obstacle_persist_count = 0
        self.clear_persist_count = 0

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        out_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, out_qos)
        self.reason_pub = self.create_publisher(String, '/emergency_stop/reason', out_qos)

        self.create_timer(0.05, self.publish_safe_cmd)

        self.get_logger().info('🛑 Emergency Stop ready (body cone = ±35°)')
        self.get_logger().info(f'  Stop dist: {self.stop_distance}m, Slow dist: {self.slow_distance}m')

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def scan_callback(self, msg: LaserScan):
        if not self.get_parameter('enable_emergency_stop').value:
            self.min_obstacle_distance = 10.0
            return

        ranges = np.array(msg.ranges, dtype=np.float64)
        n = len(ranges)
        if n == 0:
            return

        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        front_mask = np.abs(angles) <= self.body_cone_rad

        if not np.any(front_mask):
            self.min_obstacle_distance = 10.0
            self.obstacle_direction = 0.0
            return

        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]

        valid_mask = (np.isfinite(front_ranges) &
                      (front_ranges > msg.range_min) &
                      (front_ranges < msg.range_max))

        if not np.any(valid_mask):
            self.min_obstacle_distance = msg.range_min if msg.range_min > 0.0 else _BLIND_SPOT_DISTANCE
            self.obstacle_direction = 0.0
        else:
            valid_ranges = front_ranges[valid_mask]
            valid_angles = front_angles[valid_mask]
            min_idx = np.argmin(valid_ranges)
            self.min_obstacle_distance = float(valid_ranges[min_idx])
            self.obstacle_direction = float(valid_angles[min_idx])

        clear_threshold = self.stop_distance + self.hysteresis_margin

        if self.min_obstacle_distance < self.stop_distance:
            self.obstacle_persist_count += 1
            self.clear_persist_count = 0

            if not self.emergency_active:
                self.emergency_active = True
                reason = f'BODY_OBSTACLE={self.min_obstacle_distance:.2f}m @ {math.degrees(self.obstacle_direction):.1f}°'
                self.get_logger().error(f'🚨 EMERGENCY STOP! {reason}')
                self._publish_reason(reason)
        else:
            self.obstacle_persist_count = max(0, self.obstacle_persist_count - 1)

            if self.emergency_active:
                if self.min_obstacle_distance >= clear_threshold:
                    self.clear_persist_count += 1
                else:
                    self.clear_persist_count = 0

                if self.clear_persist_count >= self.clear_count_threshold:
                    self.emergency_active = False
                    self.clear_persist_count = 0
                    self.get_logger().info(
                        f'✅ Emergency cleared – obstacle at {self.min_obstacle_distance:.2f}m '
                        f'(>{clear_threshold:.2f}m)'
                    )

    def publish_safe_cmd(self):
        safe_cmd = Twist()

        if self.emergency_active:
            # Allow backward motion (negative x) but block forward (positive x)
            if self.last_cmd.linear.x < 0.0:
                safe_cmd.linear.x = self.last_cmd.linear.x   # allow backup
            else:
                safe_cmd.linear.x = 0.0                     # block forward
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            # Pass angular velocity unchanged – allows rotation
            safe_cmd.angular.x = self.last_cmd.angular.x
            safe_cmd.angular.y = self.last_cmd.angular.y
            safe_cmd.angular.z = self.last_cmd.angular.z

            self.get_logger().debug(
                f'🚨 EMERGENCY: blocking forward, allowing backward v={safe_cmd.linear.x:.2f}, w={safe_cmd.angular.z:.2f}'
            )

        elif self.min_obstacle_distance < self.slow_distance:
            scale = (self.min_obstacle_distance - self.stop_distance) / max(
                self.slow_distance - self.stop_distance, 0.01
            )
            scale = max(0.1, min(1.0, scale))
            safe_cmd.linear.x = self.last_cmd.linear.x * scale
            safe_cmd.linear.y = self.last_cmd.linear.y
            safe_cmd.linear.z = self.last_cmd.linear.z
            safe_cmd.angular.x = self.last_cmd.angular.x
            safe_cmd.angular.y = self.last_cmd.angular.y
            safe_cmd.angular.z = self.last_cmd.angular.z

        else:
            safe_cmd = self.last_cmd

        self.cmd_pub.publish(safe_cmd)

    def _publish_reason(self, reason: str):
        msg = String()
        msg.data = reason
        self.reason_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Emergency stop node shutting down…')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
