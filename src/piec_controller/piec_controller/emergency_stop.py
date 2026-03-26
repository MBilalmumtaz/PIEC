#!/usr/bin/env python3
"""
Emergency Stop node with improved hysteresis, diagnostic logging, and
rotation passthrough so the robot can escape an obstacle by turning.

Key fixes vs original:
- Angular velocity (rotation) is ALWAYS passed through, even during an
  emergency stop, allowing the robot to rotate away from the obstacle.
- Hysteresis: emergency only clears when distance > stop_dist + margin
  for ``clear_count_threshold`` consecutive scan cycles (prevents toggling).
- Detailed per-stop logging: distance, persist count, obstacle direction.
- RELIABLE QoS publisher so downstream safety topics are guaranteed.
"""
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
)

# When the forward sector is entirely empty / below range_min we conservatively
# assume the blind-spot hides a very-close obstacle.
_BLIND_SPOT_DISTANCE = 0.5  # metres


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('stop_distance', 0.55)      # reduced: 0.9→0.55 m
        self.declare_parameter('slow_distance', 0.90)      # slow-down zone
        self.declare_parameter('hysteresis_margin', 0.12)  # clear when dist > stop+margin
        self.declare_parameter('clear_count_threshold', 5) # cycles above threshold to clear
        self.declare_parameter('enable_emergency_stop', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_piec')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('forward_cone_deg', 60.0)   # half-angle of forward danger cone

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.hysteresis_margin = float(self.get_parameter('hysteresis_margin').value)
        self.clear_count_threshold = int(self.get_parameter('clear_count_threshold').value)
        self.forward_cone_rad = math.radians(
            float(self.get_parameter('forward_cone_deg').value)
        )
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        # ── State ─────────────────────────────────────────────────────────────
        self.emergency_active = False
        self.last_cmd = Twist()
        self.min_obstacle_distance = 10.0
        self.obstacle_direction = 0.0       # angle (rad) of closest obstacle
        # Persistence counters
        self.obstacle_persist_count = 0     # consecutive cycles below stop_dist
        self.clear_persist_count = 0        # consecutive cycles above clear threshold

        # ── QoS ───────────────────────────────────────────────────────────────
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Safety output must be RELIABLE so downstream consumers always receive it
        out_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, out_qos)
        # Structured stop-reason string for diagnostics / CSV logger
        self.reason_pub = self.create_publisher(String, '/emergency_stop/reason', out_qos)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(0.05, self.publish_safe_cmd)   # 20 Hz

        self.get_logger().info('🛑 Emergency Stop ready:')
        self.get_logger().info(f'  Input cmd: {self.cmd_vel_topic}')
        self.get_logger().info(f'  Output cmd: {self.output_topic}')
        self.get_logger().info(f'  Scan: {self.scan_topic} (BEST_EFFORT)')
        self.get_logger().info(f'  Stop dist: {self.stop_distance:.2f}m  '
                               f'Slow dist: {self.slow_distance:.2f}m  '
                               f'Hysteresis margin: {self.hysteresis_margin:.2f}m  '
                               f'Clear threshold: {self.clear_count_threshold} cycles')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cmd_callback(self, msg: Twist):
        """Store latest controller command (before safety gating)."""
        self.last_cmd.linear.x = msg.linear.x
        self.last_cmd.linear.y = msg.linear.y
        self.last_cmd.linear.z = msg.linear.z
        self.last_cmd.angular.x = msg.angular.x
        self.last_cmd.angular.y = msg.angular.y
        self.last_cmd.angular.z = msg.angular.z

    def scan_callback(self, msg: LaserScan):
        """Process scan; update emergency state with hysteresis."""
        if not self.get_parameter('enable_emergency_stop').value:
            self.min_obstacle_distance = 10.0
            return

        ranges = np.array(msg.ranges, dtype=np.float64)
        n = len(ranges)
        if n == 0:
            return

        # ── Identify forward sector indices ───────────────────────────────────
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        front_mask = np.abs(angles) <= self.forward_cone_rad

        if not np.any(front_mask):
            return

        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]

        # Collect valid readings (range_min < r < range_max and finite)
        valid_mask = (
            np.isfinite(front_ranges)
            & (front_ranges > msg.range_min)
            & (front_ranges < msg.range_max)
        )

        if not np.any(valid_mask):
            # Blind-spot: conservatively assume obstacle at range_min
            self.min_obstacle_distance = (
                msg.range_min if msg.range_min > 0.0 else _BLIND_SPOT_DISTANCE
            )
            self.obstacle_direction = 0.0
        else:
            valid_ranges = front_ranges[valid_mask]
            valid_angles = front_angles[valid_mask]
            min_idx = int(np.argmin(valid_ranges))
            self.min_obstacle_distance = float(valid_ranges[min_idx])
            self.obstacle_direction = float(valid_angles[min_idx])

        # ── Hysteresis state machine ──────────────────────────────────────────
        clear_threshold = self.stop_distance + self.hysteresis_margin

        if self.min_obstacle_distance < self.stop_distance:
            self.obstacle_persist_count += 1
            self.clear_persist_count = 0

            if not self.emergency_active:
                self.emergency_active = True
                reason = (
                    f'obstacle={self.min_obstacle_distance:.2f}m @ '
                    f'{math.degrees(self.obstacle_direction):.1f}°  '
                    f'persist={self.obstacle_persist_count}'
                )
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
                        f'✅ Emergency cleared – obstacle now at '
                        f'{self.min_obstacle_distance:.2f}m '
                        f'(>{clear_threshold:.2f}m for '
                        f'{self.clear_count_threshold} cycles)'
                    )

    # ── Publisher ─────────────────────────────────────────────────────────────

    def publish_safe_cmd(self):
        """Gate controller command through safety logic and publish."""
        safe_cmd = Twist()

        if self.emergency_active:
            # CRITICAL FIX: Block ONLY forward linear motion.
            # Allow angular velocity so the robot can rotate away from the obstacle.
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            # Pass angular velocity through unchanged so the robot can turn
            safe_cmd.angular.x = self.last_cmd.angular.x
            safe_cmd.angular.y = self.last_cmd.angular.y
            safe_cmd.angular.z = self.last_cmd.angular.z

        elif self.min_obstacle_distance < self.slow_distance:
            # Slow-down zone: scale forward speed proportionally with hysteresis
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
            # Free zone: pass everything through
            safe_cmd.linear.x = self.last_cmd.linear.x
            safe_cmd.linear.y = self.last_cmd.linear.y
            safe_cmd.linear.z = self.last_cmd.linear.z
            safe_cmd.angular.x = self.last_cmd.angular.x
            safe_cmd.angular.y = self.last_cmd.angular.y
            safe_cmd.angular.z = self.last_cmd.angular.z

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
