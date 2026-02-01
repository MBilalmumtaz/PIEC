#!/usr/bin/env python3
"""
Enhanced PIEC Controller with Improved Obstacle Handling
Integrates with enhanced DWA planner and proper laser scan processing
"""
import rclpy
import math
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

# Import the enhanced DWA planner
from .dynamic_dwa import DynamicDWAPlanner


class ControllerNode(Node):
    def __init__(self):
        super().__init__('enhanced_piec_controller')

        # Load parameters from YAML file
        self.load_parameters_from_yaml()

        # Initialize ALL state variables
        self.path = None
        self.odom = None
        self.scan_ranges = None
        self.scan_angles = None
        self.scan_time = None
        self.current_waypoint_idx = 0
        self.last_path_update = self.get_clock().now()
        self.path_received = False

        # Goal tracking
        self.has_explicit_goal = False
        self.last_explicit_goal = None
        self.explicit_goal_timeout = 30.0

        # Enhanced obstacle tracking
        self.obstacle_map = {}  # Grid-based obstacle memory
        self.map_resolution = 0.1
        self.obstacle_decay_rate = 0.95  # Obstacles fade over time

        # Safety tracking
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.obstacle_direction = 0.0
        self.obstacle_type = 'static'  # static or dynamic
        self.consecutive_obstacle_detections = 0
        self.last_safe_time = self.get_clock().now()

        # Statistics
        self.distance_traveled = 0.0
        self.last_position = None
        self.control_counter = 0
        self.avg_speed = 0.0
        self.speed_history = []
        self.obstacle_encounters = 0

        # DWA planner
        self.dwa_failures = 0
        self.max_dwa_failures = 5
        self.simple_control_active = False
        self.last_cmd = (0.0, 0.0)  # For acceleration limiting

        # Enhanced DWA Planner
        self.dwa = DynamicDWAPlanner(self, emergency_distance=self.emergency_stop_distance)

        # Update DWA parameters
        self.update_dwa_parameters()

        # QoS Settings
        scan_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscribers
        self.create_subscription(Path, '/piec/path', self.path_callback, 10)
        self.create_subscription(Odometry, '/ukf/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan_fixed', self.scan_callback, scan_qos)

        # Subscribe to goal topic
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Optional: Subscribe to PINN service status
        self.declare_parameter('use_pinn_in_dwa', False)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.debug_mode:
            self.target_pub = self.create_publisher(PoseStamped, '/debug/target', 10)
            self.obstacle_pub = self.create_publisher(PoseStamped, '/debug/nearest_obstacle', 10)
            self.safety_pub = self.create_publisher(Twist, '/debug/safety', 10)

        # Control Timer
        control_freq = self.control_frequency
        self.create_timer(1.0/control_freq, self.control_loop)

        # Obstacle map maintenance timer
        self.create_timer(1.0, self.update_obstacle_map)

        self.get_logger().info("🚀 Enhanced PIEC Controller READY")
        self.get_logger().info(f"Max linear: {self.max_linear_vel}m/s, Safety distance: {self.safe_distance}m")

    # ------------------------------------------------------------------
    #  Parameter loading
    # ------------------------------------------------------------------
    def load_parameters_from_yaml(self):
        """Load parameters from YAML configuration file"""
        # Try to load from YAML first
        config_path = os.path.join(
            get_package_share_directory('piec_controller'),
            'config',
            'controller_params.yaml'
        )

        # Default parameters
        params = {
            # Speed Parameters
            'max_linear_vel': 2,
            'max_angular_vel': 0.5,
            'min_linear_vel': 0.5,
            'acceleration_limit': 0.5,
            'deceleration_limit': 1.5,

            # Safety Parameters
            'emergency_stop_distance': 0.15,
            'slow_down_distance': 0.8,
            'safe_distance': 1.0,
            'obstacle_clearance': 0.5,
            'lateral_safety_margin': 0.4,

            # Path Following
            'waypoint_tolerance': 0.3,
            'path_timeout': 30.0,
            'lookahead_distance': 1.5,
            'adaptive_lookahead': True,
            'path_replan_distance': 1.0,

            # Recovery Parameters
            'recovery_timeout': 5.0,
            'stuck_threshold': 2.0,
            'stuck_time': 10.0,

            # DWA Parameters
            'dwa_max_v': 2,
            'dwa_min_v': 0.1,
            'dwa_max_w': 1.2,
            'dwa_sim_time': 2.0,
            'dwa_dt': 0.1,
            'dwa_clearance_weight': 2.5,

            # Control Parameters
            'control_frequency': 20.0,
            'debug_mode': True,
            'require_explicit_goal': True,
            'enable_obstacle_memory': True,
            'obstacle_memory_decay': 0.95,
        }

        # Load from YAML if file exists
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as file:
                    yaml_params = yaml.safe_load(file)
                    if yaml_params and '/**' in yaml_params and 'ros__parameters' in yaml_params['/**']:
                        params.update(yaml_params['/**']['ros__parameters'])
                self.get_logger().info(f"✅ Loaded parameters from: {config_path}")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to load YAML: {e}, using defaults")

        # Declare all parameters
        for key, value in params.items():
            self.declare_parameter(key, value)

        # Store as instance variables
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.acceleration_limit = self.get_parameter('acceleration_limit').value
        self.deceleration_limit = self.get_parameter('deceleration_limit').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        self.slow_down_distance = self.get_parameter('slow_down_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.obstacle_clearance = self.get_parameter('obstacle_clearance').value
        self.lateral_safety_margin = self.get_parameter('lateral_safety_margin').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.path_timeout = self.get_parameter('path_timeout').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.adaptive_lookahead = self.get_parameter('adaptive_lookahead').value
        self.path_replan_distance = self.get_parameter('path_replan_distance').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.stuck_time = self.get_parameter('stuck_time').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.require_explicit_goal = self.get_parameter('require_explicit_goal').value
        self.enable_obstacle_memory = self.get_parameter('enable_obstacle_memory').value
        self.obstacle_memory_decay = self.get_parameter('obstacle_memory_decay').value

    def update_dwa_parameters(self):
        """Update DWA planner parameters"""
        if hasattr(self.dwa, 'max_v'):
            self.dwa.max_v = self.get_parameter('dwa_max_v').value
        if hasattr(self.dwa, 'min_v'):
            self.dwa.min_v = self.get_parameter('dwa_min_v').value
        if hasattr(self.dwa, 'max_w'):
            self.dwa.max_w = self.get_parameter('dwa_max_w').value
        if hasattr(self.dwa, 'sim_time'):
            self.dwa.sim_time = self.get_parameter('dwa_sim_time').value
        if hasattr(self.dwa, 'dt'):
            self.dwa.dt = self.get_parameter('dwa_dt').value
        if hasattr(self.dwa, 'w_clearance'):
            self.dwa.w_clearance = self.get_parameter('dwa_clearance_weight').value

        self.get_logger().info(f"DWA configured: v=[{self.dwa.min_v:.1f},{self.dwa.max_v:.1f}], "
                             f"w=[{-self.dwa.max_w:.1f},{self.dwa.max_w:.1f}], "
                             f"sim_time={self.dwa.sim_time:.1f}s")

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------
    def goal_callback(self, msg: PoseStamped):
        """Handle explicit goal messages"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        self.has_explicit_goal = True
        self.last_explicit_goal = (goal_x, goal_y)
        self.last_goal_update = self.get_clock().now()

        if self.debug_mode:
            self.get_logger().info(f"🎯 Received explicit goal: ({goal_x:.3f}, {goal_y:.3f})")

    def path_callback(self, msg: Path):
        """Handle new path from optimizer"""
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path!")
            return

        # Check if we should accept this path
        if self.require_explicit_goal and not self.has_explicit_goal:
            if self.debug_mode:
                self.get_logger().info("Ignoring path - no explicit goal received yet")
            return

        self.path = msg
        self.current_waypoint_idx = 0
        self.last_path_update = self.get_clock().now()
        self.path_received = True
        self.dwa_failures = 0
        self.simple_control_active = False

        start_x = msg.poses[0].pose.position.x
        start_y = msg.poses[0].pose.position.y
        goal_x = msg.poses[-1].pose.position.x
        goal_y = msg.poses[-1].pose.position.y

        path_length = 0.0
        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i + 1].pose.position
            path_length += math.hypot(p2.x - p1.x, p2.y - p1.y)

        self.get_logger().info(f"📈 New path received: {len(msg.poses)} waypoints, {path_length:.2f}m")
        self.get_logger().info(f"  Start: ({start_x:.3f}, {start_y:.3f})")
        self.get_logger().info(f"  Goal: ({goal_x:.3f}, {goal_y:.3f})")

    def odom_callback(self, msg: Odometry):
        """Update robot pose from UKF"""
        self.odom = msg

        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_position is not None:
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            self.distance_traveled += math.hypot(dx, dy)

        self.last_position = current_position

    def scan_callback(self, msg: LaserScan):
        """Process laser scan data with enhanced obstacle detection"""
        if len(msg.ranges) == 0:
            return

        self.scan_ranges = msg.ranges
        self.scan_time = self.get_clock().now()

        # Initialize scan angles if needed
        if self.scan_angles is None or len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(len(msg.ranges))
            ]

        # Enhanced obstacle detection
        self.enhanced_obstacle_detection(msg)

        # Update obstacle map if enabled
        if self.enable_obstacle_memory and self.odom is not None:
            self.update_obstacle_map_from_scan(msg)

    def enhanced_obstacle_detection(self, scan_msg: LaserScan):
        """Enhanced obstacle detection with multiple criteria"""
        if self.scan_ranges is None or self.scan_angles is None:
            return

        ranges = np.array(self.scan_ranges)
        angles = np.array(self.scan_angles)

        # Multiple detection zones
        zones = [
            {'name': 'emergency', 'angle_range': 30, 'distance': self.emergency_stop_distance},
            {'name': 'slow', 'angle_range': 60, 'distance': self.slow_down_distance},
            {'name': 'caution', 'angle_range': 90, 'distance': self.safe_distance},
        ]

        zone_detections = {zone['name']: False for zone in zones}
        zone_distances = {zone['name']: float('inf') for zone in zones}
        zone_directions = {zone['name']: 0.0 for zone in zones}

        for zone in zones:
            angle_limit = math.radians(zone['angle_range'] / 2)

            # Find minimum distance in this zone
            for i, (range_val, angle) in enumerate(zip(ranges, angles)):
                if abs(angle) < angle_limit:
                    if 0.1 < range_val < zone['distance']:
                        zone_detections[zone['name']] = True
                        if range_val < zone_distances[zone['name']]:
                            zone_distances[zone['name']] = range_val
                            zone_directions[zone['name']] = angle

        # Determine overall obstacle state
        if zone_detections['emergency']:
            self.obstacle_detected = True
            self.obstacle_distance = zone_distances['emergency']
            self.obstacle_direction = zone_directions['emergency']
            self.obstacle_type = self.detect_obstacle_type(ranges, angles)
            self.consecutive_obstacle_detections += 1

            if self.debug_mode and self.control_counter % 5 == 0:
                self.get_logger().warn(f"⚠️ EMERGENCY obstacle at {self.obstacle_distance:.2f}m, "
                                      f"direction {math.degrees(self.obstacle_direction):.1f}°")

        elif zone_detections['slow']:
            self.obstacle_detected = True
            self.obstacle_distance = zone_distances['slow']
            self.obstacle_direction = zone_directions['slow']
            self.obstacle_type = self.detect_obstacle_type(ranges, angles)
            self.consecutive_obstacle_detections += 1

            if self.debug_mode and self.control_counter % 10 == 0:
                self.get_logger().info(f"🚧 Obstacle at {self.obstacle_distance:.2f}m, "
                                      f"direction {math.degrees(self.obstacle_direction):.1f}°")

        elif zone_detections['caution']:
            self.obstacle_detected = True
            self.obstacle_distance = zone_distances['caution']
            self.obstacle_direction = zone_directions['caution']
            self.consecutive_obstacle_detections = max(0, self.consecutive_obstacle_detections - 1)

        else:
            self.obstacle_detected = False
            self.obstacle_distance = float('inf')
            self.consecutive_obstacle_detections = 0
            self.last_safe_time = self.get_clock().now()

    def detect_obstacle_type(self, ranges, angles):
        """Detect if obstacle is static or dynamic"""
        # Simple heuristic based on range pattern
        front_ranges = []
        for i, (range_val, angle) in enumerate(zip(ranges, angles)):
            if abs(angle) < math.radians(30) and range_val < self.safe_distance:
                front_ranges.append(range_val)

        if len(front_ranges) < 3:
            return 'static'

        # Check for moving pattern (variance in close ranges)
        range_std = np.std(front_ranges)
        if range_std > 0.1 and len(front_ranges) > 5:
            return 'dynamic'

        return 'static'

    def update_obstacle_map_from_scan(self, scan_msg: LaserScan):
        """Update obstacle map from laser scan"""
        if self.odom is None:
            return

        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        ryaw = self.quat_to_yaw(self.odom.pose.pose.orientation)

        for i, range_val in enumerate(scan_msg.ranges):
            if 0.1 < range_val < 5.0:  # Valid range
                angle = scan_msg.angle_min + i * scan_msg.angle_increment

                # Convert to world coordinates
                local_x = range_val * math.cos(angle)
                local_y = range_val * math.sin(angle)

                world_x = rx + local_x * math.cos(ryaw) - local_y * math.sin(ryaw)
                world_y = ry + local_x * math.sin(ryaw) + local_y * math.cos(ryaw)

                # Grid coordinates
                grid_x = int(world_x / self.map_resolution)
                grid_y = int(world_y / self.map_resolution)

                # Update obstacle map
                key = (grid_x, grid_y)
                if key in self.obstacle_map:
                    self.obstacle_map[key]['count'] += 1
                    self.obstacle_map[key]['last_seen'] = self.get_clock().now()
                else:
                    self.obstacle_map[key] = {
                        'x': world_x,
                        'y': world_y,
                        'count': 1,
                        'last_seen': self.get_clock().now(),
                        'confidence': 1.0
                    }

    def update_obstacle_map(self):
        """Maintain obstacle map (decay old obstacles)"""
        if not self.enable_obstacle_memory:
            return

        current_time = self.get_clock().now()
        keys_to_remove = []

        for key, obstacle in self.obstacle_map.items():
            # Decay confidence
            time_diff = (current_time - obstacle['last_seen']).nanoseconds * 1e-9
            decay_factor = self.obstacle_memory_decay ** time_diff
            obstacle['confidence'] *= decay_factor

            # Remove if confidence too low
            if obstacle['confidence'] < 0.1:
                keys_to_remove.append(key)

        for key in keys_to_remove:
            del self.obstacle_map[key]

        if self.debug_mode and len(keys_to_remove) > 0:
            self.get_logger().debug(f"Removed {len(keys_to_remove)} old obstacles from map")

    # ------------------------------------------------------------------
    #  Control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        """Main control loop - SIMPLIFIED"""
        self.control_counter += 1

        # Check for essential data
        if self.odom is None:
            if self.control_counter % 100 == 0:
                self.get_logger().debug("Waiting for odometry...")
            return

        if self.scan_ranges is None:
            if self.control_counter % 100 == 0:
                self.get_logger().debug("Waiting for laser scan...")
            return

        # Get current pose
        pose = self.odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quat_to_yaw(pose.orientation)

        # Update distance tracking
        current_position = (x, y)
        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            self.distance_traveled += math.hypot(dx, dy)
        self.last_position = current_position

        # Emergency stop ONLY for immediate danger
        if self.obstacle_detected and self.obstacle_distance < 0.2:
            if self.debug_mode:
                self.get_logger().error(f"🚨 EMERGENCY STOP at {self.obstacle_distance:.2f}m!")
            self.publish_cmd(0.0, 0.0)
            return

        # Check if we have a valid path
        if self.path is None or not self.path_received:
            if self.control_counter % 50 == 0:
                self.get_logger().debug("No active path - holding")
            self.publish_cmd(0.0, 0.0)
            return

        # Update current waypoint
        self.update_current_waypoint(x, y)

        # Check if we've reached all waypoints
        if self.current_waypoint_idx >= len(self.path.poses):
            if self.debug_mode:
                self.get_logger().info("🎯 All waypoints reached!")
            self.publish_cmd(0.0, 0.0)
            self.path_received = False
            return

        # Get target waypoint with adaptive lookahead
        lookahead_idx = self.calculate_adaptive_lookahead(x, y, yaw)
        target_waypoint = self.path.poses[lookahead_idx].pose.position

        if self.debug_mode:
            target_msg = PoseStamped()
            target_msg.header.frame_id = "map"
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.pose.position = target_waypoint
            self.target_pub.publish(target_msg)

        # Plan using enhanced DWA
        try:
            v, w = self.dwa.plan_with_obstacle_awareness(
                pose=(x, y, yaw),
                path=self.path,
                scan_ranges=self.scan_ranges,
                scan_angles=self.scan_angles
            )

            # Apply enhanced speed limits based on obstacles
            v = self.apply_enhanced_speed_limit(v, w)

            # Apply path curvature-based speed adjustment
            v = self.adjust_speed_for_curvature(v, w)

            if self.debug_mode and self.control_counter % 10 == 0:
                self.get_logger().info(f"🎯 DWA output: v={v:.3f}, w={w:.3f}")

            # Check for DWA failure
            if abs(v) < 0.05 and abs(w) < 0.1:
                self.dwa_failures += 1
                if self.dwa_failures > self.max_dwa_failures:
                    self.get_logger().warn("⚠️ DWA struggling, switching to enhanced fallback")
                    self.simple_control_active = True
            else:
                self.dwa_failures = 0
                self.simple_control_active = False

        except Exception as e:
            self.get_logger().error(f"❌ DWA planning failed: {str(e)}")
            self.simple_control_active = True

        # Use enhanced fallback control if DWA is failing
        if self.simple_control_active:
            v, w = self.enhanced_fallback_control(target_waypoint, x, y, yaw)

        # Apply acceleration/deceleration limits
        v, w = self.apply_motion_limits(v, w)

        # Ensure velocities are within limits
        v = np.clip(v, 0.0, self.max_linear_vel)
        w = np.clip(w, -self.max_angular_vel, self.max_angular_vel)

        # Apply minimum velocity threshold
        if v > 0 and v < self.min_linear_vel:
            v = self.min_linear_vel

        # Update statistics
        self.update_statistics(v)

        # Publish command
        self.publish_cmd(v, w)

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------
    def calculate_adaptive_lookahead(self, x, y, yaw):
        """Calculate adaptive lookahead index based on environment"""
        if not self.adaptive_lookahead or self.path is None:
            return min(self.current_waypoint_idx + 3, len(self.path.poses) - 1)

        # Base lookahead
        lookahead = 3

        # Adjust based on speed
        if self.last_cmd[0] > self.max_linear_vel * 0.7:
            lookahead += 2

        # Adjust based on obstacles
        if self.obstacle_detected:
            if self.obstacle_distance < self.slow_down_distance:
                lookahead = max(1, lookahead - 2)
            elif self.obstacle_distance < self.safe_distance:
                lookahead = max(2, lookahead - 1)

        # Adjust based on path curvature
        if self.current_waypoint_idx < len(self.path.poses) - 3:
            current = self.path.poses[self.current_waypoint_idx].pose.position
            next_wp = self.path.poses[self.current_waypoint_idx + 1].pose.position
            next_next_wp = self.path.poses[self.current_waypoint_idx + 2].pose.position

            v1 = np.array([next_wp.x - current.x, next_wp.y - current.y])
            v2 = np.array([next_next_wp.x - next_wp.x, next_next_wp.y - next_wp.y])

            if np.linalg.norm(v1) > 0.1 and np.linalg.norm(v2) > 0.1:
                v1_norm = v1 / np.linalg.norm(v1)
                v2_norm = v2 / np.linalg.norm(v2)
                cos_angle = np.dot(v1_norm, v2_norm)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle = np.arccos(cos_angle)

                if angle > math.radians(30):  # Sharp turn
                    lookahead = max(2, lookahead - 1)

        return min(self.current_waypoint_idx + lookahead, len(self.path.poses) - 1)

    def apply_enhanced_speed_limit(self, velocity, angular_velocity):
        """Enhanced speed limiting based on multiple factors"""
        if not self.obstacle_detected or self.obstacle_distance == float('inf'):
            return min(velocity, self.max_linear_vel)

        # Emergency stop
        if self.obstacle_distance < self.emergency_stop_distance:
            return 0.0

        # Slow down zone
        elif self.obstacle_distance < self.slow_down_distance:
            # Calculate reduction factor
            t = (self.obstacle_distance - self.emergency_stop_distance) / \
                (self.slow_down_distance - self.emergency_stop_distance)

            # Sigmoid reduction for smoother transition
            reduction_factor = 1.0 / (1.0 + math.exp(-10.0 * (t - 0.5)))

            # Additional reduction for dynamic obstacles
            if self.obstacle_type == 'dynamic':
                reduction_factor *= 0.7

            return velocity * reduction_factor

        # Caution zone
        elif self.obstacle_distance < self.safe_distance:
            # Mild reduction
            reduction = 0.1 + 0.4 * (1.0 - self.obstacle_distance / self.safe_distance)
            return velocity * (1.0 - reduction)

        else:
            return min(velocity, self.max_linear_vel)

    def adjust_speed_for_curvature(self, base_speed, angular_velocity):
        """Adjust speed based on path curvature and turning"""
        adjusted_speed = base_speed

        # Reduce speed during sharp turns
        turning_factor = 1.0 - (abs(angular_velocity) / self.max_angular_vel) * 0.6
        adjusted_speed *= turning_factor

        # Check upcoming path curvature
        if self.path is not None and self.current_waypoint_idx < len(self.path.poses) - 2:
            # Analyze next few segments
            curvature_sum = 0.0
            segment_count = min(3, len(self.path.poses) - self.current_waypoint_idx - 1)

            for i in range(segment_count):
                idx = self.current_waypoint_idx + i
                if idx >= len(self.path.poses) - 2:
                    break

                p1 = self.path.poses[idx].pose.position
                p2 = self.path.poses[idx + 1].pose.position
                p3 = self.path.poses[idx + 2].pose.position

                # Calculate curvature
                v1 = np.array([p2.x - p1.x, p2.y - p1.y])
                v2 = np.array([p3.x - p2.x, p3.y - p2.y])

                if np.linalg.norm(v1) > 0.1 and np.linalg.norm(v2) > 0.1:
                    v1_norm = v1 / np.linalg.norm(v1)
                    v2_norm = v2 / np.linalg.norm(v2)
                    cos_angle = np.dot(v1_norm, v2_norm)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.arccos(cos_angle)
                    curvature_sum += angle

            avg_curvature = curvature_sum / segment_count if segment_count > 0 else 0.0

            # Reduce speed for high curvature
            if avg_curvature > math.radians(15):
                curvature_factor = 1.0 - min(0.5, avg_curvature / math.radians(90))
                adjusted_speed *= curvature_factor

        # Ensure minimum speed
        adjusted_speed = max(adjusted_speed, self.min_linear_vel)

        return adjusted_speed

    def apply_motion_limits(self, v, w):
        """Apply acceleration and deceleration limits"""
        max_accel_v = self.acceleration_limit * (1.0/self.control_frequency)
        max_decel_v = self.deceleration_limit * (1.0/self.control_frequency)
        max_accel_w = self.acceleration_limit * 2.0 * (1.0/self.control_frequency)

        delta_v = v - self.last_cmd[0]

        # Use different limits for acceleration vs deceleration
        if delta_v > 0:
            max_delta_v = max_accel_v
        else:
            max_delta_v = max_decel_v

        if abs(delta_v) > max_delta_v:
            v = self.last_cmd[0] + math.copysign(max_delta_v, delta_v)

        delta_w = w - self.last_cmd[1]
        if abs(delta_w) > max_accel_w:
            w = self.last_cmd[1] + math.copysign(max_accel_w, delta_w)

        return v, w

    def enhanced_fallback_control(self, target_waypoint, current_x, current_y, current_yaw):
        """Enhanced fallback control with obstacle avoidance"""
        target_x = target_waypoint.x
        target_y = target_waypoint.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)

        angle_diff = angle_to_target - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        v = 0.0
        w = 0.0

        if distance > self.waypoint_tolerance:
            if self.obstacle_detected:
                # Enhanced obstacle avoidance
                if self.obstacle_distance < self.emergency_stop_distance * 1.5:
                    # Too close - rotate away from obstacle
                    if self.obstacle_direction > 0:
                        w = -self.max_angular_vel * 0.8  # Turn away from obstacle
                    else:
                        w = self.max_angular_vel * 0.8
                    v = self.min_linear_vel * 0.2  # Very slow forward

                    if self.debug_mode:
                        self.get_logger().info(f"🚧 Emergency avoidance: turning away from obstacle")

                else:
                    # Obstacle in path - navigate around
                    avoidance_angle = self.calculate_avoidance_angle()
                    w = avoidance_angle * 2.0
                    v = min(self.max_linear_vel * 0.5, distance * 0.5)

            else:
                # Path is clear - go to goal
                if abs(angle_diff) > math.radians(45):
                    # Large angle difference - turn in place
                    w = self.max_angular_vel * 0.6 * (1.0 if angle_diff > 0 else -1.0)
                    v = self.min_linear_vel * 0.3
                elif abs(angle_diff) > math.radians(15):
                    # Moderate turn
                    v = min(self.max_linear_vel * 0.7, distance * 0.8)
                    w = angle_diff * 1.5
                else:
                    # Nearly aligned
                    v = min(self.max_linear_vel, distance * 1.5)
                    w = angle_diff * 2.0

        return v, w

    def calculate_avoidance_angle(self):
        """Calculate optimal angle to avoid obstacles"""
        if self.scan_ranges is None or self.scan_angles is None:
            return 0.0

        # Find best direction with most clearance
        best_angle = 0.0
        best_clearance = 0.0

        for i, (range_val, angle) in enumerate(zip(self.scan_ranges, self.scan_angles)):
            if range_val > best_clearance:
                best_clearance = range_val
                best_angle = angle

        # Adjust based on target direction
        if self.path is not None and self.current_waypoint_idx < len(self.path.poses):
            target = self.path.poses[self.current_waypoint_idx].pose.position
            dx = target.x - self.odom.pose.pose.position.x
            dy = target.y - self.odom.pose.pose.position.y
            target_angle = math.atan2(dy, dx) - self.quat_to_yaw(self.odom.pose.pose.orientation)

            # Blend obstacle avoidance with target direction
            blend_factor = min(1.0, best_clearance / self.safe_distance)
            avoidance_angle = best_angle * blend_factor + target_angle * (1.0 - blend_factor)
        else:
            avoidance_angle = best_angle

        return avoidance_angle

    def should_emergency_stop(self):
        """Check if emergency stop is needed"""
        if self.obstacle_detected and self.obstacle_distance < self.emergency_stop_distance:
            return True

        # Check for persistent obstacles
        if self.consecutive_obstacle_detections > 20:
            current_time = self.get_clock().now()
            time_since_safe = (current_time - self.last_safe_time).nanoseconds * 1e-9
            if time_since_safe > 5.0:
                self.get_logger().warn(f"Stuck for {time_since_safe:.1f}s - emergency stop")
                return True

        return False

    def should_move(self):
        """Simplified movement decision - ALWAYS try to move if we have a path"""
        # Always return True if we have a valid path
        # The DWA planner will handle obstacle avoidance
        if self.path is None or not self.path_received:
            if self.debug_mode and self.control_counter % 50 == 0:
                self.get_logger().debug("No active path")
            return False

        # Emergency stop check only
        if self.obstacle_detected and self.obstacle_distance < self.emergency_stop_distance:
            if self.debug_mode:
                self.get_logger().warn(f"Emergency: obstacle at {self.obstacle_distance:.2f}m")
            return False

        return True

    def is_robot_stuck(self):
        """Check if robot is stuck with more reasonable conditions"""
        if self.last_position is None or self.odom is None:
            return False

        current_time = self.get_clock().now()
        time_since_safe = (current_time - self.last_safe_time).nanoseconds * 1e-9

        # Check if we've been near obstacles for too long
        if time_since_safe > self.stuck_time:
            return True

        # Check if we're not making progress - MORE LENIENT CONDITION
        # Only check after significant time has passed and we should have moved
        if self.control_counter > 500:  # Increased from 100
            # Only check distance if we've been trying to move for a while
            if self.distance_traveled < 0.5:  # Reduced from 2.0 meters
                return True

        return False

    def update_current_waypoint(self, current_x, current_y):
        """Update current waypoint index with hysteresis"""
        if self.path is None or self.current_waypoint_idx >= len(self.path.poses):
            return

        waypoint = self.path.poses[self.current_waypoint_idx].pose.position
        distance = math.hypot(waypoint.x - current_x, waypoint.y - current_y)

        # Use hysteresis to prevent oscillation
        if distance < self.waypoint_tolerance * 0.8:  # 80% of tolerance
            old_idx = self.current_waypoint_idx
            self.current_waypoint_idx += 1

            if self.debug_mode:
                self.get_logger().info(f"✅ Reached waypoint {old_idx} -> {self.current_waypoint_idx}")

            if self.current_waypoint_idx >= len(self.path.poses):
                self.get_logger().info("🎯 Final waypoint reached!")

    def is_path_valid(self):
        """Check if current path is valid"""
        if self.path is None:
            return False

        if not self.path_received:
            return False

        current_time = self.get_clock().now()
        time_since_update = (current_time - self.last_path_update).nanoseconds * 1e-9

        if time_since_update > self.path_timeout:
            if self.debug_mode:
                self.get_logger().warn(f"⏰ Path expired ({time_since_update:.1f}s > {self.path_timeout}s)")
            self.path_received = False
            self.has_explicit_goal = False
            return False

        return True

    def update_statistics(self, current_speed):
        """Update performance statistics"""
        self.speed_history.append(current_speed)
        if len(self.speed_history) > 50:
            self.speed_history.pop(0)

        if len(self.speed_history) > 0:
            self.avg_speed = sum(self.speed_history) / len(self.speed_history)

        # Count obstacle encounters
        if self.obstacle_detected and self.obstacle_distance < self.safe_distance:
            if not hasattr(self, 'last_obstacle_count_time') or \
               (self.get_clock().now().nanoseconds - getattr(self, 'last_obstacle_count_time', 0)) > 2e9:
                self.obstacle_encounters += 1
                self.last_obstacle_count_time = self.get_clock().now().nanoseconds

        # Publish debug info
        if self.debug_mode and self.control_counter % 20 == 0:
            # Publish safety info
            safety_msg = Twist()
            safety_msg.linear.x = self.obstacle_distance if self.obstacle_detected else 10.0
            safety_msg.linear.y = self.avg_speed
            safety_msg.angular.z = self.obstacle_encounters
            self.safety_pub.publish(safety_msg)

    def publish_cmd(self, linear_vel, angular_vel):
        """Publish velocity command with safety checks"""
        msg = Twist()

        # Smooth very small velocities to zero
        if abs(linear_vel) < 0.01:
            linear_vel = 0.0
        if abs(angular_vel) < 0.02:
            angular_vel = 0.0

        # Final safety check
        if self.obstacle_detected and self.obstacle_distance < self.emergency_stop_distance:
            linear_vel = 0.0
            angular_vel = 0.0

        msg.linear.x = float(linear_vel)
        msg.angular.z = float(angular_vel)

        # Store for acceleration limiting
        self.last_cmd = (linear_vel, angular_vel)

        if self.debug_mode and self.control_counter % 10 == 0:
            if linear_vel > 0.1 or abs(angular_vel) > 0.2:
                obstacle_info = ""
                if self.obstacle_detected:
                    obstacle_info = f", obstacle at {self.obstacle_distance:.2f}m"
                self.get_logger().info(f"📤 Command: v={linear_vel:.3f}, w={angular_vel:.3f}{obstacle_info}")

        self.cmd_pub.publish(msg)

    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Enhanced controller shutting down...")
    except Exception as e:
        node.get_logger().error(f"💥 Controller error: {str(e)}")
    finally:
        # Send stop command
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
