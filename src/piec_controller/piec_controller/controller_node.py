#!/usr/bin/env python3
"""
Complete PIEC Controller with Enhanced Obstacle Recovery
IMPROVED: Better free space utilization and reduced stuck conditions
"""

import rclpy
import math
import numpy as np
import yaml
import os
import csv
import time
from collections import deque
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray  # ADDED THIS IMPORT
from std_msgs.msg import String           # for /emergency_stop/reason
# Try to import the correct PINN service
try:
    from piec_pinn_surrogate_msgs.srv import EvaluateTrajectory
    PINN_SERVICE_AVAILABLE = True
    PINN_SERVICE_TYPE = EvaluateTrajectory
except ImportError:
    PINN_SERVICE_AVAILABLE = False
    class EvaluateTrajectory:
        class Request:
            def __init__(self):
                self.xs = []
                self.ys = []
                self.yaws = []
                self.velocities = []
        class Response:
            def __init__(self):
                self.energy = 0.0
                self.stability = 0.0
    PINN_SERVICE_TYPE = EvaluateTrajectory

# Import the DWA planner
try:
    from .dynamic_dwa_complete import DynamicDWAComplete
    DWA_AVAILABLE = True
except ImportError:
    DWA_AVAILABLE = False
    class DynamicDWAComplete:
        def __init__(self, node, emergency_distance=0.2):
            self.node = node
        
        def plan(self, current_pose, path, scan_ranges, scan_angles):
            return 0.0, 0.0


class ControllerNode(Node):
    def __init__(self):
        super().__init__('enhanced_piec_controller')
        
        # REAL ROBOT CALIBRATION PARAMETERS
        # These parameters correct for hardware-specific discrepancies
        
        # linear_scale_factor: Scales commanded linear velocity (default: 1.0)
        # Use values < 1.0 to reduce actual speed if robot moves too fast
        self.declare_parameter('linear_scale_factor', 1.0)
        
        # angular_scale_factor: Scales commanded angular velocity (default: 1.0)
        # Use values < 1.0 to reduce actual turning speed
        self.declare_parameter('angular_scale_factor', 1.0)
        
        # angular_sign_correction: Corrects angular velocity sign convention (default: 1.0)
        # Standard ROS: +w = counter-clockwise (CCW), -w = clockwise (CW)
        # Scout Mini follows standard ROS convention: +w = CCW (left turn), -w = CW (right turn)
        # Set to 1.0 for standard ROS convention (including Scout Mini)
        # Set to -1.0 only if your robot has inverted angular velocity
        self.declare_parameter('angular_sign_correction', 1.0)
        
        self.linear_scale = self.get_parameter('linear_scale_factor').value
        self.angular_scale = self.get_parameter('angular_scale_factor').value
        self.angular_sign = self.get_parameter('angular_sign_correction').value
        
        # Load parameters from YAML file
        self.load_parameters_from_yaml()
       # --- FIX: ensure use_dwa is defined ---
        if not hasattr(self, 'use_dwa'):
            self.use_dwa = self.get_parameter('use_dwa').value
        # Initialize state variables
        self.path = None
        self.odom = None
        self.scan_ranges = []
        self.scan_angles = []
        self.scan_time = None
        self.current_waypoint_idx = 0
        self.last_path_update = self.get_clock().now()
        self.path_received = False

        # Goal tracking
        self.has_explicit_goal = False
        self.last_explicit_goal = None
        self.explicit_goal_timeout = 30.0
        self.goal_reached = False
        self.goal_completion_distance = 0.40  # Increased to 40cm for more reliable completion
        self.goal_position = None
        self.goal_reached_time = None
        self.goal_stopped = False
        self.goal_stable_start_time = None  # Track when robot became stable at goal
        self.last_goal_distance = None  # Track distance history for stability check

        # Enhanced obstacle tracking with 360-degree awareness
        self.obstacle_map = {}
        self.map_resolution = 0.1
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.obstacle_direction = 0.0
        self.obstacle_type = 'static'
        self.consecutive_obstacle_detections = 0
        
        # Free space tracking
        self.free_space_directions = []  # Available directions with clearance
        self.last_free_space_update = time.monotonic()

        # Statistics
        self.distance_traveled = 0.0
        self.last_position = None
        self.control_counter = 0
        self.avg_speed = 0.0
        self.speed_history = []
        self.obstacle_encounters = 0

        # IMPROVED Stuck detection - less sensitive, more intelligent
        self.position_history = deque(maxlen=30)
        self.stuck_detected = False
        self.stuck_start_time = None
        self.stuck_threshold_distance = 0.08  # Slightly increased
        self.stuck_threshold_time = 10.0  # Increased to reduce false positives
        self.recovery_mode = False
        self.recovery_start_time = None
        self.recovery_duration = 4.0
        self.recovery_phase = None
        self.backup_distance = 0.0
        self.max_backup_distance = 1.0
        
        # Improved obstacle presence tracking
        self.obstacle_present_when_stuck = False
        self.consecutive_obstacle_readings = 0
        self.last_obstacle_check_time = time.monotonic()
        
        # Enhanced escape tracking
        self.escape_directions_tried = []
        self.last_escape_direction = None
        self.escape_success_count = 0
        
        # Rotation timeout tracking to prevent infinite spinning
        self.rotation_start_time = None
        self.is_rotating_in_place = False  # Flag to track when rotating in place
        self.free_space_escape_preferred = True
        self.last_rotation_direction = None  # Track last rotation direction to prevent oscillation
        self.rotation_direction_lock_time = None  # Lock direction for stability
        
        # Oscillation detection - NEW
        self.angular_history = deque(maxlen=30)  # Increased from 20 to 30
        self.linear_history = deque(maxlen=30)   # Increased from 20 to 30
        self.recovery_count = 0
        self.recovery_stage = 0
        self.goal_received_time = None  # Track when goal was received

        # Reactive speed scaling state (zone hysteresis and approach velocity)
        self._reactive_prev_obstacle_dist = float('inf')
        self._reactive_in_risk_zone = False
        self._reactive_in_safety_zone = False

        # Motor health monitoring
        self.commanded_velocity_history = deque(maxlen=20)
        self.actual_velocity_history = deque(maxlen=20)
        self.motor_responsiveness = 100.0
        self.last_commanded_linear = 0.0
        self.last_actual_linear = 0.0
        self.motor_unresponsive_count = 0

        # Straight-line navigation constants
        self.STRAIGHT_LINE_ANGLE_THRESHOLD = math.radians(3)  # 3 degrees for simple control

        # DWA planner
        self.dwa_failures = 0
        self.max_dwa_failures = 5
        self.simple_control_active = False
        self.last_cmd = (0.0, 0.0)
        # Add this in controller_node.py __init__ method
        self.pinn_client = None
        if self.initialize_pinn_for_controller():
            self.get_logger().info("PINN optimization enabled for speed control")
        # Initialize DWA Planner if available
        if DWA_AVAILABLE and self.use_dwa:
            try:
                self.dwa = DynamicDWAComplete(self, emergency_distance=self.emergency_stop_distance)
                self.use_dwa = True
                self.get_logger().info("✅ DWA planner initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize DWA: {e}")
                self.use_dwa = False
        else:
            self.use_dwa = False

        # ── Diagnostics CSV logger ────────────────────────────────────────────
        self._csv_log_path = os.path.expanduser('~/piec_data/controller_anomalies.csv')
        self._csv_file = None
        self._csv_writer = None
        self._init_csv_logger()
        # Track last emergency stop reason received from the e-stop node
        self._last_estop_reason = ''

        # QoS Settings
        scan_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscribers
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(Odometry, '/ukf/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        # Subscribe to emergency stop reason for CSV logging
        self.create_subscription(
            String, '/emergency_stop/reason',
            self._estop_reason_callback, 10
        )

        # Publishers
        # Use cmd_vel_topic parameter if provided, otherwise default to /cmd_vel_piec
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_piec')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.get_logger().info(f"Publishing commands to: {cmd_vel_topic}")
        
        # Goal status publisher
        goal_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.goal_status_pub = self.create_publisher(PoseStamped, '/goal_status', goal_qos)

        if self.debug_mode:
            self.target_pub = self.create_publisher(PoseStamped, '/debug/target', 10)
            self.free_space_pub = self.create_publisher(PoseArray, '/debug/available_directions', 10)

        # Control Timer
        control_freq = self.control_frequency
        #self.create_timer(1.0/control_freq, self.control_loop)
        self.create_timer(1.0/control_freq, self.control_loop_with_pinn_optimization)

        # Stuck check timer
        self.create_timer(1.5, self.check_stuck_condition)  # Less frequent checking

        # Free space update timer
        self.create_timer(0.3, self.update_free_space_directions)

        self.get_logger().info("🚀 Enhanced PIEC Controller READY with Improved Free Space Navigation")
        self.get_logger().info(f"Stuck detection: {self.stuck_threshold_distance}m movement in {self.stuck_threshold_time}s")
        self.get_logger().info("🛑 Emergency Stop node started, output to /cmd_vel")
    def load_parameters_from_yaml(self):
        """Load parameters from YAML configuration file"""
        config_path = os.path.join(
            get_package_share_directory('piec_controller'),
            'config',
            'controller_params.yaml'
        )

        # Default parameters
        params = {
            # Speed Parameters
            'max_linear_vel': 1.0,
            'max_angular_vel': 0.7,
            'min_linear_vel': 0.05,
            'acceleration_limit': 0.8,
            'deceleration_limit': 1.2,

            # Safety Parameters
            'emergency_stop_distance': 0.60,
            'slow_down_distance': 0.90,
            'safe_distance': 1.50,
            'obstacle_clearance': 0.5,
            'lateral_safety_margin': 0.4,

            # Path Following
            'waypoint_tolerance': 0.3,
            'path_timeout': 20.0,
            'lookahead_distance': 0.8,
            'adaptive_lookahead': True,
            'path_replan_distance': 1.0,

            # Recovery Parameters
            'recovery_timeout': 5.0,
            'stuck_threshold': 1.5,
            'stuck_time': 8.0,
            'enable_stuck_recovery': True,
            'max_backup_distance': 1.2,
            'recovery_turn_angle': 75.0,

            # DWA Parameters
            'dwa_max_v': 1.0,
            'dwa_min_v': 0.05,
            'dwa_max_w': 0.7,
            'dwa_sim_time': 1.5,
            'dwa_dt': 0.04,
            'dwa_clearance_weight': 3.0,

            # Control Parameters
            'control_frequency': 20.0,
            'debug_mode': True,
            'require_explicit_goal': False,
            'enable_obstacle_memory': False,
            
            # Goal completion parameters
            'goal_completion_distance': 0.25,
            'goal_completion_angular_tolerance': 0.15,
            'min_stop_time': 1.5,
            'goal_stability_time': 2.0,
            
            # Heading control parameters
            'heading_kp': 1.5,
            'heading_deadband_deg': 2.0,
            'max_heading_rate': 0.6,
            'rotate_in_place_angle_deg': 60.0,
            'rotation_timeout': 5.0,
            'rotation_direction_lock_duration': 2.0,
            'rotation_min_angle_for_lock_deg': 10.0,
            'close_range_distance': 0.5,
            'rotate_free_space_threshold': 1.5,
            
            # Path validation parameters
            'path_staleness_threshold': 1.0,
            'path_staleness_warning_threshold': 0.3,
            
            # Control mode
            'use_dwa': True,
            
            # Stuck detection
            'stuck_threshold_distance': 0.08,
            'stuck_threshold_time': 8.0,
            'recovery_duration': 4.0,
            'obstacle_check_distance': 0.8,
            'min_obstacle_presence': 3,
            
            # Free space parameters
            'free_space_min_clearance': 0.8,
            'free_space_update_rate': 3.0,
            'prefer_free_space_turns': True,
            'use_pinn_in_controller': False,
            'scan_topic': '/scan',
            'path_topic': '/piec/path',

            # Reactive speed scaling parameters
            'reactive_risk_zone_dist': 0.3,
            'reactive_safety_zone_dist': 1.0,
            'reactive_hysteresis_factor': 0.15,
            'reactive_approach_gain': 2.0,
            'reactive_approach_threshold': -0.1,
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
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.min_linear_vel = float(self.get_parameter('min_linear_vel').value)
        self.acceleration_limit = float(self.get_parameter('acceleration_limit').value)
        self.deceleration_limit = float(self.get_parameter('deceleration_limit').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.slow_down_distance = float(self.get_parameter('slow_down_distance').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.obstacle_clearance = float(self.get_parameter('obstacle_clearance').value)
        self.lateral_safety_margin = float(self.get_parameter('lateral_safety_margin').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.path_timeout = float(self.get_parameter('path_timeout').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.adaptive_lookahead = self.get_parameter('adaptive_lookahead').value
        self.path_replan_distance = float(self.get_parameter('path_replan_distance').value)
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.stuck_threshold = float(self.get_parameter('stuck_threshold').value)
        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.enable_stuck_recovery = self.get_parameter('enable_stuck_recovery').value
        self.max_backup_distance = float(self.get_parameter('max_backup_distance').value)
        self.recovery_turn_angle = float(self.get_parameter('recovery_turn_angle').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.debug_mode = self.get_parameter('debug_mode').value
        self.require_explicit_goal = self.get_parameter('require_explicit_goal').value
        
        # Goal completion parameters
        self.goal_completion_distance = float(self.get_parameter('goal_completion_distance').value)
        self.goal_stability_time = float(self.get_parameter('goal_stability_time').value)
        
        # Heading control parameters
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_deadband_deg = float(self.get_parameter('heading_deadband_deg').value)
        self.max_heading_rate = float(self.get_parameter('max_heading_rate').value)
        self.rotate_in_place_angle_deg = float(self.get_parameter('rotate_in_place_angle_deg').value)
        self.rotation_timeout = float(self.get_parameter('rotation_timeout').value)
        self.rotation_direction_lock_duration = float(self.get_parameter('rotation_direction_lock_duration').value)
        self.rotation_min_angle_for_lock_deg = float(self.get_parameter('rotation_min_angle_for_lock_deg').value)
        self.close_range_distance = float(self.get_parameter('close_range_distance').value)
        self.rotate_free_space_threshold = float(self.get_parameter('rotate_free_space_threshold').value)
        
        # Path validation parameters
        self.path_staleness_threshold = float(self.get_parameter('path_staleness_threshold').value)
        self.path_staleness_warning_threshold = float(self.get_parameter('path_staleness_warning_threshold').value)
        
        # Control mode
        self.use_dwa = self.get_parameter('use_dwa').value
        
        # Stuck detection
        self.stuck_threshold_distance = float(self.get_parameter('stuck_threshold_distance').value)
        self.stuck_threshold_time = float(self.get_parameter('stuck_threshold_time').value)
        self.recovery_duration = float(self.get_parameter('recovery_duration').value)
        self.obstacle_check_distance = float(self.get_parameter('obstacle_check_distance').value)
        self.min_obstacle_presence = int(self.get_parameter('min_obstacle_presence').value)
        
        # Free space parameters
        self.free_space_min_clearance = float(self.get_parameter('free_space_min_clearance').value)
        self.free_space_update_rate = float(self.get_parameter('free_space_update_rate').value)
        self.prefer_free_space_turns = self.get_parameter('prefer_free_space_turns').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        # Reactive speed scaling parameters
        self.reactive_risk_zone_dist = float(self.get_parameter('reactive_risk_zone_dist').value)
        self.reactive_safety_zone_dist = float(self.get_parameter('reactive_safety_zone_dist').value)
        self.reactive_hysteresis_factor = float(self.get_parameter('reactive_hysteresis_factor').value)
        self.reactive_approach_gain = float(self.get_parameter('reactive_approach_gain').value)
        self.reactive_approach_threshold = float(self.get_parameter('reactive_approach_threshold').value)
    # ── CSV Diagnostics Logger ────────────────────────────────────────────────

    def _init_csv_logger(self):
        """Initialise the CSV anomaly log file."""
        try:
            log_dir = os.path.dirname(self._csv_log_path)
            os.makedirs(log_dir, exist_ok=True)
            self._csv_file = open(self._csv_log_path, 'a', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            # Write header only when the file is new / empty
            if os.path.getsize(self._csv_log_path) == 0:
                self._csv_writer.writerow([
                    'timestamp', 'robot_x', 'robot_y', 'robot_yaw_deg',
                    'goal_x', 'goal_y', 'goal_dist_m',
                    'heading_err_deg', 'clearance_m',
                    'v_cmd', 'w_cmd',
                    'estop_reason', 'anomaly_type',
                ])
            self.get_logger().info(f'📝 Anomaly CSV logger: {self._csv_log_path}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Could not open CSV logger: {e}')
            self._csv_writer = None

    def _log_anomaly(self, anomaly_type: str, v_cmd: float, w_cmd: float,
                     heading_err_deg: float, clearance_m: float):
        """Write one row to the anomaly CSV log."""
        if self._csv_writer is None:
            return
        try:
            robot_x = robot_y = robot_yaw_deg = 0.0
            goal_x = goal_y = goal_dist = 0.0
            if self.odom is not None:
                robot_x = self.odom.pose.pose.position.x
                robot_y = self.odom.pose.pose.position.y
                robot_yaw_deg = math.degrees(self.quat_to_yaw(self.odom.pose.pose.orientation))
            if self.goal_position is not None:
                goal_x, goal_y = self.goal_position
                goal_dist = math.hypot(goal_x - robot_x, goal_y - robot_y)
            self._csv_writer.writerow([
                time.strftime('%Y-%m-%dT%H:%M:%S'),
                f'{robot_x:.4f}', f'{robot_y:.4f}', f'{robot_yaw_deg:.2f}',
                f'{goal_x:.4f}', f'{goal_y:.4f}', f'{goal_dist:.3f}',
                f'{heading_err_deg:.2f}', f'{clearance_m:.3f}',
                f'{v_cmd:.4f}', f'{w_cmd:.4f}',
                self._last_estop_reason, anomaly_type,
            ])
            self._csv_file.flush()
        except Exception as e:
            self.get_logger().warn(f'CSV write error: {e}')

    def _estop_reason_callback(self, msg: String):
        """Store latest emergency stop reason for CSV logging."""
        self._last_estop_reason = msg.data
        if self.odom is not None:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            heading_err = 0.0
            if self.goal_position is not None:
                gx, gy = self.goal_position
                goal_dir = math.atan2(gy - y, gx - x)
                heading_err = math.degrees(abs(math.atan2(
                    math.sin(goal_dir - yaw), math.cos(goal_dir - yaw))))
            self._log_anomaly(
                'EMERGENCY_STOP', 0.0, 0.0, heading_err,
                self.obstacle_distance,
            )
        """Load parameters from YAML configuration file"""
        config_path = os.path.join(
            get_package_share_directory('piec_controller'),
            'config',
            'controller_params.yaml'
        )

        # Default parameters
        params = {
            # Speed Parameters
            'max_linear_vel': 1.0,
            'max_angular_vel': 0.7,
            'min_linear_vel': 0.05,
            'acceleration_limit': 2.0,
            'deceleration_limit': 2.0,

            # Safety Parameters
            'emergency_stop_distance': 0.60,
            'slow_down_distance': 0.90,  # Increased
            'safe_distance': 1.50,  # Increased
            'obstacle_clearance': 0.5,  # Increased
#we can use the equations (L+W)/2pir
            'lateral_safety_margin': 0.4,  # Increased change

            # Path Following
            'waypoint_tolerance': 0.3,  # Increased
            'path_timeout': 20.0,
            'lookahead_distance': 0.8,  # Increased
            'adaptive_lookahead': True,
            'path_replan_distance': 1.0,  # Increased

            # Recovery Parameters
            'recovery_timeout': 5.0,  # Increased
            'stuck_threshold': 1.5,
            'stuck_time': 8.0,  # Increased
            'enable_stuck_recovery': True,
            'max_backup_distance': 1.2,  # Increased
            'recovery_turn_angle': 75.0,  # Increased

            # DWA Parameters
            'dwa_max_v': 1.0,
            'dwa_min_v': 0.05,
            'dwa_max_w': 0.7,
            'dwa_sim_time': 1.5,  # Increased
            'dwa_dt': 0.04,
            'dwa_clearance_weight': 3.0,  # Increased

            # Control Parameters
            'control_frequency': 20.0,
            'debug_mode': True,
            'require_explicit_goal': False,  # Allow path following without explicit goal
            'enable_obstacle_memory': False,
            
            # Goal completion parameters
            'goal_completion_distance': 0.25,
            'goal_completion_angular_tolerance': 0.15,
            'min_stop_time': 1.5,
            'goal_stability_time': 2.0,  # Time robot must be stable at goal
            
            # Heading control parameters
            'heading_kp': 1.5,  # Proportional gain for heading control
            'heading_deadband_deg': 2.0,  # Deadband for small angle errors (degrees)
            'max_heading_rate': 0.6,  # Maximum angular velocity for heading control (rad/s)
            'rotate_in_place_angle_deg': 60.0,  # Rotate in place if angle error > this (degrees) - lowered to prevent oscillation
            'rotation_timeout': 5.0,  # Maximum time to spend rotating in place (seconds) - REDUCED from 10s to prevent long spinning
            'rotation_direction_lock_duration': 2.0,  # Time (seconds) to lock rotation direction to prevent oscillation
            'rotation_min_angle_for_lock_deg': 10.0,  # Minimum angle error (degrees) to maintain locked direction
            'close_range_distance': 0.5,  # Distance threshold for close-range proportional control (meters) - INCREASED from 0.3 to allow smoother approach
            'rotate_free_space_threshold': 1.5,  # Min clearance (m) to use combined forward+turn instead of rotate-in-place
            
            # Path validation parameters
            'path_staleness_threshold': 1.0,  # Increased: accept paths up to 1m from robot position
            'path_staleness_warning_threshold': 0.3,  # Warning threshold for path start deviation (meters)
            
            # Control mode
            'use_dwa': True,
            
            # Stuck detection
            'stuck_threshold_distance': 0.08,
            'stuck_threshold_time': 8.0,
            'recovery_duration': 4.0,
            'obstacle_check_distance': 0.8,  # Increased
            'min_obstacle_presence': 3,  # Increased
            
            # Free space parameters
            'free_space_min_clearance': 0.8,  # Minimum clearance for free space
            'free_space_update_rate': 3.0,  # Hz
            'prefer_free_space_turns': True,
            'use_pinn_in_controller': False,
            'scan_topic': '/scan',
            'path_topic': '/piec/path',

            # Reactive speed scaling parameters
            'reactive_risk_zone_dist': 0.3,     # m - stop linear velocity in this zone
            'reactive_safety_zone_dist': 1.0,   # m - scale linearly in this zone
            'reactive_hysteresis_factor': 0.15, # fraction - exit threshold = dist * (1 + factor)
            'reactive_approach_gain': 2.0,      # gain applied to closing velocity for extra slowdown
            'reactive_approach_threshold': -0.1, # m/s - closing speed below which approach_factor is applied
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
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.min_linear_vel = float(self.get_parameter('min_linear_vel').value)
        self.acceleration_limit = float(self.get_parameter('acceleration_limit').value)
        self.deceleration_limit = float(self.get_parameter('deceleration_limit').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.slow_down_distance = float(self.get_parameter('slow_down_distance').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.obstacle_clearance = float(self.get_parameter('obstacle_clearance').value)
        self.lateral_safety_margin = float(self.get_parameter('lateral_safety_margin').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.path_timeout = float(self.get_parameter('path_timeout').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.adaptive_lookahead = self.get_parameter('adaptive_lookahead').value
        self.path_replan_distance = float(self.get_parameter('path_replan_distance').value)
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.stuck_threshold = float(self.get_parameter('stuck_threshold').value)
        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.enable_stuck_recovery = self.get_parameter('enable_stuck_recovery').value
        self.max_backup_distance = float(self.get_parameter('max_backup_distance').value)
        self.recovery_turn_angle = float(self.get_parameter('recovery_turn_angle').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.debug_mode = self.get_parameter('debug_mode').value
        self.require_explicit_goal = self.get_parameter('require_explicit_goal').value
        
        # Goal completion parameters
        self.goal_completion_distance = float(self.get_parameter('goal_completion_distance').value)
        self.goal_stability_time = float(self.get_parameter('goal_stability_time').value)
        
        # Heading control parameters
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_deadband_deg = float(self.get_parameter('heading_deadband_deg').value)
        self.max_heading_rate = float(self.get_parameter('max_heading_rate').value)
        self.rotate_in_place_angle_deg = float(self.get_parameter('rotate_in_place_angle_deg').value)
        self.rotation_timeout = float(self.get_parameter('rotation_timeout').value)
        self.rotation_direction_lock_duration = float(self.get_parameter('rotation_direction_lock_duration').value)
        self.rotation_min_angle_for_lock_deg = float(self.get_parameter('rotation_min_angle_for_lock_deg').value)
        self.close_range_distance = float(self.get_parameter('close_range_distance').value)
        self.rotate_free_space_threshold = float(self.get_parameter('rotate_free_space_threshold').value)
        
        # Path validation parameters
        self.path_staleness_threshold = float(self.get_parameter('path_staleness_threshold').value)
        self.path_staleness_warning_threshold = float(self.get_parameter('path_staleness_warning_threshold').value)
        
        # Control mode
        self.use_dwa = self.get_parameter('use_dwa').value
        
        # Stuck detection
        self.stuck_threshold_distance = float(self.get_parameter('stuck_threshold_distance').value)
        self.stuck_threshold_time = float(self.get_parameter('stuck_threshold_time').value)
        self.recovery_duration = float(self.get_parameter('recovery_duration').value)
        self.obstacle_check_distance = float(self.get_parameter('obstacle_check_distance').value)
        self.min_obstacle_presence = int(self.get_parameter('min_obstacle_presence').value)
        
        # Free space parameters
        self.free_space_min_clearance = float(self.get_parameter('free_space_min_clearance').value)
        self.free_space_update_rate = float(self.get_parameter('free_space_update_rate').value)
        self.prefer_free_space_turns = self.get_parameter('prefer_free_space_turns').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        # Reactive speed scaling parameters
        self.reactive_risk_zone_dist = float(self.get_parameter('reactive_risk_zone_dist').value)
        self.reactive_safety_zone_dist = float(self.get_parameter('reactive_safety_zone_dist').value)
        self.reactive_hysteresis_factor = float(self.get_parameter('reactive_hysteresis_factor').value)
        self.reactive_approach_gain = float(self.get_parameter('reactive_approach_gain').value)
        self.reactive_approach_threshold = float(self.get_parameter('reactive_approach_threshold').value)

    def update_free_space_directions(self):
        """Update available free space directions"""
        if not self.scan_ranges or not self.scan_angles:
            self.free_space_directions = []
            return
        
        current_time = time.monotonic()
        if current_time - self.last_free_space_update < 1.0 / self.free_space_update_rate:
            return
        
        self.last_free_space_update = current_time
        self.free_space_directions = []
        
        # Check 24 directions around the robot (15-degree increments)
        for angle in np.arange(0, 2 * math.pi, math.pi / 12):
            clearance = self.get_clearance_in_direction(angle)
            
            if clearance >= self.free_space_min_clearance:
                # Calculate score based on clearance and alignment with current goal
                score = clearance * 0.8
                
                if self.goal_position and self.odom:
                    current_x = self.odom.pose.pose.position.x
                    current_y = self.odom.pose.pose.position.y
                    goal_x, goal_y = self.goal_position
                    goal_dir = math.atan2(goal_y - current_y, goal_x - current_x)
                    
                    angle_diff = abs(math.atan2(math.sin(angle - goal_dir), math.cos(angle - goal_dir)))
                    alignment_score = 1.0 - (angle_diff / math.pi)
                    score += alignment_score * 0.2
                
                self.free_space_directions.append((angle, clearance, score))
        
        # Sort by score (highest first)
        self.free_space_directions.sort(key=lambda x: x[2], reverse=True)
        
        # Publish for debugging
        if self.debug_mode and hasattr(self, 'free_space_pub'):
            self.publish_free_space_directions()

    def publish_free_space_directions(self):
        """Publish free space directions for visualization"""
        pose_array = PoseArray()
        pose_array.header.frame_id = 'odom'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        if self.odom and self.free_space_directions:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            
            for angle, clearance, score in self.free_space_directions[:8]:  # Top 8 directions
                pose = Pose()
                pose.position.x = x + clearance * 0.5 * math.cos(angle)
                pose.position.y = y + clearance * 0.5 * math.sin(angle)
                pose.position.z = 0.0
                pose_array.poses.append(pose)
        
        self.free_space_pub.publish(pose_array)

    def get_clearance_in_direction(self, angle):
        """Get clearance in a specific direction using laser scan"""
        if not self.scan_ranges or not self.scan_angles:
            return 0.0
        
        # Find the laser reading closest to this angle
        min_diff = float('inf')
        closest_range = 10.0  # Default large range
        
        for i, scan_angle in enumerate(self.scan_angles):
            angle_diff = abs(math.atan2(math.sin(scan_angle - angle), math.cos(scan_angle - angle)))
            if angle_diff < min_diff and i < len(self.scan_ranges):
                min_diff = angle_diff
                closest_range = self.scan_ranges[i]
        
        # Return valid range (filter out NaN/inf)
        if math.isfinite(closest_range) and closest_range > 0.1:
            return min(closest_range, 5.0)
        
        return 0.0

    def goal_callback(self, msg: PoseStamped):
        """Handle explicit goal messages"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        self.has_explicit_goal = True
        self.last_explicit_goal = (goal_x, goal_y)
        self.last_goal_update = self.get_clock().now()
        self.goal_received_time = time.monotonic()  # ADD THIS - track when goal was received
        
        # Reset goal completion flags when new goal received
        self.goal_reached = False
        self.goal_stopped = False
        self.goal_reached_time = None
        self.goal_position = (goal_x, goal_y)
        self.path = None
        self.path_received = False
        
        # Reset stuck detection - CRITICAL
        self.stuck_detected = False
        self.recovery_mode = False
        self.position_history.clear()
        self.consecutive_obstacle_readings = 0
        self.escape_directions_tried.clear()
        self.last_escape_direction = None
        self.free_space_escape_preferred = True
        
        # Reset oscillation tracking
        if hasattr(self, 'angular_history'):
            self.angular_history.clear()
        if hasattr(self, 'linear_history'):
            self.linear_history.clear()
        
        # Reset motor health history for new goal
        if hasattr(self, 'commanded_velocity_history'):
            self.commanded_velocity_history.clear()
        if hasattr(self, 'actual_velocity_history'):
            self.actual_velocity_history.clear()
        if hasattr(self, 'motor_unresponsive_count'):
            self.motor_unresponsive_count = 0
        
        # Reset recovery counter
        self.recovery_count = 0

        if self.debug_mode:
            self.get_logger().info(f"🎯 Received explicit goal: ({goal_x:.3f}, {goal_y:.3f})")

    def odom_callback(self, msg: Odometry):
        """Update robot pose from UKF"""
        self.odom = msg

        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # Update position history for stuck detection
        current_time = time.monotonic()
        self.position_history.append((current_position, current_time))
        
        # Track actual velocity for motor health monitoring
        if hasattr(msg.twist.twist.linear, 'x'):
            self.last_actual_linear = abs(msg.twist.twist.linear.x)
            self.actual_velocity_history.append(self.last_actual_linear)
            
            # Check motor health if we have commanded velocity
            if len(self.commanded_velocity_history) > 0 and len(self.actual_velocity_history) > 0:
                self.check_motor_health()
        
        # Update distance traveled
        if self.last_position is not None:
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            self.distance_traveled += math.hypot(dx, dy)

        self.last_position = current_position

    def scan_callback(self, msg: LaserScan):
        """Process laser scan data with 360-degree awareness"""
        if len(msg.ranges) == 0:
            self.scan_ranges = []
            return

        self.scan_ranges = list(msg.ranges)
        self.scan_time = self.get_clock().now()

        # Initialize scan angles if needed
        if len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = [
                float(msg.angle_min + i * msg.angle_increment)
                for i in range(len(msg.ranges))
            ]

        # Enhanced obstacle detection with full 360-degree awareness
        self.enhanced_obstacle_detection()

        # Update obstacle presence for stuck detection
        self.update_obstacle_presence()

    def update_obstacle_presence(self):
        """Update obstacle presence tracking with 360-degree awareness"""
        if not self.scan_ranges or not self.scan_angles:
            self.consecutive_obstacle_readings = 0
            return
        
        obstacle_count = 0
        
        # Check entire 360 degrees, but weight front more heavily
        for i, (range_val, angle) in enumerate(zip(self.scan_ranges, self.scan_angles)):
            # Weight based on direction (front has higher weight)
            weight = 1.0 if abs(angle) < math.radians(60) else 0.5
            
            if 0.1 < range_val < self.obstacle_check_distance:
                obstacle_count += weight
        
        # Update consecutive readings
        if obstacle_count > 4:  # Higher threshold
            self.consecutive_obstacle_readings = min(self.consecutive_obstacle_readings + 1, 10)
        else:
            self.consecutive_obstacle_readings = max(self.consecutive_obstacle_readings - 1, 0)

    def check_motor_health(self):
        """Check motor health by comparing commanded vs actual velocity"""
        if len(self.commanded_velocity_history) < 5 or len(self.actual_velocity_history) < 5:
            return
        
        # Get recent commanded and actual velocities
        recent_commanded = list(self.commanded_velocity_history)[-10:]
        recent_actual = list(self.actual_velocity_history)[-10:]
        
        # Calculate average velocities (values are already absolute)
        avg_commanded = np.mean(recent_commanded)
        avg_actual = np.mean(recent_actual)
        
        # Calculate motor responsiveness percentage
        if avg_commanded > 0.05:  # Only check if meaningful command
            self.motor_responsiveness = min(100.0, (avg_actual / avg_commanded) * 100.0)
            
            # Track unresponsive motors
            if self.motor_responsiveness < 30.0:  # Less than 30% responsiveness
                self.motor_unresponsive_count += 1
            else:
                self.motor_unresponsive_count = max(0, self.motor_unresponsive_count - 1)
            
            # Log motor health in debug mode
            if self.debug_mode and self.control_counter % 30 == 0:
                status = "UNRESPONSIVE" if self.motor_responsiveness < 30.0 else "Responsive"
                self.get_logger().info(
                    f"🚗 Motor Health: {status} ({self.motor_responsiveness:.0f}%) - "
                    f"Commanded={avg_commanded:.3f}, Actual={avg_actual:.3f}"
                )
                
                # Warn if motors are unresponsive
                if self.motor_responsiveness < 30.0 and avg_commanded > 0.1:
                    self.get_logger().warn(
                        f"⚠️ Robot speed very low: {avg_actual:.3f} m/s despite command: {avg_commanded:.3f} m/s"
                    )
        else:
            # Reset to healthy if no commands being sent
            self.motor_responsiveness = 100.0
            self.motor_unresponsive_count = max(0, self.motor_unresponsive_count - 1)

    def check_stuck_condition(self):
        """Check if robot is stuck - WITH GRACE PERIOD AND MOTOR HEALTH CHECK"""
        if not self.enable_stuck_recovery or self.goal_stopped or not self.path:
            return
        
        # CRITICAL: Do NOT check stuck if no goal has been received yet
        # This prevents stuck detection when robot is manually locked or before goal
        if not hasattr(self, 'goal_received_time') or self.goal_received_time is None:
            return
        
        # Don't check stuck if motors are unresponsive (hardware issue, not stuck)
        if hasattr(self, 'motor_unresponsive_count') and self.motor_unresponsive_count >= 3:
            if self.debug_mode and self.control_counter % 50 == 0:
                self.get_logger().warn("⚠️ Motors unresponsive - hardware issue detected, skipping recovery")
            return
        
        # Grace period after goal - reduced to 6 seconds for better responsiveness
        time_since_goal = time.monotonic() - self.goal_received_time
        if time_since_goal < 6.0:  # Reduced from 12 to 6 second grace period
            return
        
        # Only check stuck if we have enough position data
        if len(self.position_history) < 12:
            return
        
        # Skip stuck check if robot is already moving well
        if hasattr(self, 'last_actual_linear') and self.last_actual_linear > 0.15:
            # Robot is moving fine, reset stuck detection
            if self.stuck_detected:
                self.stuck_detected = False
                self.stuck_start_time = None
            return
        
        # Check for oscillation first
        is_oscillating = self.detect_oscillation()
        if is_oscillating and not self.recovery_mode:
            self.recovery_count += 1
            
            # Require 3 consecutive detections before recovery
            if self.recovery_count >= 3:
                if self.debug_mode:
                    self.get_logger().warn("🔄 Oscillation confirmed (3 checks) - initiating recovery")
                self.initiate_recovery()
                self.recovery_count = 0
            else:
                if self.debug_mode:
                    self.get_logger().debug(f"⚠️ Possible oscillation ({self.recovery_count}/3)")
            return
        else:
            # Reset counter if not oscillating
            self.recovery_count = max(0, self.recovery_count - 1)
        
        # Check if robot has moved significantly in last N positions
        recent_positions = list(self.position_history)[-12:]
        
        # Calculate max distance in recent positions
        max_distance = 0.0
        for i in range(len(recent_positions)):
            for j in range(i+1, len(recent_positions)):
                pos1, time1 = recent_positions[i]
                pos2, time2 = recent_positions[j]
                dist = math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])
                max_distance = max(max_distance, dist)
        
        # Check if we're stuck
        if max_distance < self.stuck_threshold_distance:
            if not self.stuck_detected:
                self.stuck_detected = True
                self.stuck_start_time = time.monotonic()
                
                if self.debug_mode:
                    self.get_logger().info(f"⚠️ Low movement detected: {max_distance:.3f}m")
            
            # Check if stuck for long enough AND obstacles are present
            current_time = time.monotonic()
            if (self.stuck_detected and
                self.stuck_start_time is not None and
                current_time - self.stuck_start_time > self.stuck_threshold_time and
                self.consecutive_obstacle_readings >= self.min_obstacle_presence and
                not self.recovery_mode):

                # Only initiate recovery if forward path is actually blocked
                if self.has_forward_clearance(min_clearance=1.0):
                    if self.debug_mode:
                        self.get_logger().info(
                            "✅ Forward clear (>1.0m) - skipping recovery, continuing normal control"
                        )
                    self.stuck_detected = False
                    self.stuck_start_time = None
                    return

                self.obstacle_present_when_stuck = True

                # Check if we have free space alternatives before initiating recovery
                if self.free_space_directions and self.free_space_escape_preferred:
                    if self.debug_mode:
                        self.get_logger().info("🔄 Attempting free space escape instead of recovery")
                    self.attempt_free_space_escape()
                else:
                    self.initiate_recovery()
        else:
            # Robot is moving, reset stuck detection
            if self.stuck_detected:
                self.stuck_detected = False
                self.stuck_start_time = None
                self.obstacle_present_when_stuck = False
                if self.debug_mode:
                    self.get_logger().debug("✅ Robot is moving again")

    def detect_oscillation(self):
        """Detect oscillation - WITH GRACE PERIOD AND MOTOR HEALTH CHECK"""
        # CRITICAL: Do NOT check oscillation if no goal has been received yet
        if not hasattr(self, 'goal_received_time') or self.goal_received_time is None:
            return False
        
        # Don't detect oscillation if motors are unresponsive
        # (Robot isn't actually moving, so it can't be oscillating)
        if hasattr(self, 'motor_unresponsive_count') and self.motor_unresponsive_count >= 3:
            if self.debug_mode:
                self.get_logger().debug("⚠️ Motors unresponsive - skipping oscillation check")
            return False
        
        # Grace period after new goal - reduced to 5 seconds for better responsiveness
        time_since_goal = time.monotonic() - self.goal_received_time
        if time_since_goal < 5.0:  # Reduced from 10 to 5 second grace period
            return False
        
        # Need sufficient data
        if not hasattr(self, 'angular_history') or len(self.angular_history) < 25:
            return False
        
        if not hasattr(self, 'linear_history') or len(self.linear_history) < 25:
            return False
        
        # Check if turning a lot but not moving forward
        avg_angular = np.mean([abs(w) for w in self.angular_history])
        avg_linear = np.mean([abs(v) for v in self.linear_history])
        
        # CRITICAL: Only detect oscillation if robot is ACTUALLY MOVING
        # If avg_linear is very low, the robot isn't moving at all (hardware issue, not oscillation)
        if avg_linear < 0.1:  # Robot not moving enough to be oscillating
            return False
        
        # High angular, low/moderate linear (but robot IS moving) = oscillating
        # Robot is moving (>= 0.1) but still showing oscillatory behavior
        is_oscillating = avg_angular > 0.4 and avg_linear < 0.2

        # Also check position history
        if hasattr(self, 'position_history') and len(self.position_history) >= 20:
            positions = [pos for pos, _ in list(self.position_history)[-20:]]
            center = np.mean(positions, axis=0)
            radius = np.mean([np.hypot(p[0]-center[0], p[1]-center[1]) for p in positions])

            # Small radius = circling
            if radius < 0.30:  # Reduced from 0.2
                is_oscillating = True

        # Only declare oscillation if forward path is actually blocked
        if is_oscillating and self.has_forward_clearance(min_clearance=1.0):
            if self.debug_mode:
                self.get_logger().debug("✅ Forward clear - NOT oscillating (just slow turning)")
            return False

        return is_oscillating

    def attempt_free_space_escape(self):
        """Attempt to escape using free space directions"""
        if not self.free_space_directions or not self.odom:
            self.initiate_recovery()
            return
        
        # Find best free space direction that hasn't been tried recently
        best_angle = None
        best_score = -1.0
        
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y
        current_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
        
        for angle, clearance, score in self.free_space_directions:
            # Check if this direction is significantly different from current heading
            angle_diff = abs(math.atan2(math.sin(angle - current_yaw), math.cos(angle - current_yaw)))
            
            # Avoid directions that are too similar to current heading
            if angle_diff < math.radians(30):
                continue
            
            # Avoid recently tried directions
            recently_tried = False
            for tried_angle in self.escape_directions_tried[-3:]:
                if abs(math.atan2(math.sin(angle - tried_angle), math.cos(angle - tried_angle))) < math.radians(45):
                    recently_tried = True
                    break
            
            if recently_tried:
                continue
            
            if score > best_score:
                best_score = score
                best_angle = angle
        
        if best_angle is not None:
            self.escape_angle = best_angle
            self.last_escape_direction = 'free_space'
            self.escape_directions_tried.append(best_angle)
            
            # Execute a simple turn toward free space
            self.recovery_mode = True
            self.recovery_start_time = self.get_clock().now()
            self.recovery_phase = 'free_space_turn'
            
            if self.debug_mode:
                self.get_logger().info(f"🔄 Free space escape: turning to {math.degrees(best_angle):.1f}°")
        else:
            # Fall back to regular recovery
            self.initiate_recovery()

    def initiate_recovery(self):
        """Initiate recovery maneuver"""
        if self.recovery_mode:
            return
        
        self.recovery_mode = True
        self.recovery_start_time = None  # Will be set in execute_recovery
        self.recovery_stage = 0  # Start at stage 0
        self.recovery_phase = 'assess'  # Keep for compatibility
        self.recovery_count += 1
        
        # Find best escape direction based on obstacle positions and free space
        self.find_best_escape_direction()
        
        self.backup_distance = 0.0
        
        if self.debug_mode:
            self.get_logger().warn(f"🔄 INITIATING RECOVERY (Attempt {self.recovery_count})")
            self.get_logger().warn(f"  Obstacle direction: {math.degrees(self.obstacle_direction):.1f}°")
            if hasattr(self, 'escape_angle'):
                self.get_logger().warn(f"  Escape angle: {math.degrees(self.escape_angle):.1f}°")

    def find_best_escape_direction(self):
        """Find the best direction to escape from obstacles considering free space"""
        if not self.scan_ranges or not self.scan_angles:
            self.escape_angle = 0.0
            self.last_escape_direction = 'straight_back'
            return
        
        # Try to use free space information if available
        if self.free_space_directions:
            best_free_angle = None
            best_free_score = -1.0
            
            for angle, clearance, score in self.free_space_directions:
                # Avoid directions too close to obstacle direction
                if self.obstacle_direction != 0.0:
                    obstacle_diff = abs(math.atan2(
                        math.sin(angle - self.obstacle_direction),
                        math.cos(angle - self.obstacle_direction)
                    ))
                    if obstacle_diff < math.radians(45):
                        continue
                
                if score > best_free_score:
                    best_free_score = score
                    best_free_angle = angle
            
            if best_free_angle is not None:
                self.escape_angle = best_free_angle
                self.last_escape_direction = 'free_space_optimized'
                self.escape_directions_tried.append(best_free_angle)
                return
        
        # Fallback to obstacle-based escape
        attempt_count = len(self.escape_directions_tried) % 4
        
        if attempt_count == 0:
            # First attempt: try perpendicular to obstacle
            if self.obstacle_direction != 0.0:
                self.escape_angle = self.obstacle_direction + math.pi/2
                self.last_escape_direction = 'perpendicular'
            else:
                self.escape_angle = math.pi/2
                self.last_escape_direction = 'left'
        
        elif attempt_count == 1:
            # Second attempt: opposite of first
            self.escape_angle = -self.escape_angle
            self.last_escape_direction = 'opposite'
        
        elif attempt_count == 2:
            # Third attempt: try 60 degrees
            self.escape_angle = self.obstacle_direction + math.radians(60)
            self.last_escape_direction = 'sixty_degrees'
        
        else:
            # Fourth attempt: straight back
            self.escape_angle = math.pi
            self.last_escape_direction = 'straight_back'
        
        # Record this direction
        self.escape_directions_tried.append(self.escape_angle)
        
        # Normalize angle
        self.escape_angle = math.atan2(math.sin(self.escape_angle), math.cos(self.escape_angle))

    def execute_recovery(self):
        """Multi-stage recovery with obstacle clearing"""
        if not self.recovery_mode or not self.odom:
            return True  # Recovery complete
        
        current_time = self.get_clock().now()
        
        if self.recovery_start_time is None:
            self.recovery_start_time = current_time
            self.recovery_stage = 0
            self.get_logger().warn("🔄 Starting MULTI-STAGE recovery")
        
        elapsed = (current_time - self.recovery_start_time).nanoseconds * 1e-9
        
        # Stage 0: Stop and assess (0-1s)
        if self.recovery_stage == 0:
            if elapsed < 1.0:
                self.publish_cmd(0.0, 0.0)
                return False
            else:
                self.recovery_stage = 1
                self.get_logger().info("Stage 1: Backing up...")
                return False
        
        # Stage 1: Back up (1-3s)
        elif self.recovery_stage == 1:
            if elapsed < 3.0:
                self.publish_cmd(-0.25, 0.0)  # Reverse
                return False
            else:
                self.recovery_stage = 2
                self.get_logger().info("Stage 2: Turning to clear direction...")
                return False
        
        # Stage 2: Turn to clearest direction (3-5s)
        elif self.recovery_stage == 2:
            if elapsed < 5.0:
                best_dir = self.find_best_escape_direction_angle()
                turn_rate = 0.5 if best_dir > 0 else -0.5
                self.publish_cmd(0.0, turn_rate)
                return False
            else:
                self.recovery_stage = 3
                self.get_logger().info("Stage 3: Forward probe...")
                return False
        
        # Stage 3: Probe forward slowly (5-7s)
        elif self.recovery_stage == 3:
            if elapsed < 7.0:
                # Move forward slowly while checking clearance
                clearance = self.get_current_clearance()
                if clearance > 0.5:
                    self.publish_cmd(0.15, 0.0)
                else:
                    # Hit obstacle, try different direction
                    self.recovery_stage = 1  # Go back to stage 1
                    self.recovery_start_time = current_time
                return False
            else:
                self.get_logger().info("✅ Recovery complete")
                self.recovery_mode = False
                self.recovery_start_time = None
                self.recovery_stage = 0
                self.recovery_count = 0
                return True
        
        # Legacy phase support for free_space_turn
        elif self.recovery_phase == 'free_space_turn':
            # Special phase for free space escape
            current_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            angle_diff = self.escape_angle - current_yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            if abs(angle_diff) > math.radians(15):
                # Still turning
                v = 0.0
                w = 0.5 if angle_diff > 0 else -0.5
                self.publish_cmd(v, w)
                return False
            else:
                # Turn complete, move forward
                self.recovery_phase = 'forward'
                self.recovery_start_time = current_time
                return False
        
        return True

    def find_best_escape_direction_angle(self):
        """Find the best direction angle to escape from obstacles"""
        if not self.scan_ranges or not self.scan_angles:
            return 0.0
        
        # Scan all directions and find the one with maximum clearance
        best_angle = 0.0
        max_clearance = 0.0
        
        for i, (range_val, angle) in enumerate(zip(self.scan_ranges, self.scan_angles)):
            if 0.1 < range_val < 5.0 and range_val > max_clearance:
                max_clearance = range_val
                best_angle = angle
        
        return best_angle

    def has_forward_clearance(self, min_clearance=1.0):
        """Check if there is sufficient clearance in forward direction"""
        if not self.scan_ranges or not self.scan_angles:
            return False  # No data, assume blocked for safety

        # Check forward cone (±30 degrees)
        forward_ranges = []
        for angle, range_val in zip(self.scan_angles, self.scan_ranges):
            if abs(angle) < math.radians(30):  # ±30° = 60° forward cone
                if 0.1 < range_val < 50.0:  # Valid range
                    forward_ranges.append(range_val)

        if not forward_ranges:
            return False

        min_forward_clearance = min(forward_ranges)
        return min_forward_clearance > min_clearance

    def complete_recovery(self):
        """Complete recovery sequence"""
        self.recovery_mode = False
        self.recovery_start_time = None
        self.stuck_detected = False
        self.stuck_start_time = None
        self.obstacle_present_when_stuck = False
        self.path = None  # Clear current path to get new one

        # Toggle free space preference
        self.free_space_escape_preferred = not self.free_space_escape_preferred

        if self.debug_mode:
            self.get_logger().info("✅ Recovery sequence complete")

    def enhanced_obstacle_detection(self):
        """Enhanced obstacle detection with 360-degree awareness"""
        if not self.scan_ranges or not self.scan_angles:
            return

        ranges = np.array(self.scan_ranges)
        angles = np.array(self.scan_angles)

        # Check front sector more carefully
        front_indices = np.where(np.abs(angles) < math.radians(60))[0]
        
        if len(front_indices) > 0:
            front_ranges = ranges[front_indices]
            front_angles = angles[front_indices]
            
            # Find minimum distance in front
            valid_ranges = front_ranges[front_ranges > 0.1]
            if len(valid_ranges) > 0:
                min_dist = np.min(valid_ranges)
                min_idx = np.argmin(front_ranges)
                
                if min_dist < self.safe_distance:
                    self.obstacle_detected = True
                    self.obstacle_distance = min_dist
                    self.obstacle_direction = front_angles[min_idx]
                    
                    # Determine obstacle type
                    if min_dist < self.emergency_stop_distance:
                        self.obstacle_type = 'emergency'
                    elif min_dist < self.slow_down_distance:
                        self.obstacle_type = 'close'
                    else:
                        self.obstacle_type = 'warning'
                    
                    self.consecutive_obstacle_detections += 1
                    
                    if self.debug_mode and self.control_counter % 30 == 0:
                        if self.obstacle_type == 'emergency':
                            self.get_logger().warn(f"🚨 EMERGENCY obstacle at {min_dist:.2f}m")
                        else:
                            self.get_logger().info(f"🚧 Obstacle at {min_dist:.2f}m, direction {math.degrees(self.obstacle_direction):.1f}°")
                    return
        
        # No obstacles detected in front
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.consecutive_obstacle_detections = 0

    def path_callback(self, msg: Path):
        """Handle new path from optimizer"""
        if len(msg.poses) == 0:
            if self.debug_mode:
                self.get_logger().warn("Received empty path!")
            return

        # Skip path if goal is already reached and we've stopped
        if self.goal_stopped:
            if self.debug_mode:
                self.get_logger().debug("Ignoring path - goal already reached and stopped")
            return

        # Check if we should accept this path
        if self.require_explicit_goal and not self.has_explicit_goal:
            if self.debug_mode:
                self.get_logger().info("Ignoring path - no explicit goal received yet")
            return

        # Check for path timeout
        current_time = self.get_clock().now()
        if (self.last_path_update is not None and 
            (current_time - self.last_path_update).nanoseconds * 1e-9 > self.path_timeout):
            if self.debug_mode:
                self.get_logger().warn("Path timeout - discarding old path")
            self.path = None
            self.path_received = False

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
        
        # FIX 3: Path staleness detection - check if path start matches robot position
        if self.odom is not None:
            robot_x = self.odom.pose.pose.position.x
            robot_y = self.odom.pose.pose.position.y
            path_start_deviation = math.hypot(start_x - robot_x, start_y - robot_y)

            if path_start_deviation > self.path_staleness_threshold:
                # Path start is stale. Instead of rejecting the path outright, find the
                # nearest waypoint to the robot's current position and start tracking
                # from there.  Only reject if no waypoint is close enough.
                nearest_idx = 0
                nearest_dist = float('inf')
                for i, pose in enumerate(msg.poses):
                    d = math.hypot(
                        pose.pose.position.x - robot_x,
                        pose.pose.position.y - robot_y
                    )
                    if d < nearest_dist:
                        nearest_dist = d
                        nearest_idx = i

                max_acceptable_dist = self.path_staleness_threshold * 2.0
                if nearest_dist > max_acceptable_dist:
                    self.get_logger().warn(
                        f"🚫 Rejecting stale path: nearest waypoint={nearest_dist:.3f}m away "
                        f"(max {max_acceptable_dist:.3f}m). "
                        f"Path starts at ({start_x:.3f}, {start_y:.3f}), "
                        f"robot at ({robot_x:.3f}, {robot_y:.3f})"
                    )
                    self.path = None
                    self.path_received = False
                    return
                else:
                    # Accept path and jump to nearest waypoint
                    self.current_waypoint_idx = nearest_idx
                    self.get_logger().warn(
                        f"⚠️ Stale path start ({path_start_deviation:.3f}m): "
                        f"jumping to nearest waypoint {nearest_idx} "
                        f"(dist={nearest_dist:.3f}m)"
                    )
            elif path_start_deviation > self.path_staleness_warning_threshold:
                # Log warning for moderate deviations
                if self.debug_mode:
                    self.get_logger().warn(
                        f"⚠️ Path start deviation detected: {path_start_deviation:.3f}m "
                        f"(path: ({start_x:.3f}, {start_y:.3f}), robot: ({robot_x:.3f}, {robot_y:.3f}))"
                    )

        path_length = 0.0
        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i + 1].pose.position
            path_length += math.hypot(p2.x - p1.x, p2.y - p1.y)

        if self.debug_mode:
            self.get_logger().info(f"📈 New path received: {len(msg.poses)} waypoints, {path_length:.2f}m")
            self.get_logger().info(f"  Start: ({start_x:.3f}, {start_y:.3f})")
            self.get_logger().info(f"  Goal: ({goal_x:.3f}, {goal_y:.3f})")

    def control_loop_with_pinn_optimization(self):
        """Main control loop with PINN-based speed optimization"""
        self.control_counter += 1
        
        # CRITICAL: Verify frames match
        if self.odom is not None and self.path is not None:
            if self.odom.header.frame_id != self.path.header.frame_id:
                self.get_logger().error(
                    f"❌ FRAME MISMATCH: Odom frame={self.odom.header.frame_id}, "
                    f"Path frame={self.path.header.frame_id}"
                )
                self.publish_cmd(0.0, 0.0)
                return
        
        # Check if robot actually moving
        if self.odom and hasattr(self.odom.twist.twist.linear, 'x'):
            current_speed = abs(self.odom.twist.twist.linear.x)
            if current_speed < 0.01 and self.control_counter % 50 == 0:
                self.get_logger().warn(f"⚠️ Robot speed very low: {current_speed:.3f} m/s")
        
        # Rest of control loop...

        # Check for essential data
        if self.odom is None:
            if self.control_counter % 100 == 0:
                self.get_logger().debug("Waiting for odometry...")
            return

        if not self.scan_ranges:
            if self.control_counter % 100 == 0:
                self.get_logger().debug("Waiting for laser scan...")
            return

        # Goal completion check
        if self.goal_stopped:
            if self.control_counter % 100 == 0:
                self.get_logger().debug("✅ Goal achieved - robot fully stopped")
            return

        # Get current pose
        pose = self.odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quat_to_yaw(pose.orientation)

        # Throttled pose + goal diagnostics
        if self.debug_mode and self.control_counter % 50 == 0:
            goal_info = ""
            if self.goal_position is not None:
                gx, gy = self.goal_position
                gdist = math.hypot(gx - x, gy - y)
                gdir  = math.degrees(math.atan2(gy - y, gx - x))
                goal_info = f" | goal=({gx:.2f},{gy:.2f}) dist={gdist:.2f}m dir={gdir:.1f}°"
            self.get_logger().info(
                f"🤖 Pose: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°){goal_info}"
            )

        # Update distance tracking
        current_position = (x, y)
        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            self.distance_traveled += math.hypot(dx, dy)
        self.last_position = current_position

        # Emergency stop ONLY for immediate danger
        if self.obstacle_detected and self.obstacle_distance < 0.15:
            if self.debug_mode:
                self.get_logger().error(f"🚨 EMERGENCY STOP at {self.obstacle_distance:.2f}m!")
            self.publish_cmd(0.0, 0.0)
            return

        # Check if we're in recovery mode
        if self.recovery_mode:
            recovery_complete = self.execute_recovery()
            if not recovery_complete:
                return
            else:
                # Recovery complete, wait for new path
                self.publish_cmd(0.0, 0.0)
                return

        # Check if we have a valid path
        if self.path is None or not self.path_received:
            if self.control_counter % 50 == 0:
                self.get_logger().debug("No active path - holding")
            self.publish_cmd(0.0, 0.0)
            return

        # Initialize distance_to_goal to avoid UnboundLocalError
        # This will be used later if PINN optimization is called
        distance_to_goal = float('inf')
        
        # Goal completion check with improved tolerance
        if self.goal_position is not None:
            goal_x, goal_y = self.goal_position
            distance_to_goal = math.hypot(goal_x - x, goal_y - y)
            
            # Check if we're within goal completion distance
            if distance_to_goal < self.goal_completion_distance:
                if not self.goal_reached:
                    self.goal_reached = True
                    self.goal_reached_time = time.monotonic()
                    self.goal_stable_start_time = None  # Reset stability timer
                    self.get_logger().info(f"🎯 GOAL REACHED! Distance: {distance_to_goal:.3f}m")
                    self.get_logger().info(f"   Final position: ({x:.3f}, {y:.3f})")
                    self.get_logger().info(f"   Target: ({goal_x:.3f}, {goal_y:.3f})")
                
                # Stop immediately when goal is reached
                self.publish_cmd(0.0, 0.0)
                
                # Check for stability - robot should be stopped and stable
                current_time = time.monotonic()
                
                # Check if robot is moving slowly (stable)
                if self.odom is not None:
                    linear_vel = math.hypot(
                        self.odom.twist.twist.linear.x,
                        self.odom.twist.twist.linear.y
                    )
                    angular_vel = abs(self.odom.twist.twist.angular.z)
                    is_stable = (linear_vel < 0.05 and angular_vel < 0.05)
                    
                    if is_stable:
                        if self.goal_stable_start_time is None:
                            self.goal_stable_start_time = current_time
                        elif current_time - self.goal_stable_start_time >= self.goal_stability_time:
                            self.goal_stopped = True
                            self.get_logger().info(
                                f"🛑 Goal completed - robot stable for {self.goal_stability_time:.1f}s"
                            )
                    else:
                        # Reset stability timer if robot starts moving
                        self.goal_stable_start_time = None
                elif self.goal_reached_time is not None and \
                     current_time - self.goal_reached_time >= 1.5:
                    # Fallback if no odometry - use time-based completion
                    self.goal_stopped = True
                    self.get_logger().info("🛑 Goal completed - timeout reached")
                
                return
            else:
                # Robot moved away from goal - reset completion state
                if self.goal_reached and distance_to_goal > self.goal_completion_distance * 1.5:
                    self.goal_reached = False
                    self.goal_reached_time = None
                    self.goal_stopped = False
                    self.goal_stable_start_time = None
                    if self.debug_mode:
                        self.get_logger().debug("Goal reset - moving away from goal")


        # GET TARGET WAYPOINT
        target_waypoint = self.select_target_waypoint(x, y, yaw)
        
        if target_waypoint is None:
            if self.debug_mode:
                self.get_logger().info("All waypoints reached but not at goal")
            self.publish_cmd(0.0, 0.0)
            self.path_received = False
            return

         # ------------------------------------------------------------------
        # DWA OBSTACLE AVOIDANCE (ALWAYS ACTIVE)
        # ------------------------------------------------------------------
        if self.use_dwa and not self.goal_reached and self.path is not None:
            try:
                current_pose = (x, y, yaw)
                v, w = self.dwa.plan(
                    current_pose, 
                    self.path, 
                    self.scan_ranges, 
                    self.scan_angles
                )
                
                # --- Throttled diagnostic: goal distance & heading error ---
                if self.debug_mode and self.control_counter % 40 == 0:
                    self.get_logger().info(f"🤖 DWA ACTIVE: v={v:.3f}, w={w:.3f}")
                    if self.goal_position is not None:
                        gx, gy = self.goal_position
                        g_dist = math.hypot(gx - x, gy - y)
                        g_dir = math.atan2(gy - y, gx - x)
                        h_err = math.degrees(abs(math.atan2(
                            math.sin(g_dir - yaw), math.cos(g_dir - yaw))))
                        self.get_logger().info(
                            f"   goal_dist={g_dist:.2f}m heading_err={h_err:.1f}° "
                            f"yaw={math.degrees(yaw):.1f}°"
                        )
                
                v = np.clip(v, 0.0, self.max_linear_vel * 0.8)
                w = np.clip(w, -self.max_angular_vel * 0.8, self.max_angular_vel * 0.8)
                
                # Apply min_linear_vel ONLY when angular command is small AND
                # heading error to goal is within a tolerable range.
                # When the robot needs to rotate significantly (>30°) to face the
                # goal, forcing min forward speed at the same time produces circular
                # motion – the previous failure mode.  We skip the min-vel clamp
                # whenever the heading error is large OR angular rate is high.
                _angular_threshold_for_min_vel = 0.25  # rad/s
                if v > 0 and v < self.min_linear_vel:
                    _large_heading_error = False
                    if self.goal_position is not None:
                        gx, gy = self.goal_position
                        _goal_dir = math.atan2(gy - y, gx - x)
                        _heading_err = abs(math.atan2(
                            math.sin(_goal_dir - yaw), math.cos(_goal_dir - yaw)))
                        _large_heading_error = _heading_err > math.radians(30)

                    if abs(w) < _angular_threshold_for_min_vel and not _large_heading_error:
                        v = self.min_linear_vel
                        if self.debug_mode and self.control_counter % 40 == 0:
                            self.get_logger().info(
                                f"🔒 DWA v clamped to min_linear_vel={self.min_linear_vel:.3f} "
                                f"(|w|={abs(w):.3f} < {_angular_threshold_for_min_vel})"
                            )
                    else:
                        if self.debug_mode and self.control_counter % 40 == 0:
                            reason = ("large heading error" if _large_heading_error
                                      else f"|w|={abs(w):.3f} >= {_angular_threshold_for_min_vel}")
                            self.get_logger().info(
                                f"🔄 DWA min_linear_vel clamp skipped ({reason}, preventing circular motion)"
                            )
                    
                v, w = self.apply_motion_limits(v, w)
                
                if self.pinn_client is not None:
                    v, w = self.optimize_speed_with_pinn(v, w, distance_to_goal)

                v, w = self.apply_reactive_speed_scaling(v, w)

                self.update_statistics(v)
                self.publish_cmd(float(v), float(w))
                
                return
                
            except Exception as e:
                if self.debug_mode:
                    self.get_logger().warn(f"DWA planning failed: {e}")
                self.dwa_failures += 1
                if self.dwa_failures > self.max_dwa_failures:
                    self.use_dwa = False
                    if self.debug_mode:
                        self.get_logger().warn("Disabling DWA after too many failures")

        # ------------------------------------------------------------------
        # SIMPLE CONTROL WITH FREE SPACE OPTIMIZATION AND PINN
        # ------------------------------------------------------------------
        if not self.goal_reached:
            v, w = self.calculate_simple_control(target_waypoint, x, y, yaw)
            
            # BUG FIX: Skip all velocity modifications when rotating in place
            if not self.is_rotating_in_place:
                # If turning sharply, check if there's better free space direction
                if abs(w) > 0.3 and self.prefer_free_space_turns and self.free_space_directions:
                    current_turn_dir = w > 0
                    alternative_found = False
                    
                    for angle, clearance, score in self.free_space_directions[:3]:
                        angle_diff = angle - yaw
                        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                        
                        if (clearance > self.obstacle_distance * 1.5 and 
                            abs(angle_diff) < math.radians(60)):
                            
                            w_adjust = angle_diff * 1.2
                            w_adjust = np.clip(w_adjust, -self.max_angular_vel, self.max_angular_vel)
                            
                            if self.debug_mode and abs(w_adjust - w) > 0.2:
                                self.get_logger().debug(f"Adjusting turn toward free space: {math.degrees(angle):.1f}°")
                            
                            w = w_adjust
                            alternative_found = True
                            break
                
                # Apply enhanced speed limits
                v = self.apply_enhanced_speed_limit(v, w)
                v = self.adjust_speed_for_curvature(v, w)

                # Apply motion limits
                v, w = self.apply_motion_limits(v, w)

                # Apply minimum velocity threshold
                if v > 0 and v < self.min_linear_vel:
                    v = self.min_linear_vel
                
                # CRITICAL: Only apply PINN if NOT on a straight path
                is_straight_path = False
                if self.path and len(self.path.poses) >= 2:
                    goal_pose = self.path.poses[-1].pose.position
                    start_pose = self.path.poses[0].pose.position
                    path_dx = goal_pose.x - start_pose.x
                    path_dy = goal_pose.y - start_pose.y
                    
                    # Check if path is straight
                    if abs(path_dx) > 0.2 and abs(path_dy) / abs(path_dx) < 0.03:
                        is_straight_path = True
                
                # Apply PINN optimization ONLY if not straight
                if self.pinn_client is not None and not is_straight_path:
                    v, w = self.optimize_speed_with_pinn(v, w, distance_to_goal)
                    if self.debug_mode and self.control_counter % 50 == 0:
                        self.get_logger().info(f"📐 Simple Control with PINN: v={v:.3f}, w={w:.3f}")
                elif is_straight_path:
                    # Force w=0 for straight paths
                    w = 0.0
                    if self.debug_mode and self.control_counter % 50 == 0:
                        self.get_logger().info(f"📐 Simple Control (straight, NO PINN): v={v:.3f}, w=0.000")
            else:
                # When rotating in place, only apply angular limits to w, keep v=0
                w = np.clip(w, -self.max_angular_vel, self.max_angular_vel)
                if self.debug_mode and self.control_counter % 50 == 0:
                    self.get_logger().info(f"🔄 Rotating in place (no mods): v={v:.3f}, w={w:.3f}")

            # **NEW: Check if we're needlessly going backward when forward is clear**
            if self.scan_ranges and len(self.scan_ranges) > 0 and v < 0:
                forward_ranges = []
                for i, angle in enumerate(self.scan_angles):
                    if abs(angle) < math.radians(30):  # ±30° = 60° cone
                        if 0.1 < self.scan_ranges[i] < 50.0:
                            forward_ranges.append(self.scan_ranges[i])

                if forward_ranges:
                    forward_clearance = min(forward_ranges)
                    if forward_clearance > 1.0:
                        if self.debug_mode:
                            self.get_logger().warn(
                                f"⚠️ Blocking backward motion (v={v:.3f}) - forward clear ({forward_clearance:.2f}m)"
                            )
                        v = 0.2  # Force slow forward motion instead

            # Reactive speed scaling: zone-based v scaling, w preserved
            v, w = self.apply_reactive_speed_scaling(v, w)

            self.update_statistics(v)
            self.publish_cmd(float(v), float(w))

    def calculate_simple_control(self, target_waypoint, current_x, current_y, current_yaw):
        """Simple control with improved lateral goal handling and tunable parameters"""
        target_x = target_waypoint.x
        target_y = target_waypoint.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        
        # Calculate bearing to target
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle error with proper wrapping
        angle_diff = target_angle - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Small angle error: use deadband
        heading_deadband_rad = math.radians(self.heading_deadband_deg)
        if abs(angle_diff) < heading_deadband_rad:
            v = min(self.max_linear_vel * 0.7, distance * 0.5)
            w = 0.0
            if self.debug_mode:
                self.get_logger().info(
                    f"📐 Angle error within deadband ({math.degrees(angle_diff):.2f}°) - w=0"
                )
            self.is_rotating_in_place = False
            return v, w
        
        rotate_threshold_rad = math.radians(self.rotate_in_place_angle_deg)
        
        if distance < self.close_range_distance:
            # Close to goal: proportional control
            v = min(self.max_linear_vel * 0.4, distance * 0.5)
            w = angle_diff * self.heading_kp
            w = np.clip(w, -self.max_heading_rate, self.max_heading_rate)
            if abs(v) > 0.05:
                self.rotation_start_time = None
            if self.debug_mode:
                self.get_logger().info(
                    f"🎯 Close-range proportional: dist={distance:.2f}m, angle={math.degrees(angle_diff):.1f}°, v={v:.3f}, w={w:.3f}"
                )
            self.is_rotating_in_place = False
            return v, w
        elif abs(angle_diff) > rotate_threshold_rad and \
                not self.has_forward_clearance(self.rotate_free_space_threshold):
            # Rotate in place if forward blocked and large angle error
            current_time = time.monotonic()
            if self.rotation_start_time is None:
                self.rotation_start_time = current_time
            rotation_duration = current_time - self.rotation_start_time
            if rotation_duration > self.rotation_timeout:
                # Timeout: force forward motion
                if self.debug_mode:
                    self.get_logger().warn(
                        f"⚠️ Rotation timeout ({rotation_duration:.1f}s) exceeded! Forcing forward movement."
                    )
                v = min(self.max_linear_vel * 0.4, distance * 0.5)
                w = angle_diff * self.heading_kp * 0.5
                w = np.clip(w, -self.max_heading_rate * 0.5, self.max_heading_rate * 0.5)
                self.rotation_start_time = None
                self.is_rotating_in_place = False
            else:
                v = 0.0
                current_direction = np.sign(angle_diff)
                # Anti‑oscillation lock
                if (self.rotation_direction_lock_time is not None and 
                    current_time - self.rotation_direction_lock_time < self.rotation_direction_lock_duration and
                    self.last_rotation_direction is not None):
                    min_angle_for_lock_rad = math.radians(self.rotation_min_angle_for_lock_deg)
                    if abs(angle_diff) > min_angle_for_lock_rad:
                        w = self.last_rotation_direction * self.max_heading_rate
                    else:
                        w = current_direction * self.max_heading_rate
                        self.last_rotation_direction = current_direction
                        self.rotation_direction_lock_time = current_time
                else:
                    w = current_direction * self.max_heading_rate
                    self.last_rotation_direction = current_direction
                    self.rotation_direction_lock_time = current_time
                if self.debug_mode:
                    self.get_logger().info(
                        f"🔄 Rotating in place: angle_error={math.degrees(angle_diff):.1f}°, w={w:.3f}, duration={rotation_duration:.1f}s"
                    )
                self.is_rotating_in_place = True
            return v, w
        else:
            # Forward with turning
            self.rotation_start_time = None
            alignment_factor = max(0.3, 1.0 - abs(angle_diff) / rotate_threshold_rad)
            if distance > 2.0:
                v = min(self.max_linear_vel, distance * 0.4)
            elif distance > 0.5:
                v = min(self.max_linear_vel * 0.8, distance * 0.7)
            else:
                v = min(self.max_linear_vel * 0.6, distance * 1.0)
            v *= alignment_factor
            w = angle_diff * self.heading_kp
            w = np.clip(w, -self.max_heading_rate * 0.8, self.max_heading_rate * 0.8)
            if self.debug_mode:
                self.get_logger().info(
                    f"🚗 Forward+turn: angle_error={math.degrees(angle_diff):.1f}°, v={v:.3f}, w={w:.3f}"
                )
            self.is_rotating_in_place = False
            return v, w

    def get_current_clearance(self):
        """Get minimum clearance around robot"""
        if not self.scan_ranges:
            return float('inf')
        
        min_clearance = float('inf')
        for range_val in self.scan_ranges:
            if 0.1 < range_val < min_clearance:
                min_clearance = range_val
        
        return min_clearance if min_clearance != float('inf') else 5.0

    def get_adaptive_clearance(self):
        """Get adaptive obstacle clearance based on context"""
        base_clearance = 0.35  # Increased from 0.2
        
        # If stuck or oscillating, reduce clearance to allow escape
        is_robot_stuck = self.stuck_detected or self.recovery_mode
        if is_robot_stuck or self.detect_oscillation():
            return 0.25  # Allow closer approach when stuck
        
        # If moving fast, increase clearance
        if hasattr(self, 'last_linear_vel') and abs(self.last_linear_vel) > 0.3:
            return base_clearance + 0.15
        
        return base_clearance

    def select_target_waypoint(self, current_x, current_y, current_yaw):
        """Select target waypoint with adaptive lookahead"""
        if not self.path or not self.path.poses:
            return None

        # Use adaptive lookahead based on speed and obstacles
        if self.adaptive_lookahead:
            base_lookahead = self.lookahead_distance
            if self.obstacle_detected and self.obstacle_distance < 1.0:
                lookahead = max(0.3, base_lookahead * (self.obstacle_distance / 1.0))
            else:
                lookahead = base_lookahead
        else:
            lookahead = self.lookahead_distance

        # Find waypoint at lookahead distance
        accumulated_dist = 0.0
        start_idx = self.current_waypoint_idx
        
        for i in range(start_idx, len(self.path.poses) - 1):
            p1 = self.path.poses[i].pose.position
            p2 = self.path.poses[i + 1].pose.position
            
            segment_dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
            
            if accumulated_dist + segment_dist >= lookahead:
                # Interpolate within this segment
                t = (lookahead - accumulated_dist) / segment_dist
                target_x = p1.x + t * (p2.x - p1.x)
                target_y = p1.y + t * (p2.y - p1.y)
                
                # Update current waypoint index
                if i > start_idx:
                    self.current_waypoint_idx = i
                
                return type('Point', (), {'x': target_x, 'y': target_y})()
            
            accumulated_dist += segment_dist
        
        # If we reach here, return the final waypoint
        self.current_waypoint_idx = len(self.path.poses) - 2
        final_pose = self.path.poses[-1].pose.position
        return type('Point', (), {'x': final_pose.x, 'y': final_pose.y})()
    def optimize_speed_with_pinn(self, current_v, current_w, distance_to_goal):
        """Use PINN to optimize speed for energy efficiency - WITH STRAIGHT-LINE BYPASS"""
        if not hasattr(self, 'pinn_client') or self.pinn_client is None:
            return current_v, current_w
        
        # CRITICAL: If this is a straight-line path, DON'T optimize angular velocity
        if self.path and len(self.path.poses) >= 2:
            goal = self.path.poses[-1].pose.position
            start = self.path.poses[0].pose.position if self.odom is None else self.odom.pose.pose.position
            
            dx = goal.x - start.x
            dy = goal.y - start.y
            
            # If essentially straight (dy < 5cm and dx > 20cm), preserve w=0
            if abs(dy) < 0.05 and abs(dx) > 0.2:
                # Only optimize linear velocity, keep angular at 0
                if abs(current_w) < 0.01:  # If w is already ~0
                    self.get_logger().info(f"🎯 PINN: Preserving straight motion (w=0.000)")
                    return current_v, 0.0  # Force w=0 for straight paths
        
        # Only optimize for longer distances
        if distance_to_goal < 2.0:
            return current_v, current_w
        
        try:
            # Generate a simple trajectory for PINN evaluation
            xs = [self.odom.pose.pose.position.x]
            ys = [self.odom.pose.pose.position.y]
            yaws = [self.quat_to_yaw(self.odom.pose.pose.orientation)]
            
            # Project forward 2 seconds
            for i in range(1, 5):
                xs.append(xs[0] + current_v * i * 0.5 * math.cos(yaws[0]))
                ys.append(ys[0] + current_v * i * 0.5 * math.sin(yaws[0]))
                yaws.append(yaws[0] + current_w * i * 0.5)
            
            # Call PINN service
            req = EvaluateTrajectory.Request()
            req.xs = xs
            req.ys = ys
            req.yaws = yaws
            req.velocities = [current_v] * len(xs)
            
            future = self.pinn_client.call_async(req)
            
            # Try to get result quickly
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 0.1:
                    break
                time.sleep(0.01)
            
            if future.done():
                response = future.result()
                if response:
                    energy = response.energy
                    stability = response.stability
                    
                    # Optimize: reduce speed if energy too high or stability too low
                    if energy > 50.0 or stability < 0.7:
                        return current_v * 0.8, current_w * 0.8
                    elif energy < 20.0 and stability > 0.9:
                        # Can increase speed
                        return min(current_v * 1.2, self.max_linear_vel), current_w
        
        except Exception as e:
            if self.debug_mode:
                self.get_logger().debug(f"PINN speed optimization failed: {e}")
        
        return current_v, current_w

    def setup_pinn_client_in_controller(self):
        """Setup PINN service client for controller"""
        try:
            self.pinn_client = self.create_client(
                EvaluateTrajectory,
                '/evaluate_trajectory'
            )
            
            self.get_logger().info("Creating PINN client for controller...")
            
            # Wait for service
            if self.pinn_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info("✅ PINN client connected for controller")
                return True
            else:
                self.get_logger().warn("PINN service not available for controller")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Failed to create PINN client for controller: {e}")
            return False

    def initialize_pinn_for_controller(self):
        """Initialize PINN for controller if enabled"""
        try:
            use_pinn_in_controller = self.get_parameter('use_pinn_in_controller').value
        except:
            use_pinn_in_controller = True
        
        if use_pinn_in_controller:
            return self.setup_pinn_client_in_controller()
        
        return False


    def apply_enhanced_speed_limit(self, velocity, angular_velocity):
        """Enhanced speed limiting with free space consideration - MAX SPEED in open space"""
        # Check if we're in open space
        current_clearance = self.get_current_clearance()
        is_open_space = current_clearance > 2.0
        
        if is_open_space and not self.obstacle_detected:
            # Full speed in open space
            return min(velocity, self.max_linear_vel)
        
        if not self.obstacle_detected or self.obstacle_distance == float('inf'):
            return min(velocity, self.max_linear_vel)

        if self.obstacle_distance < self.emergency_stop_distance:
            return 0.0

        elif self.obstacle_distance < self.slow_down_distance:
            t = (self.obstacle_distance - self.emergency_stop_distance) / \
                (self.slow_down_distance - self.emergency_stop_distance)
            
            reduction_factor = 0.3 + 0.7 * (1.0 / (1.0 + math.exp(-8.0 * (t - 0.5))))
            
            if abs(angular_velocity) > 0.4:
                reduction_factor *= 0.8

            return velocity * reduction_factor

        elif self.obstacle_distance < self.safe_distance:
            reduction = 0.1 + 0.2 * (1.0 - self.obstacle_distance / self.safe_distance)
            return velocity * (1.0 - reduction)

        else:
            return min(velocity, self.max_linear_vel)
    def adjust_speed_for_curvature(self, base_speed, angular_velocity):
        """Adjust speed based on path curvature"""
        if abs(angular_velocity) < 0.1:
            return base_speed
        
        turning_factor = 1.0 - (abs(angular_velocity) / self.max_angular_vel) * 0.4
        adjusted_speed = base_speed * turning_factor
        # BUG FIX: Don't enforce min_linear_vel when rotating in place (base_speed=0)
        if base_speed > 0.0:
            adjusted_speed = max(adjusted_speed, self.min_linear_vel)
        return adjusted_speed

    def apply_motion_limits(self, v, w):
        """Apply acceleration and deceleration limits - FIXED FOR REAL ROBOT"""
        # REAL ROBOT: More conservative limits
        max_accel_v = self.acceleration_limit * 0.5 * (1.0/self.control_frequency)
        max_decel_v = self.deceleration_limit * 0.5 * (1.0/self.control_frequency)
        max_accel_w = self.acceleration_limit * 1.0 * (1.0/self.control_frequency)

        delta_v = v - self.last_cmd[0]
        delta_w = w - self.last_cmd[1]

        # Limit linear acceleration
        if delta_v > 0:
            max_delta_v = max_accel_v
        else:
            max_delta_v = max_decel_v

        if abs(delta_v) > max_delta_v:
            v = self.last_cmd[0] + math.copysign(max_delta_v, delta_v)

        # Limit angular acceleration
        if abs(delta_w) > max_accel_w:
            w = self.last_cmd[1] + math.copysign(max_accel_w, delta_w)

        # REAL ROBOT: More conservative angular limits
        w = np.clip(w, -self.max_angular_vel * 0.6, self.max_angular_vel * 0.6)

        return v, w

    def update_statistics(self, current_speed):
        """Update performance statistics"""
        self.speed_history.append(current_speed)
        if len(self.speed_history) > 50:
            self.speed_history.pop(0)

        if len(self.speed_history) > 0:
            self.avg_speed = sum(self.speed_history) / len(self.speed_history)
    def get_forward_obstacle_distance(self):
        """
        Compute the minimum distance to obstacles directly in front of the robot
        within a 60° forward cone (±30°). Returns the closest distance, or inf if none.
        """
        if not self.scan_ranges or not self.scan_angles:
            return float('inf')
        forward_ranges = []
        for r, a in zip(self.scan_ranges, self.scan_angles):
            if abs(a) < math.radians(30):   # 60° forward cone
                if 0.1 < r < 50.0:
                    forward_ranges.append(r)
        return min(forward_ranges) if forward_ranges else float('inf')

    def apply_reactive_speed_scaling(self, v, w):
        """
        Scale linear speed reactively based on obstacles in the forward direction.
        Side obstacles do not reduce forward speed – they only affect turning (handled elsewhere).
        """
        if not self.scan_ranges:
            return v, w

        # Use forward distance instead of global minimum distance
        dist = self.get_forward_obstacle_distance()

        # Track approach velocity (positive = obstacle moving away, negative = closing)
        closing_vel = 0.0
        if (self._reactive_prev_obstacle_dist != float('inf')
                and dist != float('inf')):
            dt = 1.0 / max(self.control_frequency, 1.0)
            closing_vel = (dist - self._reactive_prev_obstacle_dist) / dt
        self._reactive_prev_obstacle_dist = dist

        if dist == float('inf'):
            self._reactive_in_risk_zone = False
            self._reactive_in_safety_zone = False
            return v, w

        risk_dist = self.reactive_risk_zone_dist
        safety_dist = self.reactive_safety_zone_dist
        hyst = self.reactive_hysteresis_factor

        # Update risk zone state with hysteresis
        if self._reactive_in_risk_zone:
            if dist > risk_dist * (1.0 + hyst):
                self._reactive_in_risk_zone = False
        else:
            if dist < risk_dist:
                self._reactive_in_risk_zone = True

        # Update safety zone state with hysteresis
        if self._reactive_in_safety_zone:
            if dist > safety_dist * (1.0 + hyst):
                self._reactive_in_safety_zone = False
        else:
            if dist < safety_dist:
                self._reactive_in_safety_zone = True

        # Approach velocity factor: increase slowdown when obstacle closes quickly
        approach_factor = 1.0
        if closing_vel < self.reactive_approach_threshold:
            approach_factor = max(0.3, 1.0 + closing_vel * self.reactive_approach_gain)

        if self._reactive_in_risk_zone:
            # Risk zone: stop forward motion; allow backward escape and keep w
            scaled_v = 0.0 if v > 0.0 else v
            if self.debug_mode and self.control_counter % 40 == 0:
                self.get_logger().info(
                    f"🛑 REACTIVE RISK ZONE (forward {dist:.2f}m): v→0, w={w:.3f} preserved"
                )
            return scaled_v, w

        if self._reactive_in_safety_zone:
            # Safety zone: linearly scale v from 0 (at risk boundary) to 1 (at safety boundary)
            t = (dist - risk_dist) / max(safety_dist - risk_dist, 0.01)
            t = max(0.0, min(1.0, t))
            scale = max(0.0, min(1.0, t * approach_factor))
            scaled_v = v * scale
            if self.debug_mode and self.control_counter % 40 == 0:
                self.get_logger().info(
                    f"⚠️ REACTIVE SAFETY ZONE (forward {dist:.2f}m): scale={scale:.2f}, "
                    f"v={v:.3f}→{scaled_v:.3f}, w={w:.3f}"
                )
            return scaled_v, w

        # Far zone: no scaling
        return v, w
    def publish_cmd(self, linear_vel, angular_vel):
        """Fixed publish command with calibration, motor health tracking, and diagnostics."""
        msg = Twist()
        
        # Store raw values for logging
        raw_linear = float(linear_vel)
        raw_angular = float(angular_vel)
        
        # APPLY CALIBRATION FACTORS
        linear_vel = raw_linear * self.linear_scale
        
        # CRITICAL: Apply angular_sign_correction to flip angular velocity sign if needed
        # This corrects for platform-specific rotation conventions (e.g., Scout Mini CW/CCW)
        angular_vel_scaled = raw_angular * self.angular_scale
        angular_vel = angular_vel_scaled * self.angular_sign
        
        # Clamp to limits
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # REAL ROBOT FIX: Minimum thresholds - apply deadband for very small velocities
        if abs(linear_vel) < 0.03:  # Reduced from 0.05
            linear_vel = 0.0
        if abs(angular_vel) < 0.01:  # CRITICAL: Reduced from 0.05 to 0.01 to allow turning
            angular_vel = 0.0
        
        # Track velocity history for oscillation detection
        self.linear_history.append(linear_vel)
        self.angular_history.append(angular_vel)
        self.last_linear_vel = linear_vel
        
        # Track commanded velocity for motor health monitoring
        self.last_commanded_linear = abs(linear_vel)
        self.commanded_velocity_history.append(self.last_commanded_linear)
        
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        
        # ── Per-command diagnostics: heading_err, clearance, v, w ─────────────
        if self.debug_mode and self.control_counter % 20 == 0 and self.odom is not None:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)

            heading_err_deg = 0.0
            goal_dir_deg = 0.0
            goal_dist = 0.0
            if self.goal_position is not None:
                gx, gy = self.goal_position
                goal_dist = math.hypot(gx - x, gy - y)
                goal_dir = math.atan2(gy - y, gx - x)
                goal_dir_deg = math.degrees(goal_dir)
                heading_err_deg = math.degrees(abs(math.atan2(
                    math.sin(goal_dir - yaw), math.cos(goal_dir - yaw))))

            clearance = self.obstacle_distance if self.obstacle_detected else float('inf')

            turn_dir = 'NONE'
            if abs(raw_angular) > 0.01:
                turn_dir = 'LEFT(CCW)' if raw_angular > 0 else 'RIGHT(CW)'

            self.get_logger().info(
                f'📤 CMD v={linear_vel:.3f} w={angular_vel:.3f} [{turn_dir}]'
                f' | heading_err={heading_err_deg:.1f}° goal_dir={goal_dir_deg:.1f}°'
                f' goal_dist={goal_dist:.2f}m clearance={clearance:.2f}m'
            )

            # Log anomaly to CSV when heading error is large or v=0 with a goal
            if heading_err_deg > 30.0 and goal_dist > 0.3:
                self._log_anomaly(
                    'LARGE_HEADING_ERR', linear_vel, angular_vel,
                    heading_err_deg, clearance,
                )
        
        self.cmd_pub.publish(msg)
        self.last_cmd = (raw_linear, raw_angular)

    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Enhanced controller shutting down...")
    except Exception as e:
        node.get_logger().error(f"💥 Controller error: {str(e)}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.publish_cmd(0.0, 0.0)
        # Ensure CSV log is flushed / closed cleanly
        if hasattr(node, '_csv_file') and node._csv_file is not None:
            try:
                node._csv_file.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
