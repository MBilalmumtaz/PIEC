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
from .dynamic_obstacle_tracker import DynamicObstacleTracker
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray  # ADDED THIS IMPORT
from std_msgs.msg import String, Float64MultiArray   # for /emergency_stop/reason and /ukf/health
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
        # Emergency escape mode
        self.emergency_escape_active = False
        self.emergency_escape_start_time = None
        self.emergency_escape_timeout = 3.0   # seconds
        self.escape_rotation_direction = 1   # 1 = left, -1 = right
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
        # Dynamic obstacle monitoring
        self.dynamic_predictor = None
        if hasattr(self, 'scan_topic'):
            try:
                self.dynamic_predictor = DynamicObstacleTracker(self)
                self.get_logger().info("✅ Dynamic obstacle predictor initialized")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize dynamic predictor: {e}")

        # Predictive safety margins
        self.adaptive_safety_margin = 0.6
        self.last_predicted_collision = None
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

        # ── UKF health state ─────────────────────────────────────────────────
        # Filled by /ukf/health callback: [trace, is_healthy, max_eig, mahalanobis]
        self._ukf_trace: float = 0.0
        self._ukf_healthy: bool = True  # assume healthy until first health msg

        # ── Zero-velocity watchdog ───────────────────────────────────────────
        # When DWA outputs v≈0 for longer than _zero_vel_watchdog_timeout seconds
        # and heading error is large, force a rotation-in-place to break the deadlock.
        self._zero_vel_watchdog_start: float = 0.0   # wall-clock of first v=0 cmd
        self._zero_vel_watchdog_timeout: float = 2.0  # seconds before forcing rotation
        self._watchdog_rotating: bool = False          # True while watchdog is rotating
        self._watchdog_last_warn_time: float = 0.0    # throttle watchdog log

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
        # Subscribe to UKF health for covariance-aware gating
        self.create_subscription(
            Float64MultiArray, '/ukf/health',
            self._ukf_health_callback, 10
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
        # Add AFTER emergency stop distance declaration:
        self._last_obstacle_dist = float('inf')  # Track obstacle distance for replanning
        self._zero_vel_watchdog_start = 0.0
        self._zero_vel_watchdog_timeout = 2.0
        self._watchdog_rotating = False
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
            'emergency_stop_distance': 0.80,
            'slow_down_distance': 0.90,
            'safe_distance': 1.0,
            'obstacle_clearance': 0.5,
            'lateral_safety_margin': 0.4,

            # Path Following
            'waypoint_tolerance': 0.3,
            'path_timeout': 60.0,
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

    def _ukf_health_callback(self, msg: Float64MultiArray):
        """Store UKF health data: [trace, is_healthy, max_eigenvalue, mahalanobis]."""
        if len(msg.data) >= 2:
            self._ukf_trace = float(msg.data[0])
            self._ukf_healthy = bool(msg.data[1] > 0.5)

    def call_pinn_service_nonblocking(self, path_array):
        """Call PINN asynchronously without blocking control loop."""
        if not self.use_pinn or not self.pinn_service_available:
            return {'energy': 0.0, 'stability': 1.0, 'success': False}

        if self.pinn_future_lock.locked():
            return {'energy': 0.0, 'stability': 1.0, 'success': False, 'pending': True}

        try:
            request = PINN_SERVICE_TYPE.Request()
            request.xs = [float(p[0]) for p in path_array]
            request.ys = [float(p[1]) for p in path_array]
            request.yaws = [0.0] * len(path_array)
            request.velocities = [0.5] * len(path_array)

            future = self.pinn_client.call_async(request)

            with self.pinn_future_lock:
                call_id = id(future)
                self.pinn_futures[call_id] = {
                    'future': future,
                    'time': time.time(),
                    'path_len': len(path_array)
                }

            return {'energy': 0.0, 'stability': 1.0, 'success': False, 'pending': True}

        except Exception as e:
            self.get_logger().debug(f"PINN async call failed: {e}")
            return {'energy': 0.0, 'stability': 1.0, 'success': False}

    def _process_pending_pinn_calls(self):
        """Check completed PINN calls – call this periodically, not blocking."""
        if not self.pinn_futures:
            return

        with self.pinn_future_lock:
            completed = []
            for call_id, info in list(self.pinn_futures.items()):
                future = info['future']
                elapsed = time.time() - info['time']

                if future.done():
                    try:
                        response = future.result()
                        self.get_logger().info(
                            f"✅ PINN response (path_len={info['path_len']}): "
                            f"energy={response.energy:.2f}, stability={response.stability:.3f}"
                        )
                        completed.append(call_id)
                    except Exception:
                        completed.append(call_id)
                elif elapsed > 3.0:   # timeout
                    completed.append(call_id)
                    self.get_logger().warn(f"⚠️ PINN call timed out after {elapsed:.1f}s")

            for call_id in completed:
                del self.pinn_futures[call_id]

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
        self.goal_received_time = time.monotonic()
        
        # Reset goal completion flags when new goal received
        self.goal_reached = False
        self.goal_stopped = False
        self.goal_reached_time = None
        self.goal_position = (goal_x, goal_y)
        self.path = None
        self.path_received = False
        
        # Reset stuck detection
        self.stuck_detected = False
        self.recovery_mode = False
        self.position_history.clear()
        self.consecutive_obstacle_readings = 0
        self.escape_directions_tried.clear()
        self.last_escape_direction = None
        self.free_space_escape_preferred = True
        
        # Reset emergency escape mode
        self.emergency_escape_active = False
        
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

        if len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = [
                float(msg.angle_min + i * msg.angle_increment)
                for i in range(len(msg.ranges))
            ]

        if self.dynamic_predictor is not None and self.odom is not None:
            robot_x = self.odom.pose.pose.position.x
            robot_y = self.odom.pose.pose.position.y
            robot_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            robot_vx = self.odom.twist.twist.linear.x
            robot_vy = self.odom.twist.twist.linear.y

            # LiDAR extrinsics (from tf2_echo base_link -> rslidar)
            sensor_x = 0.15
            sensor_y = 0.0
            sensor_yaw = 0.0

            # Extract scan timestamp (ROS time as float seconds)
            scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self.dynamic_predictor.update_from_scan(
                self.scan_ranges, self.scan_angles,
                robot_x, robot_y, robot_yaw,
                robot_vx, robot_vy,
                sensor_x, sensor_y, sensor_yaw,
                scan_stamp              # new argument
            )

        self.enhanced_obstacle_detection()
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
        """IMPROVED: Smarter stuck detection with context awareness"""
        if not self.enable_stuck_recovery or self.goal_stopped or not self.path:
            return
        
        if not hasattr(self, 'goal_received_time') or self.goal_received_time is None:
            return
        
        # Grace period: 3 seconds after goal (robot needs time to accelerate)
        time_since_goal = time.monotonic() - self.goal_received_time
        if time_since_goal < 3.0:
            return
        
        # Need position samples over a LONGER window (5 seconds = 50 samples at 10Hz)
        if len(self.position_history) < 25:
            return
        
        # Calculate movement over 5-second window
        recent_positions = list(self.position_history)[-25:]
        max_distance = 0.0
        for i in range(len(recent_positions)):
            for j in range(i+1, len(recent_positions)):
                pos1, time1 = recent_positions[i]
                pos2, time2 = recent_positions[j]
                dist = math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])
                max_distance = max(max_distance, dist)
        
        # CRITICAL: Check if we're moving slowly OR not moving
        # Low movement + obstacles + time = stuck
        if max_distance < 0.15:  # 15cm in 5 seconds is very slow
            if not self.stuck_detected:
                self.stuck_detected = True
                self.stuck_start_time = time.monotonic()
                if self.debug_mode:
                    self.get_logger().info(f"⚠️ Low movement detected: {max_distance:.3f}m in 5s")
                return
            
            # Only trigger recovery after 5 seconds of low movement
            current_time = time.monotonic()
            if (current_time - self.stuck_start_time > 5.0 and
                self.consecutive_obstacle_readings >= self.min_obstacle_presence):
                
                # VERIFY forward is actually blocked before recovery
                if self.has_forward_clearance(min_clearance=0.8):
                    if self.debug_mode:
                        self.get_logger().info("✅ Forward path IS clear (>0.8m) - NOT stuck")
                    self.stuck_detected = False
                    return
                
                if self.debug_mode:
                    self.get_logger().warn("🔴 STUCK CONFIRMED! Initiating recovery...")
                self.initiate_recovery()
        else:
            # Robot is moving well
            if self.stuck_detected:
                self.stuck_detected = False
                self.stuck_start_time = None
                if self.debug_mode:
                    self.get_logger().debug("✅ Robot moving again, recovery cancelled")


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

    def _get_forward_clearance(self):
        """Minimum distance in ±35° front cone (matches emergency stop)."""
        if not self.scan_ranges or not self.scan_angles:
            return 0.0
        BODY_CONE_DEG = 35.0
        body_cone_rad = math.radians(BODY_CONE_DEG)
        forward_ranges = []
        for a, r in zip(self.scan_angles, self.scan_ranges):
            if abs(a) < body_cone_rad and 0.1 < r < 50.0:
                forward_ranges.append(r)
        return min(forward_ranges) if forward_ranges else 0.0


    def execute_recovery(self):
        """Goal‑directed recovery: turn toward goal, then move forward slowly."""
        if not self.recovery_mode or not self.odom:
            return True

        current_time = self.get_clock().now()
        if self.recovery_start_time is None:
            self.recovery_start_time = current_time
            self.recovery_stage = 0
            self.get_logger().warn("🔄 Starting goal‑directed recovery")

        elapsed = (current_time - self.recovery_start_time).nanoseconds * 1e-9

        # Stage 0: stop and assess (0.5s)
        if self.recovery_stage == 0:
            self.publish_cmd(0.0, 0.0)
            if elapsed > 0.5:
                self.recovery_stage = 1
            return False

        # Stage 1: rotate toward goal (up to 3s)
        elif self.recovery_stage == 1:
            if self.goal_position is None:
                self.recovery_stage = 3  # skip to backup
                return False

            robot_x = self.odom.pose.pose.position.x
            robot_y = self.odom.pose.pose.position.y
            robot_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            goal_x, goal_y = self.goal_position
            goal_dir = math.atan2(goal_y - robot_y, goal_x - robot_x)
            heading_err = math.atan2(math.sin(goal_dir - robot_yaw),
                                     math.cos(goal_dir - robot_yaw))

            if abs(heading_err) < math.radians(10):
                # Aligned enough – move to forward probe
                self.recovery_stage = 2
                self.recovery_start_time = current_time
                return False

            # Rotate toward goal
            w = 0.5 if heading_err > 0 else -0.5
            self.publish_cmd(0.0, w)
            if elapsed > 3.0:
                self.recovery_stage = 2
            return False

        # Stage 2: forward probe (2s)
        elif self.recovery_stage == 2:
            forward_clearance = self._get_forward_clearance()
            if forward_clearance > 0.5:
                self.publish_cmd(0.1, 0.0)
                if elapsed > 2.0:
                    self.recovery_mode = False
                    self.get_logger().info("✅ Recovery complete – forward clear")
                    return True
            else:
                # Forward still blocked – go to backup
                self.recovery_stage = 3
                self.recovery_start_time = current_time
            return False

        # Stage 3: backward escape (last resort, 2s)
        elif self.recovery_stage == 3:
            back_clear = self.get_backward_clearance()
            if back_clear > 0.5:
                self.publish_cmd(-0.15, 0.0)
                if elapsed > 2.0:
                    self.recovery_mode = False
                    self.get_logger().info("✅ Recovery complete – backed up")
                    return True
            else:
                # No space – just give up and stop
                self.publish_cmd(0.0, 0.0)
                self.recovery_mode = False
                self.get_logger().warn("⚠️ Recovery failed – no space to escape")
                return True
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
    def check_predicted_collision_ahead(self, lookahead_distance: float = 1.0) -> bool:
        """
        Check if collision will occur within lookahead distance
        accounting for predicted obstacle movement
        """
        if self.dynamic_predictor is None or self.odom is None:
            return False

        robot_x = self.odom.pose.pose.position.x
        robot_y = self.odom.pose.pose.position.y
        robot_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)

        # Project ahead
        lookahead_x = robot_x + lookahead_distance * math.cos(robot_yaw)
        lookahead_y = robot_y + lookahead_distance * math.sin(robot_yaw)

        robot_radius = 0.35

        for obs_id, obstacle in self.dynamic_predictor.get_obstacles().items():
            if obstacle.confidence < 0.5:
                continue

            # Predict obstacle position
            pred_x, pred_y = obstacle.predict_position(0.2)  # 200ms ahead

            # Check collision at lookahead point
            collision_dist = robot_radius + obstacle.radius + 0.3
            actual_dist = math.hypot(lookahead_x - pred_x, lookahead_y - pred_y)

            if actual_dist < collision_dist:
                self.last_predicted_collision = (obs_id, actual_dist)
                return True

        return False

    def apply_predictive_emergency_brake(self) -> bool:
        """
        Apply emergency braking if collision predicted within 300ms
        Returns True if braking applied
        """
        if self.dynamic_predictor is None or self.odom is None:
            return False

        robot_x = self.odom.pose.pose.position.x
        robot_y = self.odom.pose.pose.position.y

        robot_radius = 0.35
        prediction_time = 0.3  # 300ms

        for obstacle in self.dynamic_predictor.get_high_confidence_obstacles():
            pred_x, pred_y = obstacle.predict_position(prediction_time)
            collision_dist = robot_radius + obstacle.radius
            actual_dist = math.hypot(robot_x - pred_x, robot_y - pred_y)

            if actual_dist < collision_dist:
                self.get_logger().warn(
                    f"🚨 PREDICTED COLLISION in {prediction_time}s: "
                    f"obstacle at ({pred_x:.2f}, {pred_y:.2f}), dist={actual_dist:.2f}m"
                )
                return True

        return False
    def validate_path_safety(self, path_array, min_clearance=0.35):
        """
        Validate path safety with adaptive clearance thresholds.
        Allows tighter clearances as robot approaches goal.
        """
        if not path_array or len(path_array) < 2:
            return True, None
        
        robot_pos = np.array([self.robot_x, self.robot_y])
        goal_pos = np.array(path_array[-1])
        distance_to_goal = np.linalg.norm(goal_pos - robot_pos)
        
        # Adaptive clearance: tighter near goal, more conservative at start
        if distance_to_goal < 0.5:
            required_clearance = 0.25  # Allow tighter fit near goal
        elif distance_to_goal < 1.0:
            required_clearance = 0.30
        else:
            required_clearance = min_clearance  # 0.35m for normal navigation
        
        # Check each waypoint
        for idx, waypoint in enumerate(path_array):
            # Get closest obstacle to this waypoint
            min_dist_to_obstacle = self.get_distance_to_nearest_obstacle(waypoint[0], waypoint[1])
            
            # Allow slightly tighter clearance for middle waypoints vs start/end
            waypoint_clearance_threshold = required_clearance
            if idx == 0 or idx == len(path_array) - 1:
                # Start/end waypoints: slightly more conservative
                waypoint_clearance_threshold = required_clearance + 0.05
            
            # Check if clearance is sufficient
            if min_dist_to_obstacle < waypoint_clearance_threshold - 0.02:  # 2cm tolerance
                self.get_logger().warn(
                    f"🚨 UNSAFE path rejected! Waypoint {idx} too close to obstacle "
                    f"(clearance={min_dist_to_obstacle:.2f}m < {waypoint_clearance_threshold:.2f}m)"
                )
                return False, idx
        
        return True, None
    def get_distance_to_nearest_obstacle(self, x, y, search_radius=0.8):
        """
        Get distance to nearest obstacle with search radius limit.
        Returns distance or search_radius if no obstacle found.
        """
        min_distance = search_radius
        robot_radius = 0.25  # Scout Mini approximate radius
        
        # Check costmap around point
        grid_x = int((x - self.map_origin[0]) / self.grid_resolution)
        grid_y = int((y - self.map_origin[1]) / self.grid_resolution)
        
        search_cells = int(search_radius / self.grid_resolution)
        
        for dx in range(-search_cells, search_cells + 1):
            for dy in range(-search_cells, search_cells + 1):
                cx = grid_x + dx
                cy = grid_y + dy
                
                # Check bounds
                if 0 <= cx < self.costmap.shape[0] and 0 <= cy < self.costmap.shape[1]:
                    if self.costmap[cx, cy] > 0.5:  # Occupied
                        # Calculate actual distance
                        cell_x = self.map_origin[0] + cx * self.grid_resolution
                        cell_y = self.map_origin[1] + cy * self.grid_resolution
                        dist = np.sqrt((cell_x - x)**2 + (cell_y - y)**2)
                        dist = max(0, dist - robot_radius)  # Account for robot radius
                        min_distance = min(min_distance, dist)
        
        return min_distance
    def path_callback(self, msg: Path):
        """Handle new path from optimizer – always accept, never reject."""
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path!")
            return

        # Skip if goal already reached
        if self.goal_stopped:
            if self.debug_mode:
                self.get_logger().debug("Ignoring path - goal already reached and stopped")
            return

        # Update timestamp – always accept the path
        self.last_path_update = self.get_clock().now()
        self.path_received = True

        # Compute path length
        path_length = 0.0
        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i + 1].pose.position
            path_length += math.hypot(p2.x - p1.x, p2.y - p1.y)

        start_x = msg.poses[0].pose.position.x
        start_y = msg.poses[0].pose.position.y
        goal_x = msg.poses[-1].pose.position.x
        goal_y = msg.poses[-1].pose.position.y

        if self.debug_mode:
            self.get_logger().info(
                f"📈 New path received: {len(msg.poses)} waypoints, {path_length:.2f}m"
            )
            self.get_logger().info(f"  Start: ({start_x:.3f}, {start_y:.3f})")
            self.get_logger().info(f"  Goal: ({goal_x:.3f}, {goal_y:.3f})")

        # Log start deviation but never reject
        if self.odom is not None:
            robot_x = self.odom.pose.pose.position.x
            robot_y = self.odom.pose.pose.position.y
            start_deviation = math.hypot(start_x - robot_x, start_y - robot_y)
            if start_deviation > self.path_staleness_warning_threshold:
                self.get_logger().warn(
                    f"⚠️ Path start deviates {start_deviation:.2f}m from robot – "
                    f"still accepting, controller will adapt"
                )

        # Accept path
        self.path = msg
        self.current_waypoint_idx = 0
        self.dwa_failures = 0
        self.simple_control_active = False
    def calculate_waypoint_velocity_and_turn(self):
        """Calculate velocity and turning rate to follow current waypoint"""
        if self.path is None or not self.path_received or len(self.path.poses) == 0:
            return 0.0, 0.0
        
        if self.odom is None:
            return 0.0, 0.0
        
        # Get current pose
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
        
        # Find the best waypoint to track (lookahead)
        best_waypoint_idx = self.current_waypoint_idx
        min_distance = float('inf')
        lookahead_dist = self.lookahead_distance
        
        # Search forward for waypoint at lookahead distance
        for i in range(self.current_waypoint_idx, min(self.current_waypoint_idx + 10, len(self.path.poses))):
            wp = self.path.poses[i].pose.position
            wp_dist = math.hypot(wp.x - x, wp.y - y)
            
            # Prefer waypoint that is at least lookahead_dist away but not too far
            if wp_dist >= lookahead_dist and wp_dist < min_distance:
                best_waypoint_idx = i
                min_distance = wp_dist
            elif wp_dist < lookahead_dist:
                # Keep searching, but update as closest
                if wp_dist < min_distance:
                    min_distance = wp_dist
                    best_waypoint_idx = i
        
        # Update current waypoint index
        if best_waypoint_idx > self.current_waypoint_idx:
            self.current_waypoint_idx = best_waypoint_idx
        
        # Get target waypoint
        target_wp = self.path.poses[self.current_waypoint_idx].pose.position
        target_x = target_wp.x
        target_y = target_wp.y
        
        # Calculate direction to waypoint
        dx = target_x - x
        dy = target_y - y
        distance_to_waypoint = math.hypot(dx, dy)
        
        # Calculate desired heading
        desired_yaw = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))
        heading_error_deg = math.degrees(heading_error)
        
        # Adaptive velocity based on distance and heading error
        if distance_to_waypoint < 0.2:
            # Very close to waypoint, move to next
            if self.current_waypoint_idx < len(self.path.poses) - 1:
                self.current_waypoint_idx += 1
            v = 0.1
            w = 0.0
        elif abs(heading_error) > self.rotate_in_place_angle_deg * (math.pi / 180):
            # Large heading error, rotate in place
            v = 0.05  # Reduced forward speed during rotation
            w = 0.5 if heading_error > 0 else -0.5
        else:
            # Normal following behavior
            # Reduce speed if heading error is large
            heading_gain = max(0.3, 1.0 - (abs(heading_error_deg) / 90.0))
            v = self.max_linear_vel * heading_gain * 0.7  # 70% of max
            
            # Angular velocity from heading error
            w = self.heading_kp * heading_error
            w = max(-self.max_angular_vel, min(self.max_angular_vel, w))
        
        return v, w
    def control_loop_with_pinn_optimization(self):
        """
        Main control loop with PINN-optimized path following and dynamic obstacle handling.
        """
        self.control_counter += 1

        # Update obstacle detection
        self.enhanced_obstacle_detection()

        if not self.odom:
            return

        robot_x = self.odom.pose.pose.position.x
        robot_y = self.odom.pose.pose.position.y
        robot_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
        robot_vx = self.odom.twist.twist.linear.x
        robot_vy = self.odom.twist.twist.linear.y

        # No goal set - maintain position
        if not self.has_explicit_goal or self.goal_position is None:
            self.publish_cmd(0.0, 0.0)
            return

        goal_x, goal_y = self.goal_position
        dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # Check goal reached
        if dist_to_goal < self.goal_completion_distance:
            if self.goal_stopped:
                if not self.goal_reached:
                    self.get_logger().info(f"✅ GOAL REACHED at ({robot_x:.2f}, {robot_y:.2f})")
                    self.goal_reached = True
                    self._log_anomaly('GOAL_REACHED', 0.0, 0.0, 0.0, self.obstacle_distance)
                self.publish_cmd(0.0, 0.0)
            else:
                self.get_logger().info("🎯 At goal - stopping")
                self.goal_stopped = True
                self.goal_reached_time = self.get_clock().now()
            return
        else:
            self.goal_stopped = False

        # ========== NEW: Dynamic obstacle threat assessment ==========
        threat = 0.0
        evade_angle = None
        if self.dynamic_predictor is not None:
            threat = self.dynamic_predictor.get_threat_level(
                robot_x, robot_y, goal_x, goal_y, abs(robot_vx)
            )
            if threat > 0.6:
                evade_angle = self.dynamic_predictor.get_suggested_evasion_direction(
                    robot_x, robot_y, robot_yaw, goal_x, goal_y
                )
                self.get_logger().error(
                    f"🆘 CRITICAL THREAT {threat:.2f} – activating evasion"
                )
            elif threat > 0.3:
                self.get_logger().info(
                    f"⚠️ Moderate threat {threat:.2f} – reducing speed"
                )
        # ============================================================

        # No path yet - wait or request from optimizer
        if self.path is None or len(self.path.poses) == 0:
            if self.control_counter % 20 == 0:
                self.get_logger().info(
                    f"⏳ Waiting for path: goal={goal_x:.2f},{goal_y:.2f} dist={dist_to_goal:.2f}m"
                )
            self.publish_cmd(0.0, 0.0)
            return

        # Find nearest path point ahead
        current_waypoint_idx = self.find_nearest_path_point(robot_x, robot_y)

        # Look ahead for target waypoint
        lookahead_idx = min(current_waypoint_idx + 2, len(self.path.poses) - 1)
        target_pose = self.path.poses[lookahead_idx]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        # Calculate required heading
        goal_dir = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_err = math.atan2(math.sin(goal_dir - robot_yaw), math.cos(goal_dir - robot_yaw))
        heading_err_deg = math.degrees(heading_err)

        # Check obstacle clearance
        front_clearance = self.get_current_clearance()

        # If emergency escape is already active, do not check again – let it run
        if self.emergency_escape_active:
            self.execute_emergency_escape()
            return

        # Otherwise, check if we need to activate escape
        if front_clearance < self.emergency_stop_distance:
            self.get_logger().error(
                f"🚨 EMERGENCY STOP TRIGGER: clearance={front_clearance:.2f}m < {self.emergency_stop_distance}m, threat={threat:.2f}"
            )
            self.execute_emergency_escape()
            return

        # ========== NEW: Speed scaling based on threat ==========
        if threat > 0.3:
            # Reduce speed proportionally to threat (0.3->0.8 scale, 0.6->0.5 scale)
            speed_scale = max(0.3, 1.0 - threat)
            if self.control_counter % 20 == 0:
                self.get_logger().info(
                    f"🐢 Threat-based speed scaling: threat={threat:.2f} → scale={speed_scale:.2f}"
                )
        else:
            # Reactive speed based on clearance (existing)
            if front_clearance < self.slow_down_distance:
                speed_scale = 0.5
                self.get_logger().debug(f"🐢 Slow down: clearance={front_clearance:.2f}m < {self.slow_down_distance}m")
            else:
                speed_scale = 1.0
        # =======================================================

        # Generate motor commands
        v_cmd = 0.0
        w_cmd = 0.0

        # If threat is critical, override normal control with evasion
        if threat > 0.6 and evade_angle is not None:
            heading_err_evade = math.atan2(math.sin(evade_angle - robot_yaw), math.cos(evade_angle - robot_yaw))
            w_cmd = 0.8 if heading_err_evade > 0 else -0.8
            v_cmd = 0.1
            self.get_logger().warn(
                f"🔄 EVASION ACTIVE: threat={threat:.2f}, turn={math.degrees(heading_err_evade):.1f}°, "
                f"v={v_cmd:.2f}, w={w_cmd:.2f}"
            )
        else:
            # Heading control (turn first if angle error is large)
            if abs(heading_err_deg) > self.rotate_in_place_angle_deg:
                # Turn in place - significant heading error
                w_cmd = 0.5 * math.copysign(1.0, heading_err)
                v_cmd = 0.1
                self.get_logger().debug(
                    f"🔄 Rotating: heading_err={heading_err_deg:.1f}°, w={w_cmd:.2f}"
                )
            else:
                # Forward motion with gentle heading correction
                v_cmd = self.max_linear_vel * speed_scale * 0.7
                w_cmd = self.heading_kp * heading_err
                w_cmd = np.clip(w_cmd, -self.max_angular_vel, self.max_angular_vel)

        # Track command history for motor health
        self.commanded_velocity_history.append(abs(v_cmd))
        self.last_commanded_linear = abs(v_cmd)
        self.angular_history.append(w_cmd)
        self.linear_history.append(v_cmd)

        # Publish command
        self.publish_cmd(v_cmd, w_cmd)

        # Log state occasionally
        if self.control_counter % 30 == 0:
            self.get_logger().info(
                f"📤 CMD v={v_cmd:.3f} w={w_cmd:.3f} | "
                f"heading_err={heading_err_deg:.1f}° "
                f"goal_dir={math.degrees(goal_dir):.1f}° "
                f"goal_dist={dist_to_goal:.2f}m "
                f"clearance={front_clearance:.2f}m "
                f"threat={threat:.2f}"
            )
    def find_nearest_path_point(self, robot_x, robot_y):
        """
        Find the nearest point on the path to the robot.
        Start searching from last known index.
        """
        if self.path is None or len(self.path.poses) == 0:
            return 0
        
        min_dist = float('inf')
        nearest_idx = 0
        
        # Search around last index first (efficiency)
        start_search = max(0, self.current_waypoint_idx - 5)
        end_search = min(len(self.path.poses), self.current_waypoint_idx + 15)
        
        for i in range(start_search, end_search):
            pose = self.path.poses[i]
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            dist = math.hypot(dx, dy)
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        self.current_waypoint_idx = nearest_idx
        return nearest_idx
    def calculate_simple_control(self, target_waypoint, current_x, current_y, current_yaw):
        """Simple proportional control – must return (v, w)."""
        if target_waypoint is None:
            return 0.0, 0.0

        dx = target_waypoint.x - current_x
        dy = target_waypoint.y - current_y
        distance = math.hypot(dx, dy)

        # Bearing to target
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Proportional control
        v = min(self.max_linear_vel * 0.5, distance * 0.5)
        w = angle_diff * 1.5   # gain
        w = np.clip(w, -self.max_angular_vel, self.max_angular_vel)

        return v, w
    def get_current_clearance(self):
        """Same ±35° front cone for consistent obstacle detection."""
        if not self.scan_ranges or not self.scan_angles:
            return float('inf')
        BODY_CONE_DEG = 35.0
        body_cone_rad = math.radians(BODY_CONE_DEG)
        min_clearance = float('inf')
        for a, r in zip(self.scan_angles, self.scan_ranges):
            if abs(a) < body_cone_rad and 0.1 < r < 50.0:
                min_clearance = min(min_clearance, r)
        return min_clearance if math.isfinite(min_clearance) else float('inf')
    def get_backward_clearance(self):
        """Minimum distance in ±35° rear cone."""
        if not self.scan_ranges or not self.scan_angles:
            return 0.0
        BODY_CONE_DEG = 35.0
        backward_ranges = []
        for a, r in zip(self.scan_angles, self.scan_ranges):
            diff = abs(math.atan2(math.sin(a - math.pi), math.cos(a - math.pi)))
            if diff < math.radians(BODY_CONE_DEG) and 0.1 < r < 50.0:
                backward_ranges.append(r)
        return min(backward_ranges) if backward_ranges else 0.0

    def get_side_clearances(self):
        """Return (left_clearance, right_clearance) for angles ±π/2."""
        left_clear = self.get_clearance_in_direction(math.pi/2)
        right_clear = self.get_clearance_in_direction(-math.pi/2)
        return left_clear, right_clear

    def execute_emergency_escape(self):
        """
        Full escape sequence: turn toward goal → turn opposite → backup (with continuous rear check)
        → rotate to goal → cautious forward. Never exits early. Only exits after moving 0.4 m
        with clearance > 0.9 m for 1 second.
        """
        current_time = time.monotonic()

        if not self.emergency_escape_active:
            self.emergency_escape_active = True
            self.emergency_escape_start_time = current_time
            self.escape_stage = 0          # 0=turn_toward, 1=turn_opposite, 2=backup, 3=rotate_to_goal, 4=cautious
            self.escape_stage_start = current_time
            self.turn_attempts = 0
            self.cautious_forward_start = None
            self.cautious_forward_dist = 0.0
            if self.odom:
                self.escape_start_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            else:
                self.escape_start_yaw = 0.0
            self.get_logger().error(
                f"🔴 EMERGENCY ESCAPE INITIATED | dist={self.obstacle_distance:.2f}m "
                f"dir={math.degrees(self.obstacle_direction):.1f}°"
            )
            return False

        # Global timeout: 12 seconds
        if current_time - self.emergency_escape_start_time > 12.0:
            self.get_logger().error("⏰ Emergency escape timeout – stopping")
            self.publish_cmd(0.0, 0.0)
            self.emergency_escape_active = False
            return True

        elapsed = current_time - self.escape_stage_start

        # --- Stage 0: Turn toward goal (max 60° or 1.2 s) ---
        if self.escape_stage == 0:
            if self.odom is None:
                return False

            if self.goal_position:
                robot_x = self.odom.pose.pose.position.x
                robot_y = self.odom.pose.pose.position.y
                robot_yaw = self.escape_start_yaw
                goal_x, goal_y = self.goal_position
                goal_dir = math.atan2(goal_y - robot_y, goal_x - robot_x)
                target_change = math.atan2(math.sin(goal_dir - robot_yaw),
                                           math.cos(goal_dir - robot_yaw))
                target_change = np.clip(target_change, -math.radians(60), math.radians(60))
            else:
                target_change = math.radians(60)

            current_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            yaw_change = current_yaw - self.escape_start_yaw
            yaw_change = math.atan2(math.sin(yaw_change), math.cos(yaw_change))

            if (target_change > 0 and yaw_change < target_change) or (target_change < 0 and yaw_change > target_change):
                if elapsed < 1.2:
                    w = 0.8 if target_change > 0 else -0.8
                    self.publish_cmd(0.0, w)
                    return False

            # Turn finished – move to opposite turn (no exit)
            self.escape_stage = 1
            self.escape_stage_start = current_time
            if self.odom:
                self.escape_start_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            return False

        # --- Stage 1: Turn opposite (away from goal) ---
        if self.escape_stage == 1:
            if self.odom is None:
                return False

            if self.goal_position:
                robot_x = self.odom.pose.pose.position.x
                robot_y = self.odom.pose.pose.position.y
                robot_yaw = self.escape_start_yaw
                goal_x, goal_y = self.goal_position
                goal_dir = math.atan2(goal_y - robot_y, goal_x - robot_x)
                target_change = -math.atan2(math.sin(goal_dir - robot_yaw),
                                            math.cos(goal_dir - robot_yaw))
                target_change = np.clip(target_change, -math.radians(60), math.radians(60))
            else:
                target_change = -math.radians(60)

            current_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            yaw_change = current_yaw - self.escape_start_yaw
            yaw_change = math.atan2(math.sin(yaw_change), math.cos(yaw_change))

            if (target_change > 0 and yaw_change < target_change) or (target_change < 0 and yaw_change > target_change):
                if elapsed < 1.2:
                    w = 0.8 if target_change > 0 else -0.8
                    self.publish_cmd(0.0, w)
                    return False

            # Both turns done – move to backup (no exit)
            self.escape_stage = 2
            self.escape_stage_start = current_time
            # Reset backup distance tracking
            for attr in ['_backup_start_pose', '_backup_distance']:
                if hasattr(self, attr):
                    delattr(self, attr)
            return False

        # --- Stage 2: Backup with continuous rear clearance check ---
        if self.escape_stage == 2:
            back_clear = self.get_backward_clearance()

            # If rear becomes blocked, stop backing and move to next stage
            if back_clear < 0.6:
                self.get_logger().warn("Rear clearance lost – stopping backup")
                self.escape_stage = 3
                self.escape_stage_start = current_time
                return False

            # Record start pose for distance measurement (if odometry works)
            if not hasattr(self, '_backup_start_pose') and self.odom:
                self._backup_start_pose = (
                    self.odom.pose.pose.position.x,
                    self.odom.pose.pose.position.y
                )
                self._backup_distance = 0.0

            if self.odom and hasattr(self, '_backup_start_pose'):
                x = self.odom.pose.pose.position.x
                y = self.odom.pose.pose.position.y
                dx = x - self._backup_start_pose[0]
                dy = y - self._backup_start_pose[1]
                self._backup_distance = math.hypot(dx, dy)

            # Stop backing if moved 0.6 m or time exceeds 2.5 seconds
            if (hasattr(self, '_backup_distance') and self._backup_distance > 0.6) or elapsed > 2.5:
                self.get_logger().info(f"Backup complete – moved {self._backup_distance:.2f}m")
                for attr in ['_backup_start_pose', '_backup_distance']:
                    if hasattr(self, attr):
                        delattr(self, attr)
                self.escape_stage = 3
                self.escape_stage_start = current_time
                return False

            # Continue backing up (slightly faster)
            self.publish_cmd(-0.3, 0.0)
            return False

        # --- Stage 3: Rotate toward goal (max 2.5 s) ---
        if self.escape_stage == 3:
            if self.goal_position is None or self.odom is None:
                self.escape_stage = 4
                self.escape_stage_start = current_time
                self.cautious_forward_start = None
                return False

            robot_x = self.odom.pose.pose.position.x
            robot_y = self.odom.pose.pose.position.y
            robot_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
            goal_x, goal_y = self.goal_position
            goal_dir = math.atan2(goal_y - robot_y, goal_x - robot_x)
            heading_err = math.atan2(math.sin(goal_dir - robot_yaw),
                                     math.cos(goal_dir - robot_yaw))

            if abs(heading_err) < math.radians(10):
                self.escape_stage = 4
                self.escape_stage_start = current_time
                self.cautious_forward_start = None
                return False

            if elapsed < 2.5:
                w = 0.6 if heading_err > 0 else -0.6
                self.publish_cmd(0.0, w)
                return False
            else:
                self.escape_stage = 4
                self.escape_stage_start = current_time
                self.cautious_forward_start = None
                return False

        # --- Stage 4: Cautious forward with path following (reduced speed) ---
        if self.escape_stage == 4:
            # Record starting pose if not set (for distance measurement)
            if self.cautious_forward_start is None and self.odom:
                self.cautious_forward_start = (
                    self.odom.pose.pose.position.x,
                    self.odom.pose.pose.position.y
                )
                self.cautious_forward_dist = 0.0

            # --- Use normal path following, but at slow speed ---
            # If we have a path, compute velocity commands that follow the path
            v_cmd = 0.0
            w_cmd = 0.0
            if self.path is not None and len(self.path.poses) > 0:
                # Find nearest path point and lookahead (same as in normal control)
                current_waypoint_idx = self.find_nearest_path_point(
                    self.odom.pose.pose.position.x,
                    self.odom.pose.pose.position.y
                )
                lookahead_idx = min(current_waypoint_idx + 2, len(self.path.poses) - 1)
                target_pose = self.path.poses[lookahead_idx]
                target_x = target_pose.pose.position.x
                target_y = target_pose.pose.position.y

                # Calculate required heading to the target waypoint
                robot_x = self.odom.pose.pose.position.x
                robot_y = self.odom.pose.pose.position.y
                robot_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
                goal_dir = math.atan2(target_y - robot_y, target_x - robot_x)
                heading_err = math.atan2(math.sin(goal_dir - robot_yaw),
                                         math.cos(goal_dir - robot_yaw))

                # Slow speed path following
                v_cmd = 0.1  # fixed slow forward speed
                w_cmd = np.clip(heading_err * 1.2, -0.5, 0.5)  # gentle correction
            else:
                # Fallback: goal‑directed steering if no path
                v_cmd = 0.1
                w_cmd = 0.0
                if self.goal_position and self.odom:
                    robot_x = self.odom.pose.pose.position.x
                    robot_y = self.odom.pose.pose.position.y
                    robot_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
                    goal_x, goal_y = self.goal_position
                    goal_dir = math.atan2(goal_y - robot_y, goal_x - robot_x)
                    heading_err = math.atan2(math.sin(goal_dir - robot_yaw),
                                             math.cos(goal_dir - robot_yaw))
                    w_cmd = np.clip(heading_err * 0.5, -0.3, 0.3)

            self.publish_cmd(v_cmd, w_cmd)

            # Update distance travelled
            if self.odom and self.cautious_forward_start:
                x = self.odom.pose.pose.position.x
                y = self.odom.pose.pose.position.y
                dx = x - self.cautious_forward_start[0]
                dy = y - self.cautious_forward_start[1]
                self.cautious_forward_dist = math.hypot(dx, dy)

            # Check forward clearance (still monitor obstacles)
            forward_clear = self._get_forward_clearance()

            # If obstacle reappears, restart the whole escape
            if forward_clear < self.emergency_stop_distance:
                self.turn_attempts += 1
                self.get_logger().warn(
                    f"⚠️ Obstacle reappeared (attempt {self.turn_attempts}) – restarting escape"
                )
                self.escape_stage = 0
                self.escape_stage_start = current_time
                if self.odom:
                    self.escape_start_yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
                self.cautious_forward_start = None
                return False

            # Exit condition: moved 0.4 m AND clearance > 0.9 m for 1 second
            if self.cautious_forward_dist > 0.4:
                if not hasattr(self, '_cf_clear_start') or self._cf_clear_start is None:
                    self._cf_clear_start = current_time
                elif current_time - self._cf_clear_start > 1.0:
                    self.get_logger().info(
                        f"✅ Escape successful – moved {self.cautious_forward_dist:.2f}m, "
                        f"clearance {forward_clear:.2f}m – resuming normal control"
                    )
                    self.emergency_escape_active = False
                    for attr in ['_cf_clear_start', '_clear_timer']:
                        if hasattr(self, attr):
                            delattr(self, attr)
                    return True
            else:
                self._cf_clear_start = None

            # Timeout for cautious phase (5 seconds)
            if elapsed > 5.0:
                self.get_logger().error("Cautious forward timeout – giving up")
                self.publish_cmd(0.0, 0.0)
                self.emergency_escape_active = False
                return True

            return False
    def _reset_escape_state(self):
        """Reset internal escape state variables."""
        for attr in ['_escape_phase', '_escape_phase_start', '_escape_phase_duration',
                     '_escape_turn_duration', '_escape_backup_duration',
                     '_escape_turn_goal_duration', '_escape_attempted_backup']:
            if hasattr(self, attr):
                delattr(self, attr)
        if hasattr(self, 'emergency_escape_start_time'):
            delattr(self, 'emergency_escape_start_time')
    def _attempt_backup_escape(self):
        """
        Attempt to escape by backing up. Returns True if backup was commanded.
        """
        # Check rear clearance (using the same method as forward clearance, but behind)
        back_clearance = self.get_backward_clearance()
        if back_clearance < 0.8:
            self.get_logger().warn(f"↩️ Cannot backup – rear clearance too low ({back_clearance:.2f}m)")
            return False

        self.get_logger().info(f"↩️ Backing up (clearance={back_clearance:.2f}m) to escape")
        self.publish_cmd(-0.2, 0.0)   # backup at 0.2 m/s
        # Reset the rotation attempt counter so that after backing up we try rotating again
        self.emergency_escape_rotate_attempts = 0
        return True
    def _get_best_rotation_toward_goal(self, heading_err):
        """
        Determine the best rotation direction (sign) to turn toward the goal,
        considering free space clearance on left and right.
        Returns 1 for left (positive), -1 for right (negative), or 0 if already aligned.
        """
        if abs(heading_err) < math.radians(10):
            return 0.0

        # Get clearance on left and right sides (using laser scan)
        left_clear = self.get_clearance_in_direction(math.pi / 2)
        right_clear = self.get_clearance_in_direction(-math.pi / 2)

        # If both sides have ample clearance, turn in the shortest direction to goal
        if left_clear > 0.8 and right_clear > 0.8:
            return 1.0 if heading_err > 0 else -1.0

        # Prefer the side with more clearance
        if left_clear > right_clear:
            return 1.0   # turn left
        elif right_clear > left_clear:
            return -1.0  # turn right
        else:
            # Equal clearance – turn toward goal
            return 1.0 if heading_err > 0 else -1.0

    def _get_best_rotation_toward_goal(self, heading_err):
        """
        Determine the best rotation direction (sign) to turn toward the goal,
        considering free space clearance on left and right.
        Returns 1 for left (positive), -1 for right (negative), or 0 if already aligned.
        """
        if abs(heading_err) < math.radians(10):
            return 0.0

        # Get clearance on left and right sides (using laser scan)
        left_clear = self.get_clearance_in_direction(math.pi / 2)
        right_clear = self.get_clearance_in_direction(-math.pi / 2)

        # If both sides have ample clearance, turn in the shortest direction to goal
        if left_clear > 0.8 and right_clear > 0.8:
            return 1.0 if heading_err > 0 else -1.0

        # Prefer the side with more clearance
        if left_clear > right_clear:
            return 1.0   # turn left
        elif right_clear > left_clear:
            return -1.0  # turn right
        else:
            # Equal clearance – turn toward goal
            return 1.0 if heading_err > 0 else -1.0
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
        import threading   # ADD THIS LINE

        try:
            use_pinn_in_controller = self.get_parameter('use_pinn_in_controller').value
        except:
            use_pinn_in_controller = False

        if use_pinn_in_controller:
            # Initialize threading lock for PINN futures
            self.pinn_future_lock = threading.Lock()
            self.pinn_futures = {}
            self.pinn_service_available = False
            return self.setup_pinn_client_in_controller()

        return False

    # Watchdog constants – kept as class-level to be easy to tune.
    _WATCHDOG_MIN_HEADING_ERROR = math.radians(20)  # trigger only above this heading error
    _WATCHDOG_W_SCALE = 0.6                         # fraction of max_w for forced rotation
    _WATCHDOG_HEADING_GAIN = 0.8                    # proportional gain on heading error

    def _apply_zero_vel_watchdog(self, v: float, w: float,
                                  x: float, y: float, yaw: float):
        """Zero-velocity watchdog: force rotation-in-place when v≈0 persists too long.

        When DWA/simple-control has been producing v≈0 for longer than
        ``_zero_vel_watchdog_timeout`` seconds AND the heading error to the goal
        is large, the robot is likely stuck in a local minimum.  We override the
        command with a pure rotation toward the goal to break the deadlock.

        The watchdog is suppressed when:
        - Emergency stop is active (obstacle_detected and obstacle_distance < stop_dist)
        - Goal is not set or already reached
        """
        _is_estop = (self.obstacle_detected and
                     self.obstacle_distance < getattr(self, 'emergency_stop_distance', 0.55))
        if _is_estop or self.goal_reached or self.goal_position is None:
            self._zero_vel_watchdog_start = 0.0
            self._watchdog_rotating = False
            return v, w

        # Compute heading error to goal
        gx, gy = self.goal_position
        goal_dir = math.atan2(gy - y, gx - x)
        heading_err = abs(math.atan2(math.sin(goal_dir - yaw),
                                     math.cos(goal_dir - yaw)))
        signed_heading_err = math.atan2(math.sin(goal_dir - yaw),
                                        math.cos(goal_dir - yaw))

        now = time.monotonic()
        _v_near_zero = abs(v) < 0.05 and abs(w) < 0.05

        if _v_near_zero:
            if self._zero_vel_watchdog_start == 0.0:
                self._zero_vel_watchdog_start = now
            elapsed = now - self._zero_vel_watchdog_start
            if elapsed >= self._zero_vel_watchdog_timeout:
                if heading_err > self._WATCHDOG_MIN_HEADING_ERROR:
                    # Force rotation toward goal
                    forced_w = math.copysign(
                        min(self.max_angular_vel * self._WATCHDOG_W_SCALE,
                            heading_err * self._WATCHDOG_HEADING_GAIN),
                        signed_heading_err
                    )
                    if now - self._watchdog_last_warn_time >= 5.0:
                        self._watchdog_last_warn_time = now
                        self.get_logger().warn(
                            f"🔁 Zero-vel watchdog: v=0 for {elapsed:.1f}s, "
                            f"heading_err={math.degrees(heading_err):.1f}° → forcing w={forced_w:.2f}"
                        )
                    self._watchdog_rotating = True
                    return 0.0, forced_w
        else:
            # Non-zero command → reset watchdog
            self._zero_vel_watchdog_start = 0.0
            self._watchdog_rotating = False

        return v, w

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
    def adjust_speed_for_curvature(self, v, w):
        """
        IMPROVED: Reduces speed based on path curvature to maintain stability.
        Prevents speed buildup on curved sections.
        """
        if v <= 0.01 or abs(w) < 0.05:
            return v

        # Estimate curvature: radius = v / |w|
        curvature_radius = abs(v) / max(abs(w), 0.01)

        # Tight curves (radius < 0.5m) require slower speed
        if curvature_radius < 0.3:  # Very tight
            return v * 0.3
        elif curvature_radius < 0.5:  # Tight
            return v * 0.5
        elif curvature_radius < 0.8:  # Moderate
            return v * 0.7

        return v
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

    def get_reactive_velocity_scaling(self, v, w):
        """
        IMPROVED: Scale velocity based on obstacle proximity with hysteresis.
        Uses multiple distance bands for smooth speed reduction.
        """
        if not self.scan_ranges or not self.scan_angles or v <= 0.01:
            return v, w

        # Find minimum distance in FRONT hemisphere (±90°)
        min_dist_front = float('inf')
        for i, (range_val, angle) in enumerate(zip(self.scan_ranges, self.scan_angles)):
            if abs(angle) < math.radians(90):  # Front 180°
                if 0.1 < range_val < 5.0:
                    min_dist_front = min(min_dist_front, range_val)

        # NO SCALING if no obstacles detected
        if min_dist_front >= 5.0:
            return v, w

        # CRITICAL SAFETY ZONES with hysteresis
        if min_dist_front < 0.25:  # EMERGENCY: < 25cm
            self.get_logger().error(f"🚨 EMERGENCY: {min_dist_front:.2f}m - HARD STOP")
            return 0.0, 0.0

        elif min_dist_front < 0.35:  # CRITICAL: < 35cm
            self.get_logger().warn(f"🛑 CRITICAL: {min_dist_front:.2f}m - stopping, can rotate")
            return 0.0, w * 0.8

        elif min_dist_front < 0.5:  # DANGER: < 50cm
            scale = (min_dist_front - 0.35) / (0.5 - 0.35)  # Linear from 0→1
            v_scaled = v * scale * 0.2  # Max 20% speed
            self.get_logger().warn(f"⚠️ DANGER: {min_dist_front:.2f}m - scaling v to {v_scaled:.3f}")
            return v_scaled, w * 0.9

        elif min_dist_front < 0.75:  # WARNING: < 75cm
            scale = (min_dist_front - 0.5) / (0.75 - 0.5)  # Linear from 0→1
            v_scaled = v * (0.2 + scale * 0.4)  # 20-60% speed
            if self.debug_mode and self.control_counter % 40 == 0:
                self.get_logger().info(f"⚠️ WARNING: {min_dist_front:.2f}m - scaling v to {v_scaled:.3f}")
            return v_scaled, w

        elif min_dist_front < 1.2:  # CAUTION: < 120cm
            scale = (min_dist_front - 0.75) / (1.2 - 0.75)  # Linear from 0→1
            v_scaled = v * (0.6 + scale * 0.4)  # 60-100% speed
            return v_scaled, w

        # Safe distance (> 120cm)
        return v, w
    def _apply_zero_vel_watchdog(self, v, w, x, y, yaw):
        """
        IMPROVED: Detects when DWA locks up (v≈0 but can't rotate),
        then forces rotation-in-place to break deadlock.
        """
        current_time = time.time()

        # Check if v is essentially zero (< 0.01)
        if v < 0.01:
            if self._zero_vel_watchdog_start == 0.0:
                self._zero_vel_watchdog_start = current_time
            
            elapsed = current_time - self._zero_vel_watchdog_start

            # After timeout, force rotation if heading error is large
            if elapsed > self._zero_vel_watchdog_timeout and abs(w) < 0.1:
                if self.goal_position is not None:
                    goal_x, goal_y = self.goal_position
                    goal_dir = math.atan2(goal_y - y, goal_x - x)
                    heading_err = abs(math.atan2(
                        math.sin(goal_dir - yaw), math.cos(goal_dir - yaw)))
                    
                    if heading_err > math.radians(20):  # Large heading error
                        self._watchdog_rotating = True
                        w = 0.5 if goal_dir > yaw else -0.5
                        
                        if current_time - self._watchdog_last_warn_time > 2.0:
                            self.get_logger().warn(
                                f"🔄 WATCHDOG: v≈0 for {elapsed:.1f}s, forcing rotation "
                                f"(heading_err={math.degrees(heading_err):.1f}°)"
                            )
                            self._watchdog_last_warn_time = current_time
                        
                        return v, w

        else:  # v > 0.01, reset watchdog
            self._zero_vel_watchdog_start = 0.0
            self._watchdog_rotating = False

        return v, w
    def apply_enhanced_speed_limit(self, v, w):
        """
        IMPROVED: Smart speed limiting based on curvature and obstacles.
        Reduces speed on sharp turns to prevent tipping/skidding.
        """
        if v <= 0.01:
            return v

        # Reduce speed on sharp turns (high |w|)
        angular_ratio = min(1.0, abs(w) / self.max_angular_vel)

        if angular_ratio > 0.8:  # Very sharp turn (>80% max angular velocity)
            v *= 0.4  # 40% speed only
        elif angular_ratio > 0.6:  # Sharp turn
            v *= 0.6  # 60% speed
        elif angular_ratio > 0.4:  # Moderate turn
            v *= 0.8  # 80% speed
        # else: straight (angular_ratio <= 0.4), keep full speed

        return v
    def publish_cmd(self, v_cmd, w_cmd):
        """
        Publish velocity command to robot.
        Apply calibration factors.
        """
        msg = Twist()
        msg.linear.x = float(v_cmd * self.linear_scale)
        msg.angular.z = float(w_cmd * self.angular_scale * self.angular_sign)
        self.cmd_pub.publish(msg)
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
