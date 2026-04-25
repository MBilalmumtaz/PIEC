
#!/usr/bin/env python3
"""
Complete Path Optimizer with PINN Integration - ALL FEATURES FIXED VERSION
Fixes stuck detection and escape path issues while keeping all features
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import math
import random
import time
import threading

from collections import deque, defaultdict
from concurrent.futures import ThreadPoolExecutor
# At the top of your file, add import
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path, Odometry
from .dynamic_obstacle_tracker import DynamicObstacleTracker, Obstacle
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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

# Import your modules
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from objectives import ObjectiveEvaluator
from uncertainty_aware import UncertaintyAwarePlanner
from nsga2 import fast_non_dominated_sort, crowding_distance, nsga2_selection

# Path validation constants
PATH_START_DEVIATION_THRESHOLD = 0.3  # meters – snap path start when deviation exceeds this
PATH_START_WARNING_THRESHOLD = 0.5    # meters – log a warn for moderate deviations
# When the nearest waypoint on the stale path exceeds this distance from the robot,
# generate a fresh quick path instead of trimming the stale path.
PATH_REPLAN_NEAREST_WAYPOINT_DIST = 0.5  # metres
# Reject a path entirely only when start deviation > this AND yaw error > this
PATH_REJECT_DEVIATION_THRESHOLD = 0.5    # metres
PATH_REJECT_YAW_ERROR_THRESHOLD = math.radians(30)  # 30°
# Minimum interval between path-start-deviation warnings to avoid log spam
PATH_DEVIATION_WARN_COOLDOWN = 5.0  # seconds
# Maximum wait time in PINN exponential backoff (seconds)
PINN_MAX_BACKOFF_SEC = 60.0

# Curved path preservation constants
MIN_TURN_CURVATURE_THRESHOLD = 0.3  # minimum curvature for a valid turning path
STRAIGHT_PATH_PENALTY_MULTIPLIER = 20.0  # penalty scale for straight paths during turns
TURN_ANGLE_CURVATURE_SCALE = 200.0  # degrees-to-curvature scaling factor
SEED_PRESERVATION_RATIO = 1.0  # fraction of generations to preserve curved seed
CURVE_PRESERVATION_FACTOR = 0.5  # scale factor for second waypoint during start correction
# Number of initial population slots (after index 0) to replace with the
# curved seed when a turn is required, preventing straight-path dominance.
MAX_CURVED_SEED_REPLACEMENTS = 3
# Reference turn angle (degrees) used to scale the straight-path penalty;
# a 30° turn yields a scale factor of 1, larger turns scale proportionally.
ANGLE_SCALE_DIVISOR_DEG = 30.0

# Bezier curve generation constants
BEZIER_CONTROL_POINT_RATIO = 1.0 / 3.0  # control-point offset as a fraction of total distance
BEZIER_WAYPOINT_SPACING = 0.3  # metres between sampled waypoints on the Bezier curve


def dominates(a, b):
    """Return True if a dominates b"""
    return all(x <= y for x, y in zip(a, b)) and any(x < y for x, y in zip(a, b))


class CompletePathOptimizer(Node):
    def __init__(self):
        super().__init__('pinn_integrated_path_optimizer')
        
        # Load parameters
        self.load_parameters()
        # Dynamic obstacle tracking
        self.obstacle_tracker = DynamicObstacleTracker(self)
        self.enable_dynamic_prediction = True
        self.dynamic_weight = 0.3  # Weight for dynamic obstacle cost        
        self.get_logger().info("🚀 PINN-Integrated Path Optimizer - ALL FEATURES RESTORED")
        self.get_logger().info(f"Using PINN: {self.use_pinn}")
        self.laser_frame = None
        # State variables
        self.robot_pose = None
        self.goal_pose = None
        self.goal_position = None
        self.laser_scan = None
        self.scan_angles = None
        self.optimization_active = False
        self.optimization_count = 0
        self.last_optimization_time = 0.0
        # Path update throttling - only regenerate if robot moved significantly.
        # Raising from 0.3 m → 0.5 m reduces churn: at planning_rate=2.5 Hz
        # (0.4 s intervals) and v≈0.5 m/s the robot moves ~0.2 m per cycle so
        # a 0.5 m threshold means one regeneration roughly every 2 timer cycles.
        self.last_path_position = None  # Track last position when path was generated
        self._last_deviation_warn_time = 0.0     # Already there? Ensure it exists
        self.last_pinn_calls = 0                 # PINN stats
        self.last_pinn_success = 0
        self.last_optimization_time = 0.0        # For optimization timer
        self.optimization_active = False         # Prevents concurrent runs
        self.last_published_path = None          # Fallback path
        self.num_candidates = 5   
        self.path_update_distance_threshold = 0.5  # Only regenerate if moved > 50 cm
        # Cooldown for path-start deviation warnings (avoids log spam while stationary)
        self._last_deviation_warn_time: float = 0.0
        # ADD THESE ATTRIBUTES (PINN tracking):
        self.pinn_call_count = 0
        self.pinn_timeout_count = 0
        self.pinn_success_count = 0
        self.pinn_response_times = deque(maxlen=100)  # For tracking performance
        # PINN exponential-backoff / auto-disable
        self.pinn_consecutive_failures = 0
        self.pinn_max_consecutive_failures = 5   # disable after this many consecutive timeouts
        self.pinn_disabled_logged = False         # log the disable event only once
        self.pinn_backoff_until: float = 0.0     # wall-clock time when backoff expires
        # Stuck detection - IMPROVED
        self.stuck_positions = deque(maxlen=25)
        self.stuck_threshold = 0.1
        self.stuck_time_threshold = 10.0
        self.last_significant_movement = time.time()
        self.stuck_count = 0
        self.obstacle_nearby_when_stuck = False
        self.consecutive_stuck_checks = 0
        self.last_stuck_time = 0.0
        self.goal_received_time = None  # Track when goal was received
        
        # Grids
        self.obstacle_grid_origin = None
        self.obstacle_grid_size = 150
        self.obstacle_grid_resolution = 0.3
        self.obstacle_grid = None
        self.free_space_grid = None
        self.free_space_waypoints = []
        self.occupied_threshold = 0.5   # 50% occupancy = obstacle
        
        # Escape paths - IMPROVED
        self.escape_paths_generated = 0
        self.last_escape_time = 0.0
        self.last_escape_pose = None
        self.last_escape_direction = None
        self.escape_attempts = defaultdict(int)
        self.escape_success_history = deque(maxlen=5)
        self.escape_path_history = deque(maxlen=5)  # FIXED: Added missing attribute
        #maximum linear velocity
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        

        # Progress monitoring
        self.progress_toward_goal = deque(maxlen=20)
        self.last_goal_distance = float('inf')
        self.position_history = deque(maxlen=50)
        
        # PINN service client
        self.pinn_client = None
        self.pinn_service_available = False
        self.pinn_ready_event = threading.Event()
        if self.use_pinn:
             # Initialize PINN client in a separate thread to avoid blocking
            self.pinn_init_thread = threading.Thread(target=self.setup_pinn_client)
            self.pinn_init_thread.daemon = True
            self.pinn_init_thread.start()
        
        # Thread pool
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        self.pinn_futures = {}
        self.pinn_future_lock = threading.Lock()
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        #self.laser_sub = self.create_subscription(LaserScan, self.laser_topic, self.laser_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/piec/path', 10)
        self.obstacle_pub = self.create_publisher(PoseArray, '/debug/obstacles', 10)
        self.stuck_pub = self.create_publisher(PoseArray, '/debug/stuck_positions', 10)
        self.free_space_pub = self.create_publisher(PoseArray, '/debug/free_space', 10)
        
        # Initialize components with YOUR modules
        self.objective_evaluator = ObjectiveEvaluator(self)
        self.uncertainty_planner = UncertaintyAwarePlanner(self)
        
        # Initialize grids
        self.initialize_obstacle_grid()
        self.last_optimized_path = None   # Store last published path for hysteresis
        # Timers
        self.optimization_timer = self.create_timer(self.planning_rate, self.optimization_timer_callback)
        self.obstacle_timer = self.create_timer(0.1, self.update_obstacle_map)
        self.stuck_check_timer = self.create_timer(1.0, self.check_stuck_status)
        self.free_space_timer = self.create_timer(2.0, self.update_free_space_map)
        self.progress_timer = self.create_timer(2.0, self.check_progress)
         # Test PINN connection after initialization
        if self.use_pinn:
            # Test PINN connection after a short delay
            test_timer = threading.Timer(2.0, self.test_pinn_connection)
            test_timer.daemon = True
            test_timer.start()
        
        self.get_logger().info("✅ Path Optimizer Ready with ALL FEATURES")
        self.last_pinn_calls = 0
        self.last_pinn_success = 0
        # Add this after initializing other counters
        self._laser_cb_counter = 0
        self._costmap_pub_counter = 0
        self._costmap_debug_counter = 0
        # In __init__, add a publisher
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create a BEST_EFFORT QoS profile for laser scan
        scan_qos = QoSProfile(
     
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,

        )
        self.laser_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.laser_callback,
            scan_qos
        )
    def load_parameters(self):
        """Load parameters from YAML"""
        default_params = {
            'goal_topic': '/goal_pose',
            'odom_topic': '/ukf/odom',
            'laser_topic': '/scan',
            'population_size': 6,
            'generations': 2,
            'crossover_rate': 0.8,
            'mutation_rate': 0.5,
            'optimization_timeout': 3.5,
            'planning_rate': 1.0,
            'waypoint_count': 8,
            'max_curvature': 2.5,
            'path_smoothing': True,
            'use_pinn_predictions': True,
            'pinn_service_name': '/evaluate_trajectory',
            'pinn_timeout': 2.0,  # Initial connection timeout
            'pinn_call_timeout': 2.0,
            'objective_weights': [0.12, 0.08, 0.35, 0.25, 0.12, 0.15, 0.1],
            "obstacle_penalty_weight": 1000.0,        # Increased from 6.0,12
            "min_obstacle_distance": 0.4,           # Reduced from 0.6
            'escape_clearance': 0.7,
            'max_escape_attempts': 3,
            'escape_cooldown': 6.0,
            'significant_turning_threshold_deg': 30.0,  # Threshold for curved path generation
            'obstacle_grid_size': 150,
            'obstacle_grid_resolution': 0.1,
            'debug_mode': True,
            'goal_completion_distance': 0.25,
            'robot_radius': 0.45,
            'max_linear_vel': 1.2,
            'max_angular_vel': 0.8,
            'escape_path_length': 1.5,
            'escape_turn_angle': 45.0,
            'free_space_weight': 0.3,
            'preferred_clearance': 1.0,
            'free_space_sample_count': 15,
            'free_space_update_rate': 2.0,
            'exploration_factor': 0.4,
            'stuck_movement_threshold': 0.2,
            'min_escape_distance': 1.0,
            'escape_backup_distance': 0.6,
            'escape_lateral_distance': 0.8,
            'max_pinn_calls_per_generation': 10,
            
        }
        
        # Declare all parameters
        for key, value in default_params.items():
            self.declare_parameter(key, value)
        
        # Store parameters
        self.goal_topic = self.get_parameter('goal_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.laser_topic = self.get_parameter('laser_topic').value
        self.population_size = self.get_parameter('population_size').value
        self.generations = self.get_parameter('generations').value
        self.crossover_rate = self.get_parameter('crossover_rate').value
        self.mutation_rate = self.get_parameter('mutation_rate').value
        self.optimization_timeout = self.get_parameter('optimization_timeout').value
        self.planning_rate = self.get_parameter('planning_rate').value
        self.waypoint_count = self.get_parameter('waypoint_count').value
        self.max_curvature = self.get_parameter('max_curvature').value
        self.path_smoothing = self.get_parameter('path_smoothing').value
        self.use_pinn = self.get_parameter('use_pinn_predictions').value
        self.pinn_service_name = self.get_parameter('pinn_service_name').value
        self.pinn_timeout = self.get_parameter('pinn_timeout').value
        self.pinn_call_timeout = self.get_parameter('pinn_call_timeout').value
        self.objective_weights = self.get_parameter('objective_weights').value
        self.obstacle_penalty_weight = self.get_parameter('obstacle_penalty_weight').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.obstacle_grid_size = self.get_parameter('obstacle_grid_size').value
        self.obstacle_grid_resolution = self.get_parameter('obstacle_grid_resolution').value
        self.escape_clearance = self.get_parameter('escape_clearance').value
        self.max_escape_attempts = self.get_parameter('max_escape_attempts').value
        self.escape_cooldown = self.get_parameter('escape_cooldown').value
        self.significant_turning_threshold_deg = self.get_parameter('significant_turning_threshold_deg').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.goal_completion_distance = self.get_parameter('goal_completion_distance').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.escape_path_length = self.get_parameter('escape_path_length').value
        self.escape_turn_angle = math.radians(self.get_parameter('escape_turn_angle').value)
        self.free_space_weight = self.get_parameter('free_space_weight').value
        self.preferred_clearance = self.get_parameter('preferred_clearance').value
        self.free_space_sample_count = self.get_parameter('free_space_sample_count').value
        self.free_space_update_rate = self.get_parameter('free_space_update_rate').value
        self.exploration_factor = self.get_parameter('exploration_factor').value
        self.stuck_movement_threshold = self.get_parameter('stuck_movement_threshold').value
        self.min_escape_distance = self.get_parameter('min_escape_distance').value
        self.escape_backup_distance = self.get_parameter('escape_backup_distance').value
        self.escape_lateral_distance = self.get_parameter('escape_lateral_distance').value
        self.max_pinn_calls_per_generation = self.get_parameter('max_pinn_calls_per_generation').value
        self.declare_parameter('occupied_threshold', 0.5)
        self.occupied_threshold = self.get_parameter('occupied_threshold').value    
    def initialize_obstacle_grid(self):
        """Initialize obstacle and free space grids"""
        size = self.obstacle_grid_size
        self.obstacle_grid = np.zeros((size, size), dtype=np.float32)
        self.free_space_grid = np.ones((size, size), dtype=np.float32)
        self.get_logger().info(f"✅ Grids initialized: {size}x{size} (resolution: {self.obstacle_grid_resolution}m)")
    
    def setup_pinn_client(self):
        """Setup PINN service client - NON-BLOCKING VERSION."""
        try:
            if not PINN_SERVICE_AVAILABLE:
                self.get_logger().warn("PINN service type not available, disabling PINN")
                self.use_pinn = False
                self.pinn_ready_event.set()
                return

            self.pinn_client = self.create_client(
                PINN_SERVICE_TYPE,
                self.pinn_service_name
            )

            self.get_logger().info(f"PINN client created (non-blocking)")

            self.pinn_service_available = False
            self.pinn_ready_event.set()

            check_thread = threading.Thread(target=self._async_pinn_check)
            check_thread.daemon = True
            check_thread.start()

        except Exception as e:
            self.get_logger().error(f"Failed to create PINN client: {e}")
            self.use_pinn = False
            self.pinn_service_available = False
            self.pinn_ready_event.set()

    def _async_pinn_check(self):
        """Async check for PINN availability (runs in background)."""
        for attempt in range(20):   # 10 seconds total
            time.sleep(0.5)
            try:
                if self.pinn_client and self.pinn_client.wait_for_service(timeout_sec=0.2):
                    self.pinn_service_available = True
                    self.get_logger().info("✅ PINN service became available")
                    return
            except Exception:
                pass

        self.get_logger().warn("⚠️ PINN service not available - running without PINN optimization")
        self.pinn_service_available = False
    
    def check_pinn_connection(self):
        """Periodically check PINN connection"""
        if not self.use_pinn or self.pinn_client is None:
            return
        
        try:
            if self.pinn_client.service_is_ready():
                if not self.pinn_service_available:
                    self.pinn_service_available = True
                    self.get_logger().info("✅ PINN service reconnected")
            else:
                if self.pinn_service_available:
                    self.pinn_service_available = False
                    self.get_logger().warn("⚠️ PINN service disconnected")
        except Exception as e:
            self.get_logger().debug(f"PINN connection check failed: {e}")
    def test_pinn_connection(self):
        """Test PINN connection - skipped at startup; first optimization will validate"""
        if self.use_pinn and self.pinn_service_available:
            self.get_logger().info(
                "🔬 PINN service connected. Will validate on first optimization."
            )
            return True
        return False
# Update the call_pinn_service_optimized method to properly handle timeouts:
    def call_pinn_service_optimized(self, path_array):
        """Call PINN service with proper timeout handling and exponential backoff."""
        if not self.use_pinn or not self.pinn_service_available or self.pinn_client is None:
            if self.debug_mode:
                self.get_logger().debug("PINN disabled or client not available")
            return None

        # Honour exponential backoff: skip PINN calls until backoff expires
        if time.time() < self.pinn_backoff_until:
            return None

        # Wait for PINN to be ready
        if not self.pinn_ready_event.wait(timeout=0.1):
            if self.debug_mode:
                self.get_logger().debug("PINN not ready yet")
            return None

        # Initialize attributes if they don't exist
        if not hasattr(self, 'pinn_call_count'):
            self.pinn_call_count = 0
            self.pinn_timeout_count = 0
            self.pinn_success_count = 0
            self.pinn_response_times = deque(maxlen=100)

        self.pinn_call_count += 1
        call_id = self.pinn_call_count

        try:
            # Prepare request
            request = PINN_SERVICE_TYPE.Request()

            # Convert path to arrays
            xs = []
            ys = []
            yaws = []
            velocities = []

            # Pre-compute per-segment curvature to set velocity
            n = len(path_array)
            seg_curvatures = [0.0] * n
            for i in range(1, n - 1):
                x0, y0 = path_array[i - 1]
                x1, y1 = path_array[i]
                x2, y2 = path_array[i + 1]
                dx1, dy1 = x1 - x0, y1 - y0
                dx2, dy2 = x2 - x1, y2 - y1
                n1 = math.hypot(dx1, dy1)
                n2 = math.hypot(dx2, dy2)
                if n1 > 1e-6 and n2 > 1e-6:
                    cos_a = max(-1.0, min(
                        1.0, (dx1 * dx2 + dy1 * dy2) / (n1 * n2)))
                    seg_curvatures[i] = abs(math.acos(cos_a))

            # Use ALL points for better accuracy
            for i in range(n):
                x, y = path_array[i]
                xs.append(float(x))
                ys.append(float(y))

                # Calculate yaw: last waypoint uses previous segment direction
                if i < n - 1:
                    next_x, next_y = path_array[i + 1]
                    yaw = math.atan2(next_y - y, next_x - x)
                elif i > 0:
                    prev_x, prev_y = path_array[i - 1]
                    yaw = math.atan2(y - prev_y, x - prev_x)
                else:
                    yaw = 0.0
                yaws.append(float(yaw))

                # Velocity: slow down proportionally to local curvature
                curv = seg_curvatures[i]
                vel = max(0.2, 0.5 * (1.0 - min(curv / math.pi, 1.0)))
                velocities.append(float(vel))

            request.xs = xs
            request.ys = ys
            request.yaws = yaws
            request.velocities = velocities

            # DEBUG: Log request details
            if self.debug_mode:
                self.get_logger().debug(f"PINN call #{call_id}: {len(xs)} points")

            # Warn when only 2 points are sent: obstacle features will be zero
            # and stability will appear artificially high (clear-path prediction).
            if len(path_array) <= 2 and self.debug_mode:
                self.get_logger().warn(
                    f"⚠️ PINN call #{call_id} with only {len(path_array)} points - "
                    f"obstacle features will be zero"
                )

            start_time = time.time()
            future = self.pinn_client.call_async(request)
            timeout_sec = self.pinn_call_timeout

            # Wait for response using future.result(timeout) – no spin_once
            try:
                response = future.result(timeout=timeout_sec)
            except TimeoutError:
                if self.debug_mode:
                    self.get_logger().debug(f"PINN call #{call_id} timeout after {timeout_sec}s")
                self.pinn_timeout_count += 1
                self.pinn_consecutive_failures += 1
                future.cancel()
                # Exponential backoff: wait 2^k seconds before trying PINN again
                backoff_sec = min(PINN_MAX_BACKOFF_SEC, 2.0 ** self.pinn_consecutive_failures)
                self.pinn_backoff_until = time.time() + backoff_sec
                if self.pinn_consecutive_failures >= self.pinn_max_consecutive_failures:
                    if not self.pinn_disabled_logged:
                        self.get_logger().warn(
                            f"⚠️ PINN service timed out {self.pinn_consecutive_failures} times in a row – "
                            "disabling PINN for this session and continuing without it."
                        )
                        self.pinn_disabled_logged = True
                    self.pinn_service_available = False
                return {
                    'energy': 0.0,
                    'stability': 0.0,
                    'response_time': timeout_sec,
                    'success': False,
                    'timeout': True
                }
            except Exception as e:
                if self.debug_mode:
                    self.get_logger().debug(f"PINN call #{call_id} failed: {e}")
                return {
                    'energy': 0.0,
                    'stability': 0.0,
                    'response_time': 0.0,
                    'success': False,
                    'timeout': False
                }

            response_time = time.time() - start_time

            if response is not None:
                self.pinn_success_count += 1
                self.pinn_response_times.append(response_time)
                # Successful response – reset backoff state
                self.pinn_consecutive_failures = 0
                self.pinn_backoff_until = 0.0
                if self.pinn_disabled_logged:
                    self.get_logger().info("✅ PINN service recovered – re-enabling.")
                    self.pinn_disabled_logged = False
                    self.pinn_service_available = True

                energy = float(response.energy)
                stability = float(response.stability)

                avg_time = np.mean(self.pinn_response_times) if self.pinn_response_times else 0.0
                success_rate = (self.pinn_success_count / self.pinn_call_count * 100) if self.pinn_call_count > 0 else 0

                self.get_logger().info(
                    f"PINN Service: Request #{call_id}, "
                    f"Energy={energy:.2f}J, "
                    f"Stability={stability:.3f}, "
                    f"Response time: {response_time:.3f}s, "
                    f"Avg: {avg_time:.3f}s, "
                    f"Success: {success_rate:.1f}%"
                )

                return {
                    'energy': energy,
                    'stability': stability,
                    'response_time': response_time,
                    'success': True,
                    'timeout': False
                }
            else:
                if self.debug_mode:
                    self.get_logger().debug(f"PINN call #{call_id} returned None")
                return {
                    'energy': 0.0,
                    'stability': 0.0,
                    'response_time': response_time,
                    'success': False,
                    'timeout': False
                }

        except Exception as e:
            if self.debug_mode:
                self.get_logger().debug(f"PINN call #{call_id} failed: {e}")
            return {
                'energy': 0.0,
                'stability': 0.0,
                'response_time': 0.0,
                'success': False,
                'timeout': False
            }
    def recenter_obstacle_grid(self, robot_x, robot_y):
        """Recenter grid when robot moves beyond threshold (bottom‑left origin)."""
        if self.obstacle_grid is None or self.obstacle_grid_origin is None:
            return

        half = self._grid_half_size_m()
        # Current grid center (in world coordinates)
        current_center_x = self.obstacle_grid_origin[0] + half
        current_center_y = self.obstacle_grid_origin[1] + half

        # If robot still within 2m of grid center, do nothing
        if math.hypot(robot_x - current_center_x, robot_y - current_center_y) < 2.0:
            return

        # New grid center at robot position
        new_center_x = robot_x
        new_center_y = robot_y
        # New bottom‑left origin
        new_origin_x = new_center_x - half
        new_origin_y = new_center_y - half

        # Compute shift in grid cells
        dx_cells = int((new_origin_x - self.obstacle_grid_origin[0]) / self.obstacle_grid_resolution)
        dy_cells = int((new_origin_y - self.obstacle_grid_origin[1]) / self.obstacle_grid_resolution)

        # Create new grid and copy overlapping region
        new_grid = np.zeros((self.obstacle_grid_size, self.obstacle_grid_size), dtype=np.float32)
        for i in range(self.obstacle_grid_size):
            for j in range(self.obstacle_grid_size):
                old_i = i - dx_cells
                old_j = j - dy_cells
                if 0 <= old_i < self.obstacle_grid_size and 0 <= old_j < self.obstacle_grid_size:
                    new_grid[i, j] = self.obstacle_grid[old_i, old_j]

        self.obstacle_grid = new_grid
        self.obstacle_grid_origin = (new_origin_x, new_origin_y)

        if self.debug_mode:
            self.get_logger().info(f"Re-centered obstacle grid to ({robot_x:.2f}, {robot_y:.2f})")
    def odom_callback(self, msg: Odometry):
        """Update robot pose"""
        self.robot_pose = msg

        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        # IMPORTANT: origin must be bottom‑left, not robot position
        self._ensure_grid_origin_initialized(robot_x, robot_y)

        current_pos = (robot_x, robot_y)
        current_time = time.time()
        self.stuck_positions.append((current_pos, current_time))
        self.position_history.append(current_pos)

        if self.goal_pose:
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            current_distance = math.hypot(goal_x - current_pos[0], goal_y - current_pos[1])
            self.progress_toward_goal.append((current_time, current_distance))
            self.last_goal_distance = current_distance

        self.uncertainty_planner.update_uncertainty_map(msg)
        self.last_position = current_pos

        # Re‑center grid when robot moves far
        self.recenter_obstacle_grid(robot_x, robot_y)
    def get_corridor_width_pythagorean(self, x, y, heading_angle, max_search=2.0):
        """
        Returns the distance between the nearest obstacles on the left and right
        of the robot's heading direction, using the Pythagorean theorem.
        Also returns the left and right obstacle positions.
        """
        left_angle = heading_angle + math.pi / 2
        right_angle = heading_angle - math.pi / 2

        step = 0.05
        left_dist = max_search
        right_dist = max_search
        left_obs = (x + max_search * math.cos(left_angle),
                    y + max_search * math.sin(left_angle))
        right_obs = (x + max_search * math.cos(right_angle),
                     y + max_search * math.sin(right_angle))

        # Raycast left
        for d in np.arange(step, max_search, step):
            test_x = x + d * math.cos(left_angle)
            test_y = y + d * math.sin(left_angle)
            if self.get_clearance_at_point(test_x, test_y) < 0.3:
                left_dist = d - step
                left_obs = (test_x, test_y)
                break

        # Raycast right
        for d in np.arange(step, max_search, step):
            test_x = x + d * math.cos(right_angle)
            test_y = y + d * math.sin(right_angle)
            if self.get_clearance_at_point(test_x, test_y) < 0.3:
                right_dist = d - step
                right_obs = (test_x, test_y)
                break

        # Pythagorean distance between left and right obstacles
        dx = left_obs[0] - right_obs[0]
        dy = left_obs[1] - right_obs[1]
        direct_distance = math.hypot(dx, dy)

        return {
            'left_dist': left_dist,
            'right_dist': right_dist,
            'corridor_width': left_dist + right_dist,   # approximate
            'direct_gap': direct_distance,              # true gap between obstacles
            'left_obstacle': left_obs,
            'right_obstacle': right_obs
        }
    def is_straight_path_clear(self, start_x, start_y, goal_x, goal_y, check_distance=None):
        """
        Strict straight‑path check that considers the robot's full footprint.
        - Forward cone check (±35°) using laser scan.
        - Samples along the line and checks clearance at center, left, and right edges.
        - Overall minimum clearance check for the whole path.
        Returns True only if the entire corridor is clear.
        """
        distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance_to_goal < 0.5:
            self.get_logger().debug("📏 is_straight_path_clear: distance <0.5m → True")
            return True

        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        max_check = min(distance_to_goal, check_distance if check_distance else 5.0)

        # ---- 1) Direct laser cone check (emergency stop cone: ±35°) ----
        if self.laser_scan is not None and self.robot_pose is not None:
            robot_x = self.robot_pose.pose.pose.position.x
            robot_y = self.robot_pose.pose.pose.position.y
            robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
            half_cone = math.radians(35)  # matches emergency stop node
            min_forward_dist = float('inf')
            for i, r in enumerate(self.laser_scan.ranges):
                if r < 0.1 or r > self.laser_scan.range_max:
                    continue
                angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
                rel_angle = angle - robot_yaw
                rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
                if abs(rel_angle) < half_cone:
                    if r < min_forward_dist:
                        min_forward_dist = r
            if min_forward_dist < 1.0:   # tight forward clearance
                self.get_logger().info(
                    f"🚫 Straight path NOT clear: obstacle at {min_forward_dist:.2f}m in forward cone"
                )
                return False
            self.get_logger().debug(f"✅ Forward cone clear: nearest obstacle at {min_forward_dist:.2f}m")

        # ---- 2) Fine sampling along the line, checking the robot's footprint ----
        step = 0.2
        num_steps = int(max_check / step) + 1
        robot_radius = self.robot_radius
        safety_margin = 0.35   # additional margin to avoid emergency stop

        for i in range(num_steps):
            dist = i * step
            if dist > max_check:
                break
            center_x = start_x + dist * math.cos(goal_dir)
            center_y = start_y + dist * math.sin(goal_dir)

            # Perpendicular direction (left/right of path)
            perp_x = -math.sin(goal_dir)
            perp_y = math.cos(goal_dir)
            left_x = center_x + robot_radius * perp_x
            left_y = center_y + robot_radius * perp_y
            right_x = center_x - robot_radius * perp_x
            right_y = center_y - robot_radius * perp_y

            for (cx, cy, label) in [(center_x, center_y, "center"),
                                     (left_x, left_y, "left"),
                                     (right_x, right_y, "right")]:
                clearance = self.get_clearance_at_point(cx, cy)
                required_clearance = robot_radius + safety_margin
                if clearance < required_clearance:
                    self.get_logger().debug(
                        f"  Point at {dist:.1f}m ({label}): clearance={clearance:.2f}m < {required_clearance:.2f}m → BLOCKED"
                    )
                    self.get_logger().info(
                        f"🚫 Straight path NOT clear: obstacle near {label} side at distance {dist:.1f}m"
                    )
                    return False
                else:
                    self.get_logger().debug(
                        f"  Point at {dist:.1f}m ({label}): clearance={clearance:.2f}m → CLEAR"
                    )

        # ---- 3) Overall minimum clearance check ----
        # Re‑sample the whole path with the improved clearance function
        overall_min = float('inf')
        for i in range(num_steps):
            dist = i * step
            if dist > max_check:
                break
            cx = start_x + dist * math.cos(goal_dir)
            cy = start_y + dist * math.sin(goal_dir)
            cl = self.get_clearance_at_point(cx, cy)
            if cl < overall_min:
                overall_min = cl
        required_overall = robot_radius + safety_margin
        if overall_min < required_overall:
            self.get_logger().info(
                f"🚫 Straight path NOT clear: overall min clearance {overall_min:.2f}m < {required_overall:.2f}m"
            )
            return False

        self.get_logger().info("✅ Straight path IS clear (all sampled points OK)")
        return True
    def _find_closest_waypoint_on_path(self, robot_x, robot_y, path):
        """Return (index, distance) of the waypoint on `path` closest to the robot."""
        if not path:
            return None, float('inf')
        best_idx = 0
        best_dist = float('inf')
        for i, (px, py) in enumerate(path):
            d = math.hypot(robot_x - px, robot_y - py)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx, best_dist

    def _calculate_path_segment_length(self, path):
        """Total length of a path (list of (x,y) tuples)."""
        if len(path) < 2:
            return 0.0
        length = 0.0
        for i in range(len(path)-1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += math.hypot(dx, dy)
        return length
    def _should_skip_replanning(self, robot_x, robot_y, goal_x, goal_y):
        """
        Returns True if the current path's remaining part is still safe and we should skip replanning.
        Now requires clearance >= 0.55m and curvature < 0.4 to be "very safe".
        """
        if not hasattr(self, 'last_optimized_path') or self.last_optimized_path is None:
            return False
        if len(self.last_optimized_path) < 2:
            return False

        closest_idx, _ = self._find_closest_waypoint_on_path(robot_x, robot_y, self.last_optimized_path)
        if closest_idx is None or closest_idx >= len(self.last_optimized_path) - 2:
            return False

        remaining = self.last_optimized_path[closest_idx:]
        remaining_length = self._calculate_path_segment_length(remaining)
        straight_dist = math.hypot(goal_x - robot_x, goal_y - robot_y)

        rem_clear = self._get_minimum_clearance_on_path(remaining)
        rem_curv = self.calculate_curvature(np.array([[p[0], p[1]] for p in remaining])) if len(remaining) >= 3 else 0.0

        self.get_logger().info(
            f"🔍 Remaining path: len={remaining_length:.2f}m, clearance={rem_clear:.2f}m, "
            f"curvature={rem_curv:.3f}, straight_dist={straight_dist:.2f}m"
        )

        # VERY SAFE: clearance >= 0.55m and curvature < 0.4 → always keep
        if rem_clear >= 0.55 and rem_curv < 0.4:
            self.get_logger().info(
                f"✅ Skipping replan – remaining path is very safe (clearance={rem_clear:.2f}m, curvature={rem_curv:.3f})"
            )
            return True

        # SAFE ENOUGH: clearance >= 0.45m and curvature < 0.5 → keep unless robot moved > 2.5m
        if rem_clear >= 0.45 and rem_curv < 0.5:
            if hasattr(self, 'last_path_position') and self.last_path_position is not None:
                moved = math.hypot(robot_x - self.last_path_position[0],
                                   robot_y - self.last_path_position[1])
                if moved < 2.5:
                    self.get_logger().info(
                        f"✅ Skipping replan – remaining path is safe (clearance={rem_clear:.2f}m, moved={moved:.2f}m)"
                    )
                    return True
            else:
                return True

        self.get_logger().info("📊 Remaining path not safe enough – allowing replan")
        return False
    def generate_initial_path(self, start_x, start_y, goal_x, goal_y):
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance < 0.5:
            self.get_logger().info("📏 generate_initial_path: short distance → two-point path")
            return [(start_x, start_y), (goal_x, goal_y)]

        straight_clear = self.is_straight_path_clear(start_x, start_y, goal_x, goal_y)
        self.get_logger().info(f"🔍 generate_initial_path: straight_clear = {straight_clear}")

        if straight_clear:
            path = self._interpolate_path(start_x, start_y, goal_x, goal_y,
                                          num_points=self.waypoint_count)
            self.get_logger().info("✅ Using straight path (CONFIRMED CLEAR)")
            return path

        # Straight blocked → force curved
        self.get_logger().warn("🚧 Straight path BLOCKED – forcing curved path")

        obstacle_path = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
        if obstacle_path and len(obstacle_path) >= 3:
            self.get_logger().info("🛣️ Using obstacle-avoiding path")
            return obstacle_path

        curved_path = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
        if curved_path and len(curved_path) >= 3:
            self.get_logger().info("🌀 Using curved turning path")
            return curved_path

        self.get_logger().warn("⚠️ All specific paths failed – using simple interpolated path")
        return self._interpolate_path(start_x, start_y, goal_x, goal_y,
                                      num_points=self.waypoint_count)
    def requires_significant_turning(self, start_x, start_y, goal_x, goal_y, threshold_degrees=30):
        """Check if goal requires significant turning from current robot orientation"""
        if self.robot_pose is None:
            return False
        
        robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
        goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)
        angle_diff = abs(math.atan2(math.sin(goal_angle - robot_yaw), math.cos(goal_angle - robot_yaw)))
        
        return angle_diff > math.radians(threshold_degrees)
    
    def should_use_simple_straight_path(self, start_x, start_y, goal_x, goal_y):
        """Check if simple straight path is sufficient - DISABLED to force PINN optimization"""
        # Always return False to force PINN optimization for research validation
        return False
    def should_update_path(self, start_x, start_y):
        """Check if path should be updated based on robot movement."""
        if self.last_path_position is not None:
            dist_moved = math.hypot(
                start_x - self.last_path_position[0],
                start_y - self.last_path_position[1]
            )
            # Increase threshold to 0.8m to reduce churn
            if dist_moved < 1.0:   # was 0.5
                return False
        self.last_path_position = (start_x, start_y)
        return True

    def check_progress(self):
        """Check if robot is making progress toward goal - ACCOUNT FOR SPEED"""
        if len(self.progress_toward_goal) < 3:
            return
        
        # Get robot's current speed if available
        current_speed = 0.0
        if self.robot_pose and hasattr(self.robot_pose.twist.twist.linear, 'x'):
            lin_x = self.robot_pose.twist.twist.linear.x
            lin_y = self.robot_pose.twist.twist.linear.y
            current_speed = math.hypot(lin_x, lin_y)
        
        # Only check progress if robot is supposed to be moving
        if current_speed < 0.1:  # Robot is essentially stopped
            return
        
        oldest_time, oldest_dist = self.progress_toward_goal[0]
        latest_time, latest_dist = self.progress_toward_goal[-1]
        
        time_diff = latest_time - oldest_time
        
        # Adjust check time based on distance to goal
        goal_x = self.goal_pose.pose.position.x if self.goal_pose else 0
        goal_y = self.goal_pose.pose.position.y if self.goal_pose else 0
        current_x = self.robot_pose.pose.pose.position.x if self.robot_pose else 0
        current_y = self.robot_pose.pose.pose.position.y if self.robot_pose else 0
        distance_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)
        
        # For longer distances, allow more time for progress
        required_time = 10.0 if distance_to_goal > 3.0 else 8.0
        
        if time_diff > required_time:
            distance_improvement = oldest_dist - latest_dist
            
            # Expected movement based on speed
            expected_movement = current_speed * time_diff
            
            # If we moved less than 30% of expected movement
            if distance_improvement < expected_movement * 0.3 and distance_improvement < 0.5:
                self.consecutive_stuck_checks += 1
                if self.debug_mode:
                    self.get_logger().debug(
                        f"Poor progress: {distance_improvement:.2f}m in {time_diff:.1f}s "
                        f"(expected: {expected_movement:.2f}m at {current_speed:.2f}m/s)"
                    )
            else:
                self.consecutive_stuck_checks = max(0, self.consecutive_stuck_checks - 1)
    
    def check_stuck_status(self):
        """Enhanced stuck detection - REQUIRE MINIMUM TIME"""
        if not self.robot_pose or not self.goal_pose:
            return

        # CRITICAL: Do NOT check stuck if no goal has been received yet
        if not hasattr(self, 'goal_received_time') or self.goal_received_time is None:
            return

        current_time = time.time()
        current_pos = (self.robot_pose.pose.pose.position.x,
                       self.robot_pose.pose.pose.position.y)

        # Cooldown after goal received
        time_since_goal = current_time - self.goal_received_time
        if time_since_goal < 8.0:
            return

        # Cooldown after last stuck detection
        if current_time - self.last_stuck_time < 20.0:
            return

        # Require enough data
        if len(self.stuck_positions) < 50:
            return

        # Calculate metrics over a long window
        time_window = 15.0
        relevant_positions = [(pos, t) for pos, t in self.stuck_positions
                              if current_time - t <= time_window]

        if len(relevant_positions) < 40:
            return

        positions = [pos for pos, _ in relevant_positions]

        total_movement = sum(
            math.hypot(positions[i+1][0] - positions[i][0],
                       positions[i+1][1] - positions[i][1])
            for i in range(len(positions) - 1)
        )

        center = np.mean(positions, axis=0)
        radius = np.mean([math.hypot(p[0]-center[0], p[1]-center[1]) for p in positions])

        goal_dist = math.hypot(
            self.goal_pose.pose.position.x - current_pos[0],
            self.goal_pose.pose.position.y - current_pos[1]
        )

        is_stuck = False
        reasons = []

        if goal_dist < 0.35:
            return

        time_since_goal = current_time - self.goal_received_time
        if time_since_goal < 12.0:
            return

        # Verify forward is actually blocked
        if self.robot_pose and self.goal_pose:
            rx = self.robot_pose.pose.pose.position.x
            ry = self.robot_pose.pose.pose.position.y
            gx = self.goal_pose.pose.position.x
            gy = self.goal_pose.pose.position.y

            forward_blocked = self.is_forward_path_blocked(rx, ry, gx, gy, min_clearance=0.5)

            if not forward_blocked:
                if self.debug_mode and current_time - self.last_stuck_time > 10.0:
                    self.get_logger().info("✅ Forward path is CLEAR - no escape needed")
                self.stuck_positions.clear()
                return

        if total_movement < 0.15 and radius < 0.30:
            is_stuck = True
            reasons.append(f"Oscillating: movement={total_movement:.2f}m, radius={radius:.2f}m")

        if is_stuck:
            self.stuck_count += 1
            self.last_stuck_time = current_time
            self.consecutive_stuck_checks = 0

            if self.debug_mode:
                self.get_logger().warn(
                    f"🤖 STUCK/OSCILLATING DETECTED! (Attempt #{self.stuck_count})"
                )
                self.get_logger().warn(f"  Reasons: {', '.join(reasons)}")
                self.get_logger().warn(f"  Goal distance: {goal_dist:.2f}m")
                self.get_logger().warn(f"  Time since goal: {time_since_goal:.1f}s")

            self.generate_adaptive_escape_path()   # <-- call the improved method
            self.stuck_positions.clear()
        else:
            if total_movement > 0.3:
                self.consecutive_stuck_checks = 0
            elif total_movement > 0.1:
                self.consecutive_stuck_checks = max(0, self.consecutive_stuck_checks - 1)
    def check_severe_blocking_obstacles(self):
        """Check for severe blocking obstacles (not just any obstacles)"""
        if self.robot_pose is None or self.goal_pose is None or self.obstacle_grid is None:
            return False
        
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        goal_dir = math.atan2(goal_y - ry, goal_x - rx)
        distance_to_goal = math.hypot(goal_x - rx, goal_y - ry)
        
        # Check multiple points along the path to goal
        check_distance = min(distance_to_goal, 2.0)
        num_checks = int(check_distance / 0.3)
        
        severe_blockages = 0
        
        for i in range(1, num_checks + 1):
            dist = i * (check_distance / num_checks)
            check_x = rx + dist * math.cos(goal_dir)
            check_y = ry + dist * math.sin(goal_dir)
            
            # Get clearance at this point
            clearance = self.get_clearance_at_point(check_x, check_y)
            
            # Check if obstacle is dangerously close
            if clearance < self.min_obstacle_distance * 0.8:  # More strict
                # Check obstacle density in this area
                cell_x, cell_y = self.world_to_grid(check_x, check_y)
                if cell_x is not None and cell_y is not None:
                    # Check 3x3 area around the point
                    obstacle_density = 0
                    for dx in range(-1, 2):
                        for dy in range(-1, 2):
                            nx, ny = cell_x + dx, cell_y + dy
                            if 0 <= nx < self.obstacle_grid_size and 0 <= ny < self.obstacle_grid_size:
                                if self.obstacle_grid[nx, ny] > 0.5:  # Strong obstacle
                                    obstacle_density += 1
                    
                    if obstacle_density >= 4:  # At least 4 cells in 3x3 are obstacles
                        severe_blockages += 1
        
        # Consider it blocked if we have multiple severe blockages
        return severe_blockages >= 2
    def calculate_movement_over_time(self, time_window):
        """Calculate total movement over a time window"""
        if not self.stuck_positions:
            return 0.0
        
        current_time = time.time()
        relevant_positions = [
            pos for pos, t in self.stuck_positions 
            if current_time - t <= time_window
        ]
        
        if len(relevant_positions) < 2:
            return 0.0
        
        # Calculate total path length (not just start-end distance)
        total_movement = 0.0
        for i in range(len(relevant_positions) - 1):
            x1, y1 = relevant_positions[i]
            x2, y2 = relevant_positions[i + 1]
            total_movement += math.hypot(x2 - x1, y2 - y1)
        
        return total_movement
    
    def check_blocking_obstacles(self):
        """Check for blocking obstacles"""
        if self.robot_pose is None or self.goal_pose is None or self.obstacle_grid is None:
            return False
        
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        goal_dir = math.atan2(goal_y - ry, goal_x - rx)
        distance_to_goal = math.hypot(goal_x - rx, goal_y - ry)
        
        check_distance = min(distance_to_goal, 2.0)
        
        # Check multiple directions
        for angle_offset in [-0.2, 0, 0.2]:
            current_dir = goal_dir + angle_offset
            
            for dist in np.arange(0.3, check_distance, 0.2):
                check_x = rx + dist * math.cos(current_dir)
                check_y = ry + dist * math.sin(current_dir)
                
                cell_x, cell_y = self.world_to_grid(check_x, check_y)
                if cell_x is not None and cell_y is not None:
                    if self.obstacle_grid[cell_x, cell_y] > 0.3:
                        clearance = self.get_clearance_at_point(check_x, check_y)
                        if clearance < self.min_obstacle_distance + 0.2:
                            return True
        
        return False
    
    def generate_adaptive_escape_path(self):
        """Generate escape path that avoids oscillation"""
        if not self.robot_pose or not self.goal_pose:
            return

        current_time = time.time()

        # Enforce cooldown between escape attempts
        if current_time - self.last_escape_time < 3.0:
            if self.debug_mode:
                self.get_logger().debug("⏳ Escape on cooldown")
            return

        x = self.robot_pose.pose.pose.position.x
        y = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y

        # **CRITICAL: Check if forward is actually blocked**
        forward_blocked = self.is_forward_path_blocked(x, y, goal_x, goal_y, min_clearance=0.6)

        if not forward_blocked:
            if self.debug_mode:
                self.get_logger().info("✅ Forward path CLEAR - no escape needed")
            self.stuck_positions.clear()
            return

        self.get_logger().warn(f"🔄 Forward BLOCKED - generating escape (attempt #{len(self.escape_path_history)+1})")

        # Find the clearest direction
        clearances = []
        for angle in np.arange(0, 2*math.pi, math.pi/8):
            clearance = self.get_clearance_in_direction(x, y, angle)
            clearances.append((angle, clearance))

        clearances.sort(key=lambda x: x[1], reverse=True)
        best_angle, best_clearance = clearances[0]

        if best_clearance < 0.6:
            self.get_logger().error("🚨 TRULY SURROUNDED - no escape possible")
            return

        # Generate path in best direction
        path = [(x, y)]

        escape_dist = min(1.5, best_clearance * 0.7)
        mid_x = x + escape_dist * math.cos(best_angle)
        mid_y = y + escape_dist * math.sin(best_angle)
        path.append((mid_x, mid_y))

        # Arc back toward goal
        goal_angle = math.atan2(goal_y - mid_y, goal_x - mid_x)
        for i in range(1, 4):
            t = i / 3.0
            angle = best_angle + t * (goal_angle - best_angle)
            px = mid_x + (1.0 - t) * escape_dist * 0.5 * math.cos(angle)
            py = mid_y + (1.0 - t) * escape_dist * 0.5 * math.sin(angle)
            path.append((px, py))

        path.append((goal_x, goal_y))

        self.publish_path(path)
        self.last_escape_time = current_time
        self.escape_path_history.append({'time': current_time, 'path': path, 'angle': best_angle})

        self.get_logger().info(
            f"✅ Escape path generated: angle={math.degrees(best_angle):.0f}°, "
            f"clearance={best_clearance:.2f}m, distance={escape_dist:.2f}m"
        )

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
    def _time_remaining(self):
        """Return seconds left before the optimization timeout."""
        elapsed = time.time() - self.last_optimization_time
        return max(0.0, self.optimization_timeout - elapsed)
    def generate_backward_escape(self, x, y, goal_x, goal_y):
        """Generate backward escape when surrounded"""
        # **NEW: Double-check we're actually surrounded**
        clearances = []
        for angle in np.arange(0, 2*math.pi, math.pi/8):
            clearance = self.get_clearance_in_direction(x, y, angle)
            clearances.append(clearance)

        max_clearance = max(clearances)

        if max_clearance > 0.6:
            # We have clearance in SOME direction - use that instead of backward
            best_idx = clearances.index(max_clearance)
            best_angle = best_idx * math.pi / 8

            if self.debug_mode:
                self.get_logger().info(
                    f"✅ Found clearance ({max_clearance:.2f}m) at {math.degrees(best_angle):.0f}° - "
                    f"using side escape instead of backward"
                )

            # Generate side escape instead
            path = [(x, y)]
            escape_dist = min(1.0, max_clearance * 0.7)
            escape_x = x + escape_dist * math.cos(best_angle)
            escape_y = y + escape_dist * math.sin(best_angle)
            path.append((escape_x, escape_y))
            path.append((goal_x, goal_y))

            self.publish_path(path)
            return

        # Truly surrounded - NOW use backward escape
        if self.debug_mode:
            self.get_logger().warn(f"⚠️ TRULY SURROUNDED (max clearance: {max_clearance:.2f}m) - using backward escape")

        # Get robot's current heading
        quat = self.robot_pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y**2 + quat.z**2))
        
        # Move backward
        path = [(x, y)]
        back_x = x - 0.8 * math.cos(yaw)
        back_y = y - 0.8 * math.sin(yaw)
        path.append((back_x, back_y))
        
        # Then turn and head to goal
        path.append((goal_x, goal_y))
        
        self.publish_path(path)
        self.get_logger().info("🔙 Backward escape path generated")
    
    def analyze_obstacle_situation(self, x, y, goal_x, goal_y):
        """Analyze obstacle situation"""
        analysis = {
            'type': 'unknown',
            'blocked_sides': 0,
            'main_direction': None,
            'clear_directions': [],
            'goal_direction': math.atan2(goal_y - y, goal_x - x),
            'max_clearance': 0.0,
            'max_clearance_dir': None
        }
        
        directions = np.arange(0, 2*math.pi, math.pi/6)
        direction_status = {}
        
        max_clearance = 0
        max_clearance_dir = None
        
        for angle in directions:
            clearance = self.get_clearance_in_direction(x, y, angle)
            
            if clearance > max_clearance:
                max_clearance = clearance
                max_clearance_dir = angle
            
            if clearance < self.min_obstacle_distance + 0.2:
                direction_status[angle] = 'blocked'
                analysis['blocked_sides'] += 1
            else:
                direction_status[angle] = 'clear'
                analysis['clear_directions'].append(angle)
        
        analysis['max_clearance'] = max_clearance
        analysis['max_clearance_dir'] = max_clearance_dir
        
        max_obstacle_score = 0
        main_direction = None
        
        for angle in directions:
            if direction_status[angle] == 'blocked':
                obstacle_density = self.get_obstacle_density_in_direction(x, y, angle)
                if obstacle_density > max_obstacle_score:
                    max_obstacle_score = obstacle_density
                    main_direction = angle
        
        analysis['main_direction'] = main_direction
        
        if analysis['blocked_sides'] >= 6:
            analysis['type'] = 'surrounded'
        elif main_direction is not None:
            analysis['type'] = 'directional_block'
        else:
            analysis['type'] = 'general_stuck'
        
        return analysis
    
    def get_obstacle_density_in_direction(self, x, y, angle):
        """Get obstacle density in direction"""
        if self.obstacle_grid is None:
            return 0.0
        
        cell_x, cell_y = self.world_to_grid(x, y)
        if cell_x is None or cell_y is None:
            return 0.0
        
        density = 0.0
        check_radius = int(1.0 / self.obstacle_grid_resolution)
        
        for dist in range(1, check_radius + 1):
            check_cell_x = cell_x + int(dist * math.cos(angle))
            check_cell_y = cell_y + int(dist * math.sin(angle))
            
            if 0 <= check_cell_x < self.obstacle_grid_size and 0 <= check_cell_y < self.obstacle_grid_size:
                density += self.obstacle_grid[check_cell_x, check_cell_y]
        
        return density / check_radius
    
    def generate_surrounded_escape(self, start_x, start_y, analysis):
        """Generate surrounded escape"""
        path = []
        path.append((start_x, start_y))
        
        # Go toward maximum clearance direction
        if analysis['max_clearance_dir'] is not None and analysis['max_clearance'] > 0.8:
            move_dist = min(analysis['max_clearance'] * 0.7, 1.5)
            waypoint_x = start_x + move_dist * math.cos(analysis['max_clearance_dir'])
            waypoint_y = start_y + move_dist * math.sin(analysis['max_clearance_dir'])
            path.append((waypoint_x, waypoint_y))
        
        return path
    
    def generate_directional_escape(self, start_x, start_y, goal_x, goal_y, analysis):
        """Generate directional escape"""
        path = []
        path.append((start_x, start_y))
        
        blocked_dir = analysis['main_direction']
        goal_dir = analysis['goal_direction']
        
        # Try both sides
        left_dir = blocked_dir + math.pi/2
        right_dir = blocked_dir - math.pi/2
        
        left_clearance = self.get_clearance_in_direction(start_x, start_y, left_dir)
        right_clearance = self.get_clearance_in_direction(start_x, start_y, right_dir)
        
        if left_clearance > right_clearance:
            bypass_dir = left_dir
            bypass_clearance = left_clearance
        else:
            bypass_dir = right_dir
            bypass_clearance = right_clearance
        
        # First bypass point
        bypass_dist = min(bypass_clearance * 0.7, 1.2)
        waypoint1_x = start_x + bypass_dist * math.cos(bypass_dir)
        waypoint1_y = start_y + bypass_dist * math.sin(bypass_dir)
        path.append((waypoint1_x, waypoint1_y))
        
        # Second point toward goal
        waypoint2_x = waypoint1_x + 0.8 * math.cos(goal_dir)
        waypoint2_y = waypoint1_y + 0.8 * math.sin(goal_dir)
        path.append((waypoint2_x, waypoint2_y))
        
        return path
    
    def generate_side_step_escape(self, start_x, start_y, goal_x, goal_y, analysis):
        """Generate side step escape"""
        path = []
        path.append((start_x, start_y))
        
        goal_dir = analysis['goal_direction']
        
        # Check both sides
        left_clearance = self.get_clearance_in_direction(start_x, start_y, goal_dir + math.pi/2)
        right_clearance = self.get_clearance_in_direction(start_x, start_y, goal_dir - math.pi/2)
        
        if left_clearance > right_clearance:
            side_step_angle = goal_dir + math.pi/2
            side_clearance = left_clearance
        else:
            side_step_angle = goal_dir - math.pi/2
            side_clearance = right_clearance
        
        side_dist = min(side_clearance * 0.7, 1.0)
        side_x = start_x + side_dist * math.cos(side_step_angle)
        side_y = start_y + side_dist * math.sin(side_step_angle)
        path.append((side_x, side_y))
        
        forward_x = side_x + 0.6 * math.cos(goal_dir)
        forward_y = side_y + 0.6 * math.sin(goal_dir)
        path.append((forward_x, forward_y))
        
        return path
    
    def get_clearance_in_direction(self, x, y, angle):
        """
        Fused clearance in a given direction: uses direct laser ray cast first,
        then falls back to costmap ray casting. Returns the best available clearance.
        """
        max_distance = 3.0
        step = 0.1

        # 1) Direct laser ray cast (real‑time)
        laser_clearance = self._get_clearance_laser_first(x, y, angle)
        laser_valid = laser_clearance is not None

        # 2) Costmap clearance (persistent, used only if laser invalid)
        costmap_clearance = max_distance
        if not laser_valid and self.obstacle_grid is not None:
            for distance in np.arange(step, max_distance, step):
                check_x = x + distance * math.cos(angle)
                check_y = y + distance * math.sin(angle)
                clearance = self.get_clearance_at_point(check_x, check_y)
                if clearance < 0.2:
                    costmap_clearance = distance - step
                    break

        # 3) Decision: use laser if valid, otherwise costmap
        if laser_valid:
            result = laser_clearance
            self.get_logger().info(
                f"📡 Clearance dir {math.degrees(angle):.1f}°: laser={laser_clearance:.2f}m (costmap={costmap_clearance:.2f}m) → using laser"
            )
        else:
            result = costmap_clearance
            self.get_logger().info(
                f"📡 Clearance dir {math.degrees(angle):.1f}°: costmap only={costmap_clearance:.2f}m"
            )
        return result
    def are_obstacles_nearby(self, robot_x, robot_y, radius=2.0):
        """Return True if any obstacle is within radius meters of the robot."""
        if self.obstacle_grid is None or self.obstacle_grid_origin is None:
            return False
        gx, gy = self.world_to_grid(robot_x, robot_y)
        if gx is None or gy is None:
            return False
        cell_radius = int(radius / self.obstacle_grid_resolution) + 1
        for dx in range(-cell_radius, cell_radius+1):
            for dy in range(-cell_radius, cell_radius+1):
                nx, ny = gx+dx, gy+dy
                if 0 <= nx < self.obstacle_grid_size and 0 <= ny < self.obstacle_grid_size:
                    if self.obstacle_grid[nx, ny] > self.occupied_threshold:
                        return True
        return False
    def is_forward_path_blocked(self, start_x, start_y, goal_x, goal_y, min_clearance=0.6):
        """Check if forward direction (center ±15°) is blocked using fused clearance."""
        if self.obstacle_grid is None and self.laser_scan is None:
            self.get_logger().debug("⚠️ is_forward_path_blocked: no sensor data → assuming not blocked")
            return False

        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        directions = [
            (goal_dir, "center"),
            (goal_dir + math.radians(15), "left"),
            (goal_dir - math.radians(15), "right"),
        ]

        clear_count = 0
        blocked_count = 0
        self.get_logger().debug("🔍 Checking forward path blockage:")

        for check_dir, label in directions:
            clearance = self.get_clearance_in_direction(start_x, start_y, check_dir)
            if clearance >= min_clearance:
                clear_count += 1
                self.get_logger().debug(f"   {label}: {clearance:.2f}m → CLEAR")
            else:
                blocked_count += 1
                self.get_logger().debug(f"   {label}: {clearance:.2f}m → BLOCKED")

        if clear_count >= 2:
            self.get_logger().info(f"✅ Forward path NOT blocked ({clear_count}/3 directions clear)")
            return False
        if blocked_count >= 2:
            self.get_logger().info(f"🚫 Forward path BLOCKED ({blocked_count}/3 directions blocked)")
            return True

        # ambiguous: check corridor width
        corridor = self.get_corridor_width_pythagorean(start_x, start_y, goal_dir)
        blocked = corridor['corridor_width'] < min_clearance
        self.get_logger().info(f"   Ambiguous → corridor width = {corridor['corridor_width']:.2f}m → {'BLOCKED' if blocked else 'CLEAR'}")
        return blocked
    def _check_remaining_path_and_replan(self):
        """
        Check if the remaining part of the current path is still safe.
        If corridor width falls below 0.8m, force an immediate replan.
        """
        if not hasattr(self, 'last_optimized_path') or self.last_optimized_path is None:
            return False
        if self.robot_pose is None or self.goal_pose is None:
            return False

        robot_x = self.robot_pose.pose.pose.position.x
        robot_y = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y

        # Find closest waypoint on the current path
        closest_idx, _ = self._find_closest_waypoint_on_path(robot_x, robot_y, self.last_optimized_path)
        if closest_idx is None or closest_idx >= len(self.last_optimized_path) - 2:
            return False

        remaining = self.last_optimized_path[closest_idx:]
        if len(remaining) < 2:
            return False

        # Compute minimum corridor width on the remaining path
        min_width, _ = self._get_minimum_corridor_width_on_path(remaining)

        # Also compute straight-line distance to goal to avoid replanning when very close
        dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # If remaining corridor is too narrow and we are not extremely close to the goal
        if min_width < 0.8 and dist_to_goal > 0.8:
            self.get_logger().warn(
                f"⚠️ Remaining path corridor width {min_width:.2f}m < 0.8m – forcing replan"
            )
            self.trigger_optimization()
            return True

        return False    
    def optimization_timer_callback(self):
        if self.optimization_active or self.goal_pose is None:
            return
        if self.robot_pose is None:
            return

        robot_x = self.robot_pose.pose.pose.position.x
        robot_y = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y

        # ----- Goal‑approach lock: if close to goal and remaining path safe, stop replanning -----
        dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if dist_to_goal <= 1.5 and hasattr(self, 'last_optimized_path') and self.last_optimized_path:
            if self.is_remaining_path_safe(self.last_optimized_path, robot_x, robot_y, min_clearance=0.45):
                if self.debug_mode:
                    self.get_logger().debug("Goal‑approach lock active – skipping optimisation")
                return

        # ----- NEW: Check if remaining path corridor is still safe -----
        if self._check_remaining_path_and_replan():
            return   # replan triggered, skip this cycle

        current_time = time.time()
        time_since_last = current_time - self.last_optimization_time

        # Faster replanning when obstacles are present
        if hasattr(self, 'obstacle_tracker') and self.obstacle_tracker.get_obstacles():
            min_interval = 0.3
        else:
            min_interval = 1.0 / self.planning_rate

        if time_since_last < min_interval:
            return

        # Skip if robot moved very little and current path safe
        if hasattr(self, 'last_optimized_path') and self.last_optimized_path is not None:
            if self.last_path_position is not None:
                moved = math.hypot(robot_x - self.last_path_position[0],
                                   robot_y - self.last_path_position[1])
                if moved < 0.3:
                    if self.is_remaining_path_safe(self.last_optimized_path, robot_x, robot_y, 0.45):
                        if self.debug_mode:
                            self.get_logger().debug("Skipping optimisation – robot moved <0.3m and path safe")
                        return

        self.trigger_optimization()
    def prevent_zigzag_in_path(self, path, max_angle_change=math.radians(120)):
        """Remove waypoints that cause true zigzag, but never reduce below 5 waypoints."""
        if len(path) < 5:
            return path   # Keep short paths as they are

        filtered = [path[0]]
        for i in range(1, len(path)-1):
            prev = filtered[-1]
            curr = path[i]
            nxt = path[i+1]
            v1 = (curr[0]-prev[0], curr[1]-prev[1])
            v2 = (nxt[0]-curr[0], nxt[1]-curr[1])
            len1 = math.hypot(v1[0], v1[1])
            len2 = math.hypot(v2[0], v2[1])
            if len1 > 0.01 and len2 > 0.01:
                cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1*len2)
                cos_angle = max(-1.0, min(1.0, cos_angle))
                angle = math.acos(cos_angle)
                if angle > max_angle_change:
                    continue   # remove this waypoint
            filtered.append(curr)
        filtered.append(path[-1])

        # If removal reduced too much, keep original
        if len(filtered) < 5:
            self.get_logger().info(f"  Zigzag removal would reduce to {len(filtered)} waypoints – keeping original")
            return path

        self.get_logger().info(f"✅ Removed zigzag: {len(path)} → {len(filtered)} waypoints")
        # Do NOT interpolate to waypoint_count – keep the filtered waypoints as they are
        return filtered
    def trigger_optimization(self):
        if self.optimization_active or not self.goal_pose:
            return
        if self.robot_pose is None:
            return

        robot_x = self.robot_pose.pose.pose.position.x
        robot_y = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y

        # ========== GOAL‑APPROACH LOCK (ADDED) ==========
        # If very close to goal and the remaining part of the current path is safe,
        # skip replanning entirely to avoid last‑meter oscillations.
        dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if dist_to_goal <= 1.5 and hasattr(self, 'last_optimized_path') and self.last_optimized_path:
            if self.is_remaining_path_safe(self.last_optimized_path, robot_x, robot_y, min_clearance=0.45):
                if self.debug_mode:
                    self.get_logger().debug("Goal‑approach lock – skipping trigger_optimization")
                return
        # =================================================

        # ========== SKIP IF CURRENT PATH IS STILL EXCELLENT ==========
        if self._should_skip_replanning(robot_x, robot_y, goal_x, goal_y):
            return

        # ========== ADDITIONAL: Skip if last path is safe and robot is close to it ==========
        if hasattr(self, 'last_optimized_path') and self.last_optimized_path is not None:
            # Compute clearance on the REMAINING part, not the whole path
            closest_idx, _ = self._find_closest_waypoint_on_path(robot_x, robot_y, self.last_optimized_path)
            if closest_idx is not None and closest_idx < len(self.last_optimized_path) - 2:
                remaining = self.last_optimized_path[closest_idx:]
                min_clear = self._get_minimum_clearance_on_path(remaining)
                remaining_len = self._calculate_path_segment_length(remaining)
                # If remaining path has decent clearance and length, skip replanning
                if min_clear >= 0.45 and remaining_len > 0.5:
                    # Also check that robot hasn't deviated far from the path
                    _, dist_to_path = self._find_closest_waypoint_on_path(robot_x, robot_y, self.last_optimized_path)
                    if dist_to_path < 0.5:
                        if self.debug_mode:
                            self.get_logger().debug("Skipping optimisation – current remaining path safe and robot near path")
                        return

        current_time = time.time()
        time_since_last = current_time - self.last_optimization_time

        # Much longer intervals to reduce churn – even with obstacles, wait at least 3 seconds
        min_interval = 4.0   # was 3.0

        if time_since_last < min_interval:
            return

        # ========== MOVEMENT‑BASED SKIP (REDUCES CHURN) ==========
        if hasattr(self, 'last_optimized_path') and self.last_optimized_path is not None:
            if self.last_path_position is not None:
                moved = math.hypot(robot_x - self.last_path_position[0],
                                   robot_y - self.last_path_position[1])
                # Increase movement threshold to 1.5m (was 1.2m)
                if moved < 1.5:
                    # Check remaining path safety again
                    closest_idx, _ = self._find_closest_waypoint_on_path(robot_x, robot_y, self.last_optimized_path)
                    if closest_idx is not None and closest_idx < len(self.last_optimized_path) - 2:
                        remaining = self.last_optimized_path[closest_idx:]
                        if self.is_remaining_path_safe(remaining, robot_x, robot_y, min_clearance=0.45):
                            if self.debug_mode:
                                self.get_logger().debug("Skipping optimisation – robot moved <1.5m and current remaining path safe")
                            return

        self.optimization_active = True
        self.last_optimization_time = time.time()
        self.optimization_count += 1

        self.get_logger().info(
            f"⚙️ trigger_optimization #{self.optimization_count} – "
            f"start=({robot_x:.2f},{robot_y:.2f}) goal=({goal_x:.2f},{goal_y:.2f})"
        )

        try:
            forward_blocked = self.is_forward_path_blocked(robot_x, robot_y, goal_x, goal_y,
                                                           min_clearance=0.6)

            if forward_blocked:
                self.get_logger().warn("🚧 Forward BLOCKED – trying advanced obstacle‑avoiding path first")
                adv_path = self.generate_obstacle_avoiding_path(robot_x, robot_y, goal_x, goal_y)
                if adv_path and len(adv_path) >= 3:
                    min_clear = self._get_minimum_clearance_on_path(adv_path)
                    if min_clear >= 0.25:
                        if len(adv_path) >= 8:
                            adv_path = self.prevent_zigzag_in_path(adv_path)
                        else:
                            self.get_logger().info("  Skipping zigzag removal on short advanced path")
                        self.get_logger().info("✅ Advanced obstacle‑avoiding path accepted – publishing")
                        self.publish_path(adv_path)
                        self.last_optimized_path = adv_path
                        self.optimization_active = False
                        return
                    else:
                        self.get_logger().warn(f"⚠️ Advanced path clearance too low ({min_clear:.2f}m) – trying curved fallback")
                else:
                    self.get_logger().warn("⚠️ Advanced path generation failed – falling back to curved path")

                curved_path = self.generate_curved_path_for_turning(robot_x, robot_y, goal_x, goal_y, force_wide_turn=True)
                if curved_path and len(curved_path) >= 3:
                    if len(curved_path) >= 8:
                        curved_path = self.prevent_zigzag_in_path(curved_path)
                    else:
                        self.get_logger().info("  Skipping zigzag removal on curved path (too short)")
                    self.publish_path(curved_path)
                    self.last_optimized_path = curved_path
                    self.optimization_active = False
                    return
                else:
                    self.get_logger().error("❌ Failed to generate any path for blocked forward")

            else:
                # ========== PROACTIVE CLEARANCE CHECK (FORCE AVOIDANCE) ==========
                goal_dir = math.atan2(goal_y - robot_y, goal_x - robot_x)
                current_fwd_clear = self.get_clearance_in_direction(robot_x, robot_y, goal_dir)
                if current_fwd_clear < 1.0:
                    self.get_logger().warn(
                        f"⚠️ Forward clearance low ({current_fwd_clear:.2f}m) – forcing obstacle‑avoiding path"
                    )
                    adv_path = self.generate_obstacle_avoiding_path(robot_x, robot_y, goal_x, goal_y)
                    if adv_path and len(adv_path) >= 3:
                        self.publish_path(adv_path)
                        self.last_optimized_path = adv_path
                        self.optimization_active = False
                        return
                # ================================================================

                # ========== ENHANCED STRAIGHT PATH HANDLING ==========
                if self.is_straight_path_clear(robot_x, robot_y, goal_x, goal_y):
                    straight_path = self.generate_straight_path_with_waypoints(robot_x, robot_y, goal_x, goal_y)
                    straight_clearance = self._get_minimum_clearance_on_path(straight_path)
                    if straight_clearance >= 0.6:  # increased from 0.5
                        self.get_logger().info("✅ Straight path CONFIRMED CLEAR (clearance ≥0.6m) – using direct interpolation")
                        self.publish_path(straight_path)
                        self.last_optimized_path = straight_path
                        # Continue to run optimization to refine, but robot can follow this safe path
                    else:
                        self.get_logger().warn(
                            f"⚠️ Straight path clearance too low ({straight_clearance:.2f}m) – forcing obstacle-avoiding path"
                        )
                        initial_path = self.generate_obstacle_avoiding_path(robot_x, robot_y, goal_x, goal_y)
                        if initial_path and len(initial_path) >= 3:
                            self.publish_path(initial_path)
                            if self.debug_mode:
                                self.get_logger().info("⚡ Obstacle-avoiding path published (straight unsafe)")
                else:
                    self.get_logger().info("🔄 Straight path not clear – using obstacle‑avoiding path")
                    initial_path = self.generate_obstacle_avoiding_path(robot_x, robot_y, goal_x, goal_y)
                    if initial_path:
                        if len(initial_path) >= 8:
                            initial_path = self.prevent_zigzag_in_path(initial_path)
                        else:
                            self.get_logger().info("  Skipping zigzag removal on initial path (too short)")
                        self.publish_path(initial_path)
                        if self.debug_mode:
                            self.get_logger().info("⚡ Quick-response path published; launching optimization")

            # ========== RUN OPTIMIZATION (INCLUDES PINN) ==========
            if self.use_pinn and self.pinn_service_available:
                self.get_logger().info("PINN  available – running NSGA-II with PINN")
                self.run_enhanced_optimization()# with PINN
            else:
                self.get_logger().info("PINN not available – running NSGA-II without PINN")
                self.run_enhanced_optimization()   # without PINN
            # ====================================================

        except Exception as e:
            self.get_logger().error(f"❌ Optimization failed: {e}")
        finally:
            self.optimization_active = False
    def publish_path_with_hysteresis(self, new_path):
        """
        Publish new path only if it is significantly better than the REMAINING
        portion of the current path. Now requires score >= 0.5 and rejects
        paths that are not at least 30% shorter or 0.2m clearer.
        """
        if new_path is None or len(new_path) < 2:
            return False

        if not hasattr(self, 'last_published_optimized_path') or self.last_published_optimized_path is None:
            self.publish_path(new_path)
            self.last_published_optimized_path = new_path
            if self.debug_mode:
                self.get_logger().info("📤 First path published (no previous path)")
            return True

        old_path = self.last_published_optimized_path
        if self.robot_pose is None:
            self.publish_path(new_path)
            self.last_published_optimized_path = new_path
            return True

        robot_x = self.robot_pose.pose.pose.position.x
        robot_y = self.robot_pose.pose.pose.position.y

        closest_idx, _ = self._find_closest_waypoint_on_path(robot_x, robot_y, old_path)
        if closest_idx is None or closest_idx >= len(old_path) - 2:
            self.publish_path(new_path)
            self.last_published_optimized_path = new_path
            return True

        remaining_old = old_path[closest_idx:]

        old_length = self._calculate_path_segment_length(remaining_old)
        old_clear = self._get_minimum_clearance_on_path(remaining_old)
        old_curv = self.calculate_curvature(np.array([[p[0], p[1]] for p in remaining_old])) if len(remaining_old) >= 3 else 0.0

        new_clear = self._get_minimum_clearance_on_path(new_path)
        new_length = self._calculate_path_segment_length(new_path)
        new_curv = self.calculate_curvature(np.array([[p[0], p[1]] for p in new_path])) if len(new_path) >= 3 else 0.0

        # Hard safety: reject if new clearance is < 0.5m and old clearance >= 0.5m
        if new_clear < 0.5 and old_clear >= 0.5:
            self.get_logger().warn(
                f"⏸️  Rejecting new path: clearance too low ({new_clear:.2f}m) while "
                f"current remaining path has {old_clear:.2f}m clearance – keeping current"
            )
            return False

        # Reject if new clearance is worse by > 0.1m and not at least 30% shorter
        if new_clear < old_clear - 0.1:
            length_ratio = new_length / old_length if old_length > 0 else 1.0
            if length_ratio > 0.7:   # not at least 30% shorter
                self.get_logger().warn(
                    f"⏸️  Rejecting new path: clearance worse ({new_clear:.2f}m vs {old_clear:.2f}m) "
                    f"and not significantly shorter ({length_ratio:.1%} of remaining length)"
                )
                return False

        # Protect curved paths: if old path is curved (curv > 0.3) and has clearance > 0.5,
        # reject straight new paths (curv < 0.2) unless the straight path is at least 40% shorter
        if old_curv > 0.3 and old_clear >= 0.5:
            if new_curv < 0.2:
                if new_length >= old_length * 0.6:
                    self.get_logger().info(
                        f"⏸️  Keeping curved path (curvature {old_curv:.2f}) – new straight path not significantly shorter"
                    )
                    return False

        # Similarity check – reject if paths are too close (Hausdorff distance < 0.6m)
        def path_distance(p1, p2):
            max_min = 0.0
            for a in p1:
                min_d = min(math.hypot(a[0]-b[0], a[1]-b[1]) for b in p2)
                max_min = max(max_min, min_d)
            for b in p2:
                min_d = min(math.hypot(b[0]-a[0], b[1]-a[1]) for a in p1)
                max_min = max(max_min, min_d)
            return max_min

        old_sampled = remaining_old[::2]
        new_sampled = new_path[::2]
        dist = path_distance(old_sampled, new_sampled)
        if dist < 0.6:
            if self.debug_mode:
                self.get_logger().info(f"⏸️  Rejecting new path: too similar to current (distance {dist:.2f}m < 0.6m)")
            return False

        if self.debug_mode:
            self.get_logger().info(
                f"📊 Hysteresis check (robot at idx {closest_idx}/{len(old_path)}):\n"
                f"  Old (remaining): len={old_length:.2f}m, clear={old_clear:.2f}m, curv={old_curv:.3f}\n"
                f"  New (proposed):  len={new_length:.2f}m, clear={new_clear:.2f}m, curv={new_curv:.3f}"
            )

        length_improve = (old_length - new_length) / max(old_length, 0.1)
        clear_improve = (new_clear - old_clear) / 0.5
        curv_improve = old_curv - new_curv
        score = (length_improve * 0.4) + (clear_improve * 0.3) + (curv_improve * 0.3)

        self.get_logger().info(
            f"🔄 Path improvement score: {score:.3f} "
            f"(length: {length_improve:+.1%}, clearance: {clear_improve:+.2f}, curvature: {curv_improve:+.3f})"
        )

        # Require score >= 0.5 (much stricter) and not longer (unless clearance much better)
        if new_length > old_length * 1.1 and new_clear <= old_clear + 0.2:
            self.get_logger().info(f"⏸️  Rejecting new path because it is longer and not much safer")
            return False

        if score >= 0.5:
            self.get_logger().info(f"✅ Publishing new path (score {score:.3f} ≥ 0.5)")
            self.publish_path(new_path)
            self.last_published_optimized_path = new_path
            return True
        else:
            self.get_logger().info(f"⏸️  Keeping current path (score {score:.3f} < 0.5)")
            return False

    def _get_minimum_corridor_width_on_path(self, path, robot_pose=None):
        """
        Compute the minimum corridor width (left_dist + right_dist) along the path.
        Uses the path direction at each waypoint to measure perpendicular obstacles.
        Returns a tuple (min_width, avg_width).
        """
        if len(path) < 2:
            return 0.0, 0.0

        widths = []
        for i in range(1, len(path)):
            prev = path[i-1]
            curr = path[i]
            # Direction of this segment
            dx = curr[0] - prev[0]
            dy = curr[1] - prev[1]
            seg_len = math.hypot(dx, dy)
            if seg_len < 0.01:
                continue
            heading = math.atan2(dy, dx)
            # Get left/right distances at the current waypoint
            perp = self.get_perpendicular_obstacle_distances(curr[0], curr[1], heading)
            # Use the sum of left and right distances as corridor width
            width = perp['left_obstacle_dist'] + perp['right_obstacle_dist']
            widths.append(width)

        if not widths:
            return 0.0, 0.0
        min_width = min(widths)
        avg_width = sum(widths) / len(widths)
        return min_width, avg_width
    def is_remaining_path_safe(self, path, robot_x, robot_y, min_clearance=0.45,
                               max_deviation=0.6, sample_step=2, min_corridor_width=0.8):
        """
        Check if the portion of `path` from the robot's closest waypoint onward is safe.
        - robot must be within `max_deviation` meters of the path.
        - every sampled waypoint must have clearance >= `min_clearance`.
        - the minimum corridor width (left+right) on the remaining part must be >= `min_corridor_width`.
        Returns True only if all conditions hold.
        """
        if not path or len(path) < 2:
            return False

        # Find closest waypoint on the path
        closest_idx, dist_to_path = self._find_closest_waypoint_on_path(robot_x, robot_y, path)
        if closest_idx is None:
            return False

        # Robot too far from path → should replan
        if dist_to_path > max_deviation:
            if self.debug_mode:
                self.get_logger().debug(
                    f"Remaining path unsafe: robot deviated {dist_to_path:.2f}m > {max_deviation:.2f}m"
                )
            return False

        remaining = path[closest_idx:]
        if len(remaining) < 2:
            return False

        # ---- ORIGINAL CLEARANCE CHECK (preserved) ----
        # Check clearance on remaining waypoints (skip first if it's the robot's own position)
        start_i = 1 if len(remaining) > 2 else 0
        step = max(1, int(sample_step))
        min_found = float('inf')
        for i in range(start_i, len(remaining), step):
            x, y = remaining[i]
            c = self.get_clearance_at_point(x, y)
            if c < min_found:
                min_found = c
            if min_found < min_clearance:
                if self.debug_mode:
                    self.get_logger().info(
                        f"Remaining path unsafe: clearance {min_found:.2f}m < {min_clearance:.2f}m"
                    )
                return False

        # ---- NEW CORRIDOR WIDTH CHECK (added) ----
        min_width, _ = self._get_minimum_corridor_width_on_path(remaining)
        if min_width < min_corridor_width:
            if self.debug_mode:
                self.get_logger().info(
                    f"Remaining path unsafe: corridor width {min_width:.2f}m < {min_corridor_width:.2f}m"
                )
            return False

        if self.debug_mode:
            self.get_logger().debug(
                f"Remaining path safe: clearance={min_found:.2f}m, corridor_width={min_width:.2f}m, dist_to_path={dist_to_path:.2f}m"
            )
        return True
    def run_optimization_loop(self, initial_path):
        """
        IMPROVED: Run NSGA-II with better population initialization.
        Initial population includes obstacle-aware variants.
        """
        if not initial_path or len(initial_path) < 2:
            return
        
        start_x = self.robot_pose.pose.pose.position.x
        start_y = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        # Create diverse initial population
        population = []
        
        # Individual 1: Original initial path
        population.append(initial_path)
        
        # Individual 2-4: Variants with different clearance preferences
        for i in range(3):
            variant = self._create_path_variant(
                initial_path, start_x, start_y, goal_x, goal_y, 
                variant_type='clearance_offset', offset=0.3 * (i + 1)
            )
            if variant:
                population.append(variant)
        
        # Individual 5-6: Alternative obstacle-avoiding paths
        for i in range(2):
            alt_path = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
            if alt_path:
                population.append(alt_path)
        
        # Evaluate population with PINN
        evaluated_pop = []
        for i, candidate in enumerate(population):
            if not candidate or len(candidate) < 2:
                continue
            
            # Get PINN evaluation
            pinn_result = self.call_pinn_service_optimized(candidate)
            
            # Compute objectives (returns list of 7 numbers)
            objectives = self.objective_evaluator.evaluate(candidate)
            
            # Validate objectives are numeric list
            if not isinstance(objectives, (list, tuple)) or len(objectives) != 7:
                if self.debug_mode:
                    self.get_logger().warn(f"Invalid objectives format: {objectives}")
                continue
            
            # Add PINN metrics if available
            if pinn_result and pinn_result.get('success'):
                # PINN energy and stability are already included in objectives via evaluate method
                pass
            
            evaluated_pop.append((candidate, objectives))
        
        if not evaluated_pop:
            if self.debug_mode:
                self.get_logger().warn("⚠️ No evaluated individuals - using initial path")
            return
        
        # Select best based on weighted sum of objectives (lower is better)
        best_path = None
        best_score = float('inf')
        for candidate, obj_list in evaluated_pop:
            # Compute weighted sum using objective_weights
            try:
                # Use the same weights as objective_evaluator
                weights = self.objective_weights
                score = sum(w * o for w, o in zip(weights, obj_list))
                if score < best_score:
                    best_score = score
                    best_path = candidate
            except Exception as e:
                if self.debug_mode:
                    self.get_logger().debug(f"Error computing score: {e}")
                continue
        
        if best_path is None:
            if self.debug_mode:
                self.get_logger().warn("⚠️ Could not select best path - using first candidate")
            best_path = evaluated_pop[0][0]
        
        if self.validate_path_safety(best_path, strict=False):
            self.publish_path(best_path)
            if self.debug_mode:
                self.get_logger().info(
                    f"✅ Optimization #{self.optimization_count} found better path "
                    f"(score={best_score:.3f})"
                )
    def _create_path_variant(self, base_path, start_x, start_y, goal_x, goal_y,
                              variant_type='clearance_offset', offset=0.3):
        """Create variant of base path with different characteristics."""
        if variant_type == 'clearance_offset':
            # Shift waypoints to increase clearance
            variant = []
            mid_x = (start_x + goal_x) / 2
            mid_y = (start_y + goal_y) / 2
            
            for i, (x, y) in enumerate(base_path):
                # Calculate angle from center
                dx = x - mid_x
                dy = y - mid_y
                dist = math.hypot(dx, dy)
                
                if dist > 0.1:
                    # Move waypoint outward by offset
                    new_x = x + offset * (dx / dist)
                    new_y = y + offset * (dy / dist)
                    variant.append((new_x, new_y))
                else:
                    variant.append((x, y))
            
            return variant
        
        return None    
    def generate_free_space_focused_path(self, start_x, start_y, goal_x, goal_y):
        """
        Generate a smooth, short, obstacle-aware path with many waypoints.
        Uses iterative forward stepping toward the goal, staying in free space.
        Returns a path with exactly self.waypoint_count points.
        """
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        start_to_goal_dist = math.hypot(goal_x - start_x, goal_y - start_y)

        if start_to_goal_dist < 0.5:
            return [(start_x, start_y), (goal_x, goal_y)]

        # If straight path is already safe, use it (fast)
        if self.is_straight_path_clear(start_x, start_y, goal_x, goal_y):
            return self._interpolate_path(start_x, start_y, goal_x, goal_y,
                                          num_points=self.waypoint_count)

        # ----- Iterative forward stepping (corridor following) -----
        max_steps = 20                 # increased from 12 to get more points
        step_size = 0.25               # small step for smoothness
        path = [(start_x, start_y)]
        current_x, current_y = start_x, start_y

        for _ in range(max_steps):
            dx = goal_x - current_x
            dy = goal_y - current_y
            dist_to_goal = math.hypot(dx, dy)
            if dist_to_goal < step_size * 1.2:
                break

            goal_dir = math.atan2(dy, dx)

            # Sample directions within ±45° of goal direction
            best_angle = None
            best_clearance = -1.0
            for delta in np.linspace(-math.radians(45), math.radians(45), 9):
                angle = goal_dir + delta
                clearance = self.get_clearance_in_direction(current_x, current_y, angle)
                if clearance > best_clearance:
                    best_clearance = clearance
                    best_angle = angle

            if best_angle is None:
                best_angle = goal_dir

            # Move step – but never increase distance to goal
            step = min(step_size, best_clearance * 0.8, dist_to_goal * 0.6)
            next_x = current_x + step * math.cos(best_angle)
            next_y = current_y + step * math.sin(best_angle)

            # Ensure we are actually moving toward the goal
            new_dist = math.hypot(goal_x - next_x, goal_y - next_y)
            if new_dist > dist_to_goal + 0.05:
                # Fallback: step directly toward goal
                next_x = current_x + step * math.cos(goal_dir)
                next_y = current_y + step * math.sin(goal_dir)

            path.append((next_x, next_y))
            current_x, current_y = next_x, next_y

        path.append((goal_x, goal_y))

        # ----- Smooth and enforce clearance -----
        # Interpolate the coarse path to many points (e.g., 50)
        dense = self.smooth_path_for_controller(path, num_points=50)
        # Then interpolate to exactly self.waypoint_count points
        result = self._interpolate_path_to_count(dense, self.waypoint_count)

        # Ensure minimum clearance (gentle enforcement)
        result = self.enforce_path_clearance(result, start_x, start_y, goal_x, goal_y,
                                             min_clearance=0.35, max_iterations=2)
        return result
    def _detect_obstacles_in_cone(self, x, y, direction, max_distance=2.5, cone_angle=math.radians(90)):
        """Return list of (obs_x, obs_y, distance) for obstacles within a forward cone."""
        obstacles = []
        half_angle = cone_angle / 2.0
        step = 0.15
        for dist in np.arange(0.3, max_distance, step):
            for angle_offset in np.linspace(-half_angle, half_angle, 9):
                angle = direction + angle_offset
                check_x = x + dist * math.cos(angle)
                check_y = y + dist * math.sin(angle)
                clearance = self.get_clearance_at_point(check_x, check_y)
                if clearance < 0.3:
                    # Found obstacle
                    obstacles.append((check_x, check_y, dist))
                    break  # only need one per distance
        # Remove duplicates close together
        unique = []
        for o in obstacles:
            if not any(math.hypot(o[0]-u[0], o[1]-u[1]) < 0.3 for u in unique):
                unique.append(o)
        return unique

    def _generate_bezier_around_obstacle(self, start_x, start_y, goal_x, goal_y,
                                         obs_x, obs_y, obs_dist):
        """
        Create a cubic Bezier curve that smoothly goes around a single obstacle.
        The curve will pass on the side with more clearance.
        """
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        # Vector from start to obstacle
        to_obs_x = obs_x - start_x
        to_obs_y = obs_y - start_y
        # Perpendicular direction (left and right)
        perp_x = -math.sin(goal_dir)
        perp_y = math.cos(goal_dir)

        # Determine which side has more clearance
        left_clear = self.get_clearance_in_direction(start_x, start_y, goal_dir + math.pi/2)
        right_clear = self.get_clearance_in_direction(start_x, start_y, goal_dir - math.pi/2)

        # Choose side with larger clearance (or default to left)
        if left_clear >= right_clear:
            side = 1   # left
        else:
            side = -1  # right

        # Control point offset: how far to deviate
        # Larger deviation for closer obstacles
        deviation = max(0.6, min(1.2, 1.2 - obs_dist * 0.3))
        offset_x = deviation * side * perp_x
        offset_y = deviation * side * perp_y

        # Control point 1: start + offset
        cp1_x = start_x + offset_x
        cp1_y = start_y + offset_y
        # Control point 2: goal + offset (mirrored)
        cp2_x = goal_x + offset_x
        cp2_y = goal_y + offset_y

        # Generate cubic Bezier curve with many points
        num_points = self.waypoint_count
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            mt = 1.0 - t
            x = (mt**3 * start_x +
                 3 * mt**2 * t * cp1_x +
                 3 * mt * t**2 * cp2_x +
                 t**3 * goal_x)
            y = (mt**3 * start_y +
                 3 * mt**2 * t * cp1_y +
                 3 * mt * t**2 * cp2_y +
                 t**3 * goal_y)
            path.append((x, y))

        # Ensure clearance along the curve (adjust if needed)
        path = self.enforce_path_clearance(path, start_x, start_y, goal_x, goal_y,
                                           min_clearance=0.35, max_iterations=2)
        return path

    def _generate_corridor_following_path(self, start_x, start_y, goal_x, goal_y):
        """Legacy wrapper – now uses smooth corridor following."""
        return self.generate_smooth_corridor_following_path(start_x, start_y, goal_x, goal_y)
    def find_free_space_directions(self, start_x, start_y, goal_x, goal_y):
        """Find free space directions"""
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        
        free_directions = []
        
        for angle_offset in np.arange(-math.pi, math.pi, math.pi/12):
            angle = goal_dir + angle_offset
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            clearance = self.get_clearance_in_direction(start_x, start_y, angle)
            free_space_score = self.get_free_space_score_in_direction(start_x, start_y, angle)
            
            if clearance > self.preferred_clearance:
                alignment = 1.0 - abs(angle_offset) / math.pi
                score = clearance * 0.4 + free_space_score * 0.4 + alignment * 0.2
                free_directions.append((angle, clearance, score))
        
        free_directions.sort(key=lambda x: x[2], reverse=True)
        
        return free_directions[:self.free_space_sample_count]
    
    def get_free_space_score_in_direction(self, x, y, angle):
        """Get free space score in direction"""
        if self.free_space_grid is None:
            return 0.0
        
        total_score = 0.0
        sample_count = 5
        
        for i in range(1, sample_count + 1):
            distance = i * 0.4
            check_x = x + distance * math.cos(angle)
            check_y = y + distance * math.sin(angle)
            
            cell_x, cell_y = self.world_to_grid(check_x, check_y)
            if cell_x is not None and cell_y is not None:
                total_score += self.free_space_grid[cell_x, cell_y]
        
        return total_score / sample_count
    
    def update_free_space_map(self):
        """Update free space map"""
        if self.obstacle_grid is None or self.free_space_grid is None:
            return
        
        try:
            # Mark free space based on obstacle absence
            for i in range(self.obstacle_grid_size):
                for j in range(self.obstacle_grid_size):
                    is_free = True
                    for di in range(-2, 3):
                        for dj in range(-2, 3):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < self.obstacle_grid_size and 0 <= nj < self.obstacle_grid_size:
                                if self.obstacle_grid[ni, nj] > 0.2:
                                    is_free = False
                                    break
                        if not is_free:
                            break
                    
                    self.free_space_grid[i, j] = 1.0 if is_free else 0.0
        except Exception as e:
            self.get_logger().error(f"Error updating free space map: {e}")
    
    def generate_quick_response_path(self, start_x, start_y, goal_x, goal_y):
        """Generate quick response path - avoid straight path if obstacles exist."""
        distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)

        # If path is very short, still verify it is clear before returning straight
        if distance_to_goal < 0.5:
            return [(start_x, start_y), (goal_x, goal_y)]

        # IMPORTANT: Only use straight line if it is actually clear
        if self.is_straight_path_clear(start_x, start_y, goal_x, goal_y, min(distance_to_goal, 3.0)):
            return [(start_x, start_y), (goal_x, goal_y)]

        # If blocked: generate obstacle-avoiding seed path
        obstacle_avoiding = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
        if obstacle_avoiding and len(obstacle_avoiding) >= 2:
            return obstacle_avoiding

        # Fallback: curved quick path (safer than straight when blocked)
        curved = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
        if curved and len(curved) >= 2:
            return curved

        # Absolute fallback: keep something, but this should be rare
        return [(start_x, start_y), (goal_x, goal_y)]
    
    def generate_straight_path_with_waypoints(self, start_x, start_y, goal_x, goal_y):
        """Generate straight-line path with evenly spaced waypoints - BUG FIX for curved paths"""
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        
        # Handle case where start and goal are identical
        if distance < 0.01:  # Less than 1cm - essentially the same position
            return [(start_x, start_y)]
        
        # One waypoint every 0.3m for smooth control, minimum of 2, maximum of 20
        num_waypoints = min(max(2, int(distance / 0.3) + 1), 20)
        
        path = []
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)  # Safe: num_waypoints >= 2 always
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            path.append((x, y))
        
        return path
    
    def generate_curved_path_for_turning(self, start_x, start_y, goal_x, goal_y, force_wide_turn=False):
        self.get_logger().info("🌀 [CURVED] generate_curved_path_for_turning() called")
        if self.robot_pose is not None:
            robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
        else:
            robot_yaw = 0.0

        goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)
        distance = math.hypot(goal_x - start_x, goal_y - start_y)

        left_clear = self.get_clearance_in_direction(start_x, start_y, goal_angle + math.pi/2)
        right_clear = self.get_clearance_in_direction(start_x, start_y, goal_angle - math.pi/2)
        self.get_logger().info(f"   Left clearance = {left_clear:.2f}m, Right clearance = {right_clear:.2f}m")

        # Choose the side with more clearance
        if left_clear > right_clear:
            turn_dir = 1
            free_clearance = left_clear
            self.get_logger().info("   Turning LEFT")
        else:
            turn_dir = -1
            free_clearance = right_clear
            self.get_logger().info("   Turning RIGHT")

        forward_clearance = self.get_clearance_in_direction(start_x, start_y, goal_angle)
        self.get_logger().info(f"   Forward clearance = {forward_clearance:.2f}m")

        # Dynamic turn angle: more aggressive when forward is very blocked or force_wide_turn
        if force_wide_turn:
            turn_angle = math.radians(120.0)   # wide turn up to 120°
        elif forward_clearance < 0.5:
            turn_angle = math.radians(90.0)
        elif forward_clearance < 0.8:
            turn_angle = math.radians(75.0)
        elif forward_clearance < 1.2:
            turn_angle = math.radians(60.0)
        else:
            turn_angle = math.radians(50.0)

        # ---- INCREASE TURN ANGLE BY 30% ----
        turn_angle = turn_angle * 1.3
        self.get_logger().info(f"   Increased turn angle (×1.3) = {math.degrees(turn_angle):.1f}°")

        # Limit by available side clearance (don't turn into an obstacle)
        if not force_wide_turn:
            max_turn_by_clearance = math.radians(min(90.0, math.degrees(math.asin(min(1.0, free_clearance / 1.5)))))
            turn_angle = min(turn_angle, max_turn_by_clearance)
        else:
            max_turn_by_clearance = math.radians(min(120.0, math.degrees(math.asin(min(1.0, free_clearance / 1.2)))))
            turn_angle = min(turn_angle, max_turn_by_clearance)

        # Also ensure we turn at least as much as needed to face the goal
        angle_to_goal = abs(math.atan2(math.sin(goal_angle - robot_yaw),
                                       math.cos(goal_angle - robot_yaw)))
        turn_angle = max(turn_angle, min(angle_to_goal, math.radians(120.0)))

        self.get_logger().info(f"   Final turn angle = {math.degrees(turn_angle):.1f}°")

        # Intermediate distance – go further when forward is blocked
        if force_wide_turn:
            intermediate_dist = min(distance * 1.2, 5.0)   # go further for wide turn
        elif forward_clearance < 0.5:
            intermediate_dist = min(distance * 0.9, 4.0)
        else:
            intermediate_dist = min(distance * 0.7, 2.5)

        intermediate_angle = robot_yaw + turn_dir * turn_angle

        mid_x = start_x + intermediate_dist * math.cos(intermediate_angle)
        mid_y = start_y + intermediate_dist * math.sin(intermediate_angle)

        # Control point offset: larger when turn is more aggressive
        if force_wide_turn:
            cp_offset = intermediate_dist * 0.8   # more curved
        else:
            cp_offset = intermediate_dist * 0.6

        p1_x = start_x + cp_offset * math.cos(intermediate_angle)
        p1_y = start_y + cp_offset * math.sin(intermediate_angle)
        p2_x = goal_x - cp_offset * math.cos(goal_angle)
        p2_y = goal_y - cp_offset * math.sin(goal_angle)

        num_waypoints = max(12, int(distance / 0.25))
        path = []
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            mt = 1.0 - t
            bx = (mt**3 * start_x + 3.0 * mt**2 * t * p1_x +
                  3.0 * mt * t**2 * p2_x + t**3 * goal_x)
            by = (mt**3 * start_y + 3.0 * mt**2 * t * p1_y +
                  3.0 * mt * t**2 * p2_y + t**3 * goal_y)
            path.append((bx, by))

        path = self.enforce_path_clearance(path, start_x, start_y, goal_x, goal_y,
                                           min_clearance=0.25, max_iterations=3)
        path = self.prevent_zigzag_in_path(path)
        min_clear = self._get_minimum_clearance_on_path(path)
        self.get_logger().info(f"   Curved path generated with min clearance = {min_clear:.2f}m")
        return path
    def is_path_significantly_better(self, new_path, old_path):
        """
        Check if new path is significantly better than old path.
        STRONGLY prefers keeping curved paths over straight ones.
        """
        if old_path is None or len(old_path) < 2:
            return True
        if len(new_path) < 2:
            return False
        
        # Compute metrics
        new_length = self.calculate_path_length(new_path)
        old_length = self.calculate_path_length(old_path)
        new_clearance = self._get_minimum_clearance_on_path(new_path)
        old_clearance = self._get_minimum_clearance_on_path(old_path)
        new_curvature = self.calculate_curvature(np.array([[p[0], p[1]] for p in new_path]))
        old_curvature = self.calculate_curvature(np.array([[p[0], p[1]] for p in old_path]))
        
        CURVED_THRESHOLD = 0.3  # same as MIN_TURN_CURVATURE_THRESHOLD
        
        # If current path is curved, strongly resist switching to straight
        if old_curvature > CURVED_THRESHOLD:
            # New path is straight (or nearly straight) -> reject unless massively better
            if new_curvature <= CURVED_THRESHOLD:
                # Only allow straight replacement if clearance is >0.5m better AND path is much shorter
                if new_clearance > old_clearance + 0.5 and new_length < old_length * 0.8:
                    return True
                return False  # Keep curved path
            
            # Both curved: normal improvement thresholds
            length_improvement = (old_length - new_length) / old_length if old_length > 0.1 else 0
            clearance_improvement = new_clearance - old_clearance
            
            if length_improvement >= 0.15:      # 15% shorter
                return True
            if clearance_improvement >= 0.2 and new_clearance >= 0.5:   # +20cm clearance
                return True
            if abs(length_improvement) < 0.05 and abs(clearance_improvement) < 0.1:
                if new_curvature < old_curvature * 0.9:  # 10% smoother
                    return True
            return False
        
        # Current path is straight: easier to replace
        length_improvement = (old_length - new_length) / old_length if old_length > 0.1 else 0
        clearance_improvement = new_clearance - old_clearance
        
        if length_improvement >= 0.10:      # 10% shorter
            return True
        if clearance_improvement >= 0.15 and new_clearance >= 0.5:
            return True
        if abs(length_improvement) < 0.05 and abs(clearance_improvement) < 0.1:
            if new_curvature < old_curvature * 0.9:
                return True
        return False

    def run_enhanced_optimization(self):
        try:
            start_time = time.time()
            self.optimization_count += 1

            if self.robot_pose is None or self.goal_pose is None:
                self.optimization_active = False
                return

            start_x = self.robot_pose.pose.pose.position.x
            start_y = self.robot_pose.pose.pose.position.y
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y

            if not self.should_update_path(start_x, start_y):
                self.optimization_active = False
                return

            distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)
            if distance_to_goal < self.goal_completion_distance:
                self.get_logger().info(f"🎯 Already at goal ({distance_to_goal:.2f}m)")
                self.optimization_active = False
                return

            forward_blocked = self.is_forward_path_blocked(start_x, start_y, goal_x, goal_y,
                                                           min_clearance=0.6)

            if forward_blocked:
                if self.debug_mode:
                    self.get_logger().warn("🔄 Forward BLOCKED – publishing curved path")
                curved_path = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
                if curved_path and len(curved_path) >= 3:
                    if len(curved_path) >= 8:
                        curved_path = self.prevent_zigzag_in_path(curved_path)
                    else:
                        self.get_logger().info("  Skipping zigzag removal on curved path (too short)")
                    self.publish_path(curved_path)
                    self.last_optimized_path = curved_path
                    self.optimization_active = False
                    return

            best_path = self.run_enhanced_nsga2_with_free_space(
                start_x, start_y, goal_x, goal_y,
                self.population_size, self.generations,
                seed_path=None, force_curved=forward_blocked
            )

            if best_path is None or len(best_path) < 2:
                self.get_logger().warn("⚠️ NSGA-II produced no path – using fallback")
                best_path = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
                if best_path is None or len(best_path) < 2:
                    best_path = [(start_x, start_y), (goal_x, goal_y)]
                if len(best_path) >= 8:
                    best_path = self.prevent_zigzag_in_path(best_path)
                else:
                    self.get_logger().info("  Skipping zigzag removal on fallback path (too short)")
                self.publish_path(best_path)
                self.last_optimized_path = best_path
                self.optimization_active = False
                return

            if len(best_path) >= 8:
                best_path = self.prevent_zigzag_in_path(best_path)
            else:
                self.get_logger().info("  Skipping zigzag removal on best path (too short)")

            self.publish_path_with_hysteresis(best_path)
            self.last_optimized_path = best_path

            elapsed = time.time() - start_time
            if self.debug_mode:
                self.get_logger().info(f"✅ Optimization #{self.optimization_count} in {elapsed:.2f}s")

        except Exception as e:
            self.get_logger().error(f"❌ Optimization error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.optimization_active = False
    def get_free_space_bonus(self, path_array):
        """
        Compute a bonus (0.0 to 1.0) representing the average free space score
        along the path. Higher values mean the path stays in more open areas.
        """
        if path_array.size == 0 or self.free_space_grid is None:
            return 0.0

        total_score = 0.0
        for x, y in path_array:
            total_score += self.get_free_space_score_at_point(x, y)
        return total_score / len(path_array) if len(path_array) > 0 else 0.0
    def run_enhanced_nsga2_with_free_space(self, start_x, start_y, goal_x, goal_y,
                                          pop_size, generations, seed_path=None, force_curved=False):
        """
        Enhanced NSGA-II with STRICT SELECTION and early termination.
        ALWAYS returns a valid path – uses fallback if no individuals survive.
        """
        HARD_TIMEOUT = max(1.0, self.optimization_timeout - 1.0)
        optimization_start = time.time()

        population = self.initialize_enhanced_population(
            start_x, start_y, goal_x, goal_y, pop_size, skip_straight=force_curved
        )

        # --- Enforce clearance on initial population ---
        for idx in range(len(population)):
            population[idx] = self.enforce_path_clearance(
                population[idx], start_x, start_y, goal_x, goal_y,
                min_clearance=0.35, max_iterations=2
            )

        if seed_path is not None:
            if self.debug_mode:
                self.get_logger().info("🌱 Seeding population with curved path")
            seed_path_corrected = (
                [(start_x, start_y)] + list(seed_path[1:-1]) + [(goal_x, goal_y)]
            )
            seed_path_corrected = self.enforce_path_clearance(
                seed_path_corrected, start_x, start_y, goal_x, goal_y,
                min_clearance=0.35, max_iterations=2
            )
            population[0] = seed_path_corrected

        straight_line_length = math.hypot(goal_x - start_x, goal_y - start_y)
        evaluated_individuals = []
        evaluated_objectives = []
        total_pinn_calls = 0
        total_pinn_success = 0
        bad_generations_count = 0
        MAX_BAD_GENERATIONS = 2

        # --- MAIN NSGA-II LOOP ---
        for gen in range(generations):
            if time.time() - optimization_start > HARD_TIMEOUT:
                if self.debug_mode:
                    self.get_logger().debug(f"Hard timeout after {gen} generations")
                break

            all_objectives = []
            valid_individuals = []
            valid_paths = []
            generation_has_valid = False
            gen_dynamic_costs = []  # for logging

            # --- Evaluate individuals ---
            for idx, individual in enumerate(population):
                elapsed = time.time() - self.last_optimization_time
                if elapsed > self.optimization_timeout:
                    break

                path_array = np.array([[p[0], p[1]] for p in individual])

                min_clear = self._get_minimum_clearance_on_path(individual)
                if min_clear < 0.0:
                    if self.debug_mode:
                        self.get_logger().debug(f"  Gen {gen}: Individual {idx} rejected (negative clearance {min_clear:.2f}m)")
                    continue
                obstacle_cost = self.get_enhanced_obstacle_cost(path_array)
                if obstacle_cost == float('inf'):
                    if self.debug_mode:
                        self.get_logger().debug(f"  Gen {gen}: Individual {idx} rejected (infinite obstacle cost)")
                    continue

                # ========== NEW: Dynamic obstacle cost ==========
                dynamic_cost = self.get_dynamic_obstacle_cost(individual)
                total_obstacle_cost = obstacle_cost + dynamic_cost * 0.5
                gen_dynamic_costs.append(dynamic_cost)
                if dynamic_cost > 10.0 and self.debug_mode:
                    self.get_logger().info(
                        f"  Gen {gen}: Individual {idx} dynamic cost = {dynamic_cost:.2f}"
                    )
                # ================================================

                objectives = self.objective_evaluator.evaluate_all_objectives_with_timeout(
                    path_array,
                    total_obstacle_cost,
                    0.0,
                    straight_line_length,
                    use_pinn=self.use_pinn and self.pinn_service_available,
                    timeout=0.5
                )

                if not isinstance(objectives, (list, tuple)) or len(objectives) < 7:
                    continue
                if any(math.isnan(o) or math.isinf(o) for o in objectives):
                    continue

                all_objectives.append(objectives)
                valid_individuals.append(idx)
                valid_paths.append(individual)
                evaluated_individuals.append(individual)
                evaluated_objectives.append(objectives)
                generation_has_valid = True

            # Log dynamic cost summary for this generation
            if gen_dynamic_costs and self.debug_mode:
                self.get_logger().info(
                    f"📊 Gen {gen}: dynamic costs min={min(gen_dynamic_costs):.2f}, "
                    f"max={max(gen_dynamic_costs):.2f}, avg={sum(gen_dynamic_costs)/len(gen_dynamic_costs):.2f}"
                )

            # --- Check generation validity ---
            if not all_objectives:
                bad_generations_count += 1
                if self.debug_mode:
                    self.get_logger().warn(
                        f"⚠️ Gen {gen}: NO valid individuals ({bad_generations_count}/{MAX_BAD_GENERATIONS})"
                    )

                if bad_generations_count >= MAX_BAD_GENERATIONS:
                    if self.debug_mode:
                        self.get_logger().warn("❌ Too many bad generations - terminating optimization")
                    break

                population = self.initialize_enhanced_population(
                    start_x, start_y, goal_x, goal_y, pop_size, skip_straight=force_curved
                )
                for idx in range(len(population)):
                    population[idx] = self.enforce_path_clearance(
                        population[idx], start_x, start_y, goal_x, goal_y,
                        min_clearance=0.35, max_iterations=2
                    )
                continue
            else:
                bad_generations_count = max(0, bad_generations_count - 1)

            if time.time() - self.last_optimization_time > self.optimization_timeout:
                break

            # --- Selection and reproduction ---
            if len(all_objectives) >= 2:
                try:
                    selected_indices = nsga2_selection(valid_paths, all_objectives, pop_size)
                    new_population = [valid_paths[i] for i in selected_indices]

                    while len(new_population) < pop_size:
                        if time.time() - self.last_optimization_time > self.optimization_timeout:
                            break

                        if random.random() < self.crossover_rate and len(new_population) >= 2:
                            parent1 = random.choice(new_population)
                            parent2 = random.choice(new_population)
                            child = self.crossover_paths(parent1, parent2)
                        else:
                            child = random.choice(new_population)

                        child = self.mutate_with_free_space_and_corridor(
                            child, start_x, start_y, goal_x, goal_y, min_clearance=0.35
                        )
                        child = self.enforce_path_clearance(
                            child, start_x, start_y, goal_x, goal_y,
                            min_clearance=0.35, max_iterations=1
                        )
                        new_population.append(child)

                    population = new_population

                except Exception as e:
                    if self.debug_mode:
                        self.get_logger().warn(f"⚠️ NSGA-II selection failed: {e}")
                    population = self.enhanced_selection(population, valid_individuals, all_objectives, pop_size)

            else:
                population = self.enhanced_selection(population, valid_individuals, all_objectives, pop_size)

            if time.time() - self.last_optimization_time > self.optimization_timeout:
                break

        # --- Select best individual from ALL evaluated ---
        if not evaluated_objectives:
            if self.debug_mode:
                self.get_logger().warn("⚠️ No evaluated individuals found – using fallback path")
            fallback = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
            if fallback is None or len(fallback) < 2:
                fallback = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
            if fallback is None or len(fallback) < 2:
                fallback = [(start_x, start_y), (goal_x, goal_y)]
            return self.prevent_zigzag_in_path(fallback) if len(fallback) >= 8 else fallback

        best_idx = self.get_best_individual(evaluated_objectives)

        # Safety: if best_idx is None or out of range, use first valid individual
        if best_idx is None or best_idx >= len(evaluated_individuals):
            if self.debug_mode:
                self.get_logger().warn(f"⚠️ Invalid best_idx={best_idx}, len(evaluated)={len(evaluated_individuals)} – using first valid")
            best_idx = 0
            if best_idx >= len(evaluated_individuals):
                best_idx = 0

        best_individual = evaluated_individuals[best_idx]

        # Final validation: if best individual has zero clearance, generate a fresh safe path
        min_final_clear = self._get_minimum_clearance_on_path(best_individual)
        if min_final_clear < 0.2:
            if self.debug_mode:
                self.get_logger().warn(f"⚠️ Best path clearance {min_final_clear:.2f}m < 0.2m – using fallback")
            fallback = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
            if fallback is None or len(fallback) < 2:
                fallback = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
            if fallback is None or len(fallback) < 2:
                fallback = [(start_x, start_y), (goal_x, goal_y)]
            best_individual = fallback

        if len(best_individual) < 2:
            if self.debug_mode:
                self.get_logger().warn("⚠️ Best individual too short – using fallback")
            best_individual = [(start_x, start_y), (goal_x, goal_y)]

        # Apply smoothing if enabled (commented out as per original)
        #if self.path_smoothing:
        #    best_individual = self.smooth_path(best_individual)

        # ---- POST-SMOOTHING CLEARANCE CHECK (ADDED) ----
        final_clearance = self._get_minimum_clearance_on_path(best_individual)
        if final_clearance < 0.2:
            if self.debug_mode:
                self.get_logger().warn(f"⚠️ Smoothed path clearance {final_clearance:.2f}m < 0.2m – using fallback")
            fallback = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
            if fallback is None or len(fallback) < 2:
                fallback = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
            if fallback is None or len(fallback) < 2:
                fallback = [(start_x, start_y), (goal_x, goal_y)]
            best_individual = fallback
        # --------------------------------------------------

        self.last_pinn_calls = total_pinn_calls
        self.last_pinn_success = total_pinn_success

        if self.debug_mode:
            elapsed = time.time() - optimization_start
            self.get_logger().info(
                f"✅ NSGA-II completed: {len(evaluated_individuals)} individuals evaluated, "
                f"best clearance={final_clearance:.2f}m, time={elapsed:.2f}s"
            )

        return best_individual
    def get_enhanced_obstacle_cost(self, path_array):
        """
        Calculate obstacle cost with balanced hard/soft enforcement.
        Hard rejection only when clearance < 0.2m (was 0.25m).
        Soft penalty for 0.2–1.0m zone.
        """
        if len(path_array) == 0:
            return float('inf')
        
        HARD_CLEARANCE = 0.20          # reject only if closer than 0.2m (was 0.25)
        DANGER_ZONE_START = 0.20
        DANGER_ZONE_END = 1.0
        total_cost = 0.0
        
        for idx, point in enumerate(path_array):
            x, y = point[0], point[1]
            min_dist = self.get_min_distance_to_obstacle(x, y)
            
            if min_dist < HARD_CLEARANCE:
                if self.debug_mode:
                    self.get_logger().debug(
                        f"🚨 Hard reject – waypoint {idx}: clearance={min_dist:.3f}m < {HARD_CLEARANCE}m"
                    )
                return float('inf')
            
            if min_dist < DANGER_ZONE_END:
                penalty = (1.0 - (min_dist - DANGER_ZONE_START) / (DANGER_ZONE_END - DANGER_ZONE_START)) \
                          * self.obstacle_penalty_weight
                total_cost += penalty
        
        return total_cost
    
    def generate_obstacle_avoiding_path(self, start_x, start_y, goal_x, goal_y):
        self.get_logger().info("🔍 [PATH] generate_obstacle_avoiding_path() called")
        if not self.robot_pose or not self.goal_pose:
            self.get_logger().warn("❌ [PATH] No robot_pose or goal_pose")
            return None

        distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance_to_goal < 0.5:
            return [(start_x, start_y), (goal_x, goal_y)]

        # Check straight path
        if self.is_straight_path_clear(start_x, start_y, goal_x, goal_y, distance_to_goal):
            self.get_logger().info("✅ [PATH] Straight path clear")
            return self._interpolate_path(start_x, start_y, goal_x, goal_y,
                                          num_points=self.waypoint_count)

        self.get_logger().info("🔄 [PATH] Straight blocked – analysing free space")
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        left_clear = self.get_clearance_in_direction(start_x, start_y, goal_dir + math.pi/2)
        right_clear = self.get_clearance_in_direction(start_x, start_y, goal_dir - math.pi/2)
        forward_clear = self.get_clearance_in_direction(start_x, start_y, goal_dir)
        self.get_logger().info(f"   Left:{left_clear:.2f}m, Right:{right_clear:.2f}m, Forward:{forward_clear:.2f}m")

        # Estimate path length for left and right deviations
        def estimate_deviation_length(offset):
            mid_x = (start_x + goal_x) / 2.0
            mid_y = (start_y + goal_y) / 2.0
            perp_x = -math.sin(goal_dir)
            perp_y = math.cos(goal_dir)
            ctrl_x = mid_x + offset * perp_x
            ctrl_y = mid_y + offset * perp_y
            length = 0.0
            prev_x, prev_y = start_x, start_y
            for t in np.linspace(0.1, 1.0, 10):
                u = 1.0 - t
                x = u*u*start_x + 2*u*t*ctrl_x + t*t*goal_x
                y = u*u*start_y + 2*u*t*ctrl_y + t*t*goal_y
                length += math.hypot(x - prev_x, y - prev_y)
                prev_x, prev_y = x, y
            return length

        # Only consider sides with at least 0.6m clearance
        viable_sides = []
        if left_clear >= 0.6:
            left_offset = min(1.5, left_clear * 0.6)
            left_len = estimate_deviation_length(left_offset)
            viable_sides.append((left_offset, left_len, 'LEFT'))
        if right_clear >= 0.6:
            right_offset = -min(1.5, right_clear * 0.6)
            right_len = estimate_deviation_length(right_offset)
            viable_sides.append((right_offset, right_len, 'RIGHT'))

        # Try each viable side (prefer shorter path)
        viable_sides.sort(key=lambda x: x[1])
        for offset, length, side in viable_sides:
            # Increase offset by 20% as before
            offset = offset * 1.2
            self.get_logger().info(f"   → Trying {side} deviation (offset={offset:.2f}m)")
            dev_path = self._generate_deviated_path_adaptive(
                start_x, start_y, goal_x, goal_y, goal_dir, offset)
            if dev_path:
                refined = self.refine_deviated_path_smoothly(
                    dev_path, start_x, start_y, goal_x, goal_y, offset)
                # Check clearance of refined path
                min_clr = self._get_minimum_clearance_on_path(refined)
                if min_clr >= 0.35:
                    self.get_logger().info(f"✅ {side} deviation path safe (clearance={min_clr:.2f}m)")
                    if len(refined) >= 10:
                        refined = self.prevent_zigzag_in_path(refined)
                    return refined
                else:
                    self.get_logger().warn(f"⚠️ {side} deviation path unsafe (clearance={min_clr:.2f}m) – trying next side")
            else:
                self.get_logger().warn(f"⚠️ {side} deviation generation failed")

        # If no deviation works, fallback to curved path (usually safer)
        self.get_logger().info("   → No safe deviation found, trying curved turning path")
        curved_path = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
        if curved_path:
            min_clr = self._get_minimum_clearance_on_path(curved_path)
            if min_clr >= 0.35:
                self.get_logger().info(f"✅ Curved path safe (clearance={min_clr:.2f}m)")
                return curved_path
            else:
                self.get_logger().warn(f"⚠️ Curved path also unsafe (clearance={min_clr:.2f}m)")

        # Last resort: corridor following
        self.get_logger().info("   → Falling back to corridor‑following")
        path = self.generate_smooth_corridor_following_path(start_x, start_y, goal_x, goal_y)
        if path:
            min_clr = self._get_minimum_clearance_on_path(path)
            if min_clr >= 0.35:
                return path

        # Ultimate fallback: straight line with warning (should never happen)
        self.get_logger().error("❌ No safe path found – using straight line (extremely dangerous!)")
        return self._interpolate_path(start_x, start_y, goal_x, goal_y, num_points=self.waypoint_count)
    def generate_smooth_corridor_following_path(self, start_x, start_y, goal_x, goal_y):
        self.get_logger().info("🛤️ [SMOOTH_CORRIDOR] Generating smooth corridor‑following path")
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance < 0.5:
            return [(start_x, start_y), (goal_x, goal_y)]
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        path = [(start_x, start_y)]
        x, y = start_x, start_y
        step_size = min(0.5, distance / 15)
        prev_dir = goal_dir
        for _ in range(15):
            remaining = math.hypot(goal_x - x, goal_y - y)
            if remaining < step_size * 1.2:
                break
            best_angle = None
            best_clear = -1.0
            for delta in np.linspace(-math.radians(75), math.radians(75), 5):
                cand_angle = goal_dir + delta
                # direction smoothing: 70% goal, 30% previous
                smoothed = 0.7 * cand_angle + 0.3 * prev_dir
                smoothed = math.atan2(math.sin(smoothed), math.cos(smoothed))
                clearance = self.get_clearance_in_direction(x, y, smoothed)
                if clearance > best_clear:
                    best_clear = clearance
                    best_angle = smoothed
            if best_angle is None:
                best_angle = goal_dir
            final_angle = 0.8 * best_angle + 0.2 * prev_dir
            final_angle = math.atan2(math.sin(final_angle), math.cos(final_angle))
            step = min(step_size, best_clear * 0.7, remaining * 0.7)
            x += step * math.cos(final_angle)
            y += step * math.sin(final_angle)
            path.append((x, y))
            prev_dir = final_angle
        path.append((goal_x, goal_y))
        path = self.smooth_path_for_controller(path, num_points=max(30, len(path)*2))
        return self._interpolate_path_to_count(path, self.waypoint_count)
    def _generate_deviated_path_adaptive(self, start_x, start_y, goal_x, goal_y,
                                         goal_direction, side_offset):
        self.get_logger().info(f"🔧 [DEVIATION] Creating adaptive deviated path with offset = {side_offset:.2f}m")
        path = [(start_x, start_y)]
        perp_dir = goal_direction + math.pi / 2
        total_dist = math.hypot(goal_x - start_x, goal_y - start_y)

        mid_x = (start_x + goal_x) / 2.0
        mid_y = (start_y + goal_y) / 2.0
        ctrl_x = mid_x + side_offset * math.cos(perp_dir)
        ctrl_y = mid_y + side_offset * math.sin(perp_dir)
        self.get_logger().debug(f"   Control point: ({ctrl_x:.2f}, {ctrl_y:.2f})")

        num_waypoints = max(8, int(total_dist / 0.3))
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            u = 1.0 - t
            x = u*u * start_x + 2*u*t * ctrl_x + t*t * goal_x
            y = u*u * start_y + 2*u*t * ctrl_y + t*t * goal_y
            path.append((x, y))

        nudged = 0
        for i in range(1, len(path) - 1):
            x, y = path[i]
            clearance = self.get_clearance_at_point(x, y)
            if clearance < 0.25:
                push = 0.15 * (1.0 - clearance / 0.25)
                new_x = x + push * (1 if side_offset > 0 else -1) * math.cos(perp_dir)
                new_y = y + push * (1 if side_offset > 0 else -1) * math.sin(perp_dir)
                path[i] = (new_x, new_y)
                nudged += 1
        if nudged:
            self.get_logger().info(f"   Nudged {nudged} waypoints")

        path = self.smooth_path_for_controller(path, num_points=50)
        return self._interpolate_path_to_count(path, self.waypoint_count)
    def refine_deviated_path_smoothly(self, base_path, start_x, start_y, goal_x, goal_y,
                                      lateral_offset, smoothness_factor=0.7):
        """Apply cosine taper to deviation for smooth lateral offset."""
        if not base_path or len(base_path) < 2:
            return base_path
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        perp_dir = goal_dir + math.pi / 2
        refined = []
        for i, (x, y) in enumerate(base_path):
            pos_ratio = i / (len(base_path) - 1) if len(base_path) > 1 else 0.0
            # Cosine taper: max offset at middle, zero at ends
            taper = (1.0 - abs(math.cos(pos_ratio * math.pi))) / 2.0
            adapt_offset = lateral_offset * taper
            new_x = x + adapt_offset * math.cos(perp_dir)
            new_y = y + adapt_offset * math.sin(perp_dir)
            # Avoid pushing into obstacle
            if self.get_clearance_at_point(new_x, new_y) < self.min_obstacle_distance:
                adapt_offset *= 0.5
                new_x = x + adapt_offset * math.cos(perp_dir)
                new_y = y + adapt_offset * math.sin(perp_dir)
            refined.append((new_x, new_y))
        refined = self.smooth_path_for_controller(refined, num_points=max(20, len(refined)))
        self.get_logger().info(f"✅ Refined deviated path: offset={lateral_offset:.2f}m")
        return self._interpolate_path_to_count(refined, self.waypoint_count)
    def _generate_weaving_path(self, start_x, start_y, goal_x, goal_y):
        self.get_logger().info("🧵 [WEAVE] Attempting to generate weaving path")
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance < 1.0:
            self.get_logger().warn("   Distance too short for weaving")
            return None

        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        perp = math.pi / 2
        num_samples = 15
        gap_centers = []
        for i in range(1, num_samples):
            t = i / num_samples
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            left_dist = 0.0
            right_dist = 0.0
            max_search = 2.0
            step = 0.05
            for d in np.arange(step, max_search, step):
                lx = x + d * math.cos(goal_dir + perp)
                ly = y + d * math.sin(goal_dir + perp)
                if self.get_clearance_at_point(lx, ly) < 0.3:
                    left_dist = d - step
                    break
            for d in np.arange(step, max_search, step):
                rx = x - d * math.cos(goal_dir + perp)
                ry = y - d * math.sin(goal_dir + perp)
                if self.get_clearance_at_point(rx, ry) < 0.3:
                    right_dist = d - step
                    break
            corridor_width = left_dist + right_dist
            if corridor_width > 0.8:
                offset = (right_dist - left_dist) / 2.0
                gap_centers.append((t * distance, offset))
                self.get_logger().debug(f"   Gap at t={t:.2f}: width={corridor_width:.2f}m, offset={offset:.2f}m")

        if not gap_centers:
            self.get_logger().warn("   No suitable gaps found")
            return None

        waypoints = [(start_x, start_y)]
        last_t, last_offset = 0.0, 0.0
        for dist_along, offset in gap_centers:
            t = dist_along / distance
            for s in np.linspace(0, 1, 5):
                tt = last_t + s * (t - last_t)
                off = last_offset + s * (offset - last_offset)
                x = start_x + tt * (goal_x - start_x) + off * math.cos(goal_dir + perp)
                y = start_y + tt * (goal_y - start_y) + off * math.sin(goal_dir + perp)
                waypoints.append((x, y))
            last_t, last_offset = t, offset
        waypoints.append((goal_x, goal_y))
        return self.smooth_weaving_path_optimized(waypoints, num_smooth_points=50)
    def smooth_weaving_path_optimized(self, waypoints, num_smooth_points=50):
        if len(waypoints) < 2:
            return waypoints
        if len(waypoints) < 4:
            return self.smooth_path_for_controller(waypoints, num_smooth_points)
        try:
            from scipy.interpolate import CubicSpline
            arr = np.array(waypoints)
            # arc-length parameterisation
            dists = [0.0]
            for i in range(1, len(waypoints)):
                dx = waypoints[i][0] - waypoints[i-1][0]
                dy = waypoints[i][1] - waypoints[i-1][1]
                dists.append(dists[-1] + math.hypot(dx, dy))
            total = dists[-1]
            if total < 1e-6:
                return waypoints
            t = np.array(dists) / total
            cs_x = CubicSpline(t, arr[:, 0], bc_type='natural')
            cs_y = CubicSpline(t, arr[:, 1], bc_type='natural')
            t_smooth = np.linspace(0, 1, num_smooth_points)
            smooth = list(zip(cs_x(t_smooth), cs_y(t_smooth)))
            self.get_logger().debug(f"Smoothed weaving path: {len(waypoints)} → {len(smooth)} points")
            return self._interpolate_path_to_count(smooth, self.waypoint_count)
        except ImportError:
            self.get_logger().debug("CubicSpline not available, using linear smoothing")
            return self.smooth_path_for_controller(waypoints, num_smooth_points)
    def _generate_deviated_path_aggressive(self, start_x, start_y, goal_x, goal_y,
                                           goal_direction, side_offset):
        path = [(start_x, start_y)]
        perp_dir = goal_direction + math.pi/2
        total_dist = math.hypot(goal_x - start_x, goal_y - start_y)

        # 1/3 waypoint
        px1 = start_x + (total_dist / 3) * math.cos(goal_direction) + side_offset * math.cos(perp_dir)
        py1 = start_y + (total_dist / 3) * math.sin(goal_direction) + side_offset * math.sin(perp_dir)
        if self.get_clearance_at_point(px1, py1) < 0.35:
            side_offset *= 0.5
            px1 = start_x + (total_dist / 3) * math.cos(goal_direction) + side_offset * math.cos(perp_dir)
            py1 = start_y + (total_dist / 3) * math.sin(goal_direction) + side_offset * math.sin(perp_dir)
        path.append((px1, py1))

        # 2/3 waypoint
        px2 = start_x + (2 * total_dist / 3) * math.cos(goal_direction) + side_offset * 0.8 * math.cos(perp_dir)
        py2 = start_y + (2 * total_dist / 3) * math.sin(goal_direction) + side_offset * 0.8 * math.sin(perp_dir)
        if self.get_clearance_at_point(px2, py2) < 0.35:
            px2 = start_x + (2 * total_dist / 3) * math.cos(goal_direction) + (side_offset * 0.4) * math.cos(perp_dir)
            py2 = start_y + (2 * total_dist / 3) * math.sin(goal_direction) + (side_offset * 0.4) * math.sin(perp_dir)
        path.append((px2, py2))

        # near goal
        px3 = goal_x + (side_offset * 0.3) * math.cos(perp_dir)
        py3 = goal_y + (side_offset * 0.3) * math.sin(perp_dir)
        if self.get_clearance_at_point(px3, py3) < 0.35:
            px3, py3 = goal_x, goal_y
        path.append((px3, py3))
        path.append((goal_x, goal_y))

        # smooth and interpolate
        path = self.smooth_path_for_controller(path, num_points=50)
        return self._interpolate_path_to_count(path, self.waypoint_count)
    def _generate_deviated_path(self, start_x, start_y, goal_x, goal_y, 
                                 goal_direction, side_offset, distance):
        """Generate path with perpendicular deviation to avoid obstacles."""
        # Perpendicular direction (90 degrees from goal direction)
        perp_dir = goal_direction + math.pi / 2
        
        # Midpoint with side deviation
        mid_x = start_x + (distance / 2) * math.cos(goal_direction) + side_offset * math.cos(perp_dir)
        mid_y = start_y + (distance / 2) * math.sin(goal_direction) + side_offset * math.sin(perp_dir)
        
        # Generate smooth path: start → deviated midpoint → goal
        path = []
        
        # Segment 1: start to midpoint (5 points for smooth curve)
        for i in range(5):
            t = i / 4.0
            x = start_x + t * (mid_x - start_x)
            y = start_y + t * (mid_y - start_y)
            path.append((x, y))
        
        # Segment 2: midpoint to goal (5 points)
        for i in range(1, 6):
            t = i / 5.0
            x = mid_x + t * (goal_x - mid_x)
            y = mid_y + t * (goal_y - mid_y)
            path.append((x, y))
        
        # Validate this path has good clearance
        if self._get_minimum_clearance_on_path(path) > 0.35:
            return path
        return None
    def _get_minimum_clearance_on_path(self, path):
        if not path or len(path) < 2:
            return 0.0
        min_clearance = float('inf')
        # Start from index 1 to skip the robot's own position
        for idx in range(1, len(path)):
            x, y = path[idx]
            clearance = self.get_clearance_at_point(x, y)
            if clearance < min_clearance:
                min_clearance = clearance
            if clearance < 0.1:
                self.get_logger().warn(
                    f"  Waypoint {idx} ({x:.2f},{y:.2f}) has clearance {clearance:.3f}m"
                )
        # If all waypoints (except start) are safe, return the minimum
        return max(0.0, min_clearance) if min_clearance != float('inf') else 1.0

    def _interpolate_path(self, start_x, start_y, goal_x, goal_y, num_points=8):
        """Generate smoothly interpolated path from start to goal."""
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            path.append((x, y))
        return path

    def _generate_offset_path(self, start_x, start_y, goal_x, goal_y, offset=3.0, smoothness=0.7):
        """Generate path with perpendicular offset from direct line."""
        direct_vector = np.array([goal_x - start_x, goal_y - start_y])
        distance = np.linalg.norm(direct_vector)
        
        if distance < 0.1:
            return None
        
        direction = direct_vector / distance
        perpendicular = np.array([-direction[1], direction[0]])
        
        # Middle point offset perpendicular to goal direction
        middle_x = start_x + 0.5 * direct_vector[0] + offset * perpendicular[0]
        middle_y = start_y + 0.5 * direct_vector[1] + offset * perpendicular[1]
        
        # Check if middle point is valid (not in occupied cell)
        mid_clearance = self.get_clearance_at_point(middle_x, middle_y)
        if mid_clearance < 0.3:
            # Middle point is occupied, try smaller offset
            offset *= 0.7
            middle_x = start_x + 0.5 * direct_vector[0] + offset * perpendicular[0]
            middle_y = start_y + 0.5 * direct_vector[1] + offset * perpendicular[1]
        
        # Generate path via quadratic Bezier: start → middle → goal
        waypoints = [(start_x, start_y)]
        num_intermediate = max(8, int(distance / 0.3))
        
        for t in np.linspace(0, 1, num_intermediate):
            # Quadratic Bezier formula
            bx = (1 - t) ** 2 * start_x + 2 * (1 - t) * t * middle_x + t ** 2 * goal_x
            by = (1 - t) ** 2 * start_y + 2 * (1 - t) * t * middle_y + t ** 2 * goal_y
            waypoints.append((bx, by))
        
        waypoints.append((goal_x, goal_y))
        return waypoints

    def _generate_arc_path(self, start_x, start_y, goal_x, goal_y):
        """Generate a circular arc path around obstacles."""
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        
        # Calculate arc parameters - bulge out from straight line
        # Use quadratic Bezier curve for smooth arc
        bulge_distance = distance * 0.35  # How far to deviate from straight line
        bulge_dir = goal_dir + math.pi / 2  # Perpendicular direction
        
        # Control point for Bezier curve
        ctrl_x = start_x + (distance / 2) * math.cos(goal_dir) + bulge_distance * math.cos(bulge_dir)
        ctrl_y = start_y + (distance / 2) * math.sin(goal_dir) + bulge_distance * math.sin(bulge_dir)
        
        # Generate path using quadratic Bezier
        path = []
        num_segments = 10
        for i in range(num_segments + 1):
            t = i / num_segments
            # Quadratic Bezier: P(t) = (1-t)²P0 + 2(1-t)tP1 + t²P2
            u = 1.0 - t
            x = u*u*start_x + 2*u*t*ctrl_x + t*t*goal_x
            y = u*u*start_y + 2*u*t*ctrl_y + t*t*goal_y
            path.append((x, y))
        
        return path

    def _generate_multi_step_path(self, start_x, start_y, goal_x, goal_y, sidestep_distance=2.5):
        """Generate path with multiple sidesteps to avoid obstacles."""
        direct_vector = np.array([goal_x - start_x, goal_y - start_y])
        distance = np.linalg.norm(direct_vector)
        
        if distance < 0.1:
            return None
        
        direction = direct_vector / distance
        perpendicular = np.array([-direction[1], direction[0]])
        
        waypoints = [(start_x, start_y)]
        
        # Take sidesteps: 1/3 distance forward, sidestep, 2/3 distance forward, sidestep, etc.
        step_positions = [0.25, 0.5, 0.75]
        
        for step_pos in step_positions:
            # Forward position
            forward_x = start_x + step_pos * direct_vector[0]
            forward_y = start_y + step_pos * direct_vector[1]
            
            # Sidestep
            sidestep_x = forward_x + sidestep_distance * perpendicular[0]
            sidestep_y = forward_y + sidestep_distance * perpendicular[1]
            
            # Check clearance; if occupied, try other direction
            if self.get_clearance_at_point(sidestep_x, sidestep_y) < 0.3:
                sidestep_x = forward_x - sidestep_distance * perpendicular[0]
                sidestep_y = forward_y - sidestep_distance * perpendicular[1]
            
            waypoints.append((sidestep_x, sidestep_y))
            waypoints.append((forward_x, forward_y))
        
        waypoints.append((goal_x, goal_y))
        
        # Smooth the path
        return self.smooth_path_for_controller(waypoints, num_points=20)

    def _score_path_clearance(self, path):
        """Score path based on minimum clearance and smoothness."""
        if not path or len(path) < 2:
            return -float('inf')
        
        clearances = [self.get_clearance_at_point(x, y) for x, y in path]
        min_clearance = min(clearances)
        avg_clearance = np.mean(clearances)
        
        # Penalize sharp turns
        turn_penalty = 0.0
        for i in range(1, len(path) - 1):
            p1, p2, p3 = path[i-1:i+2]
            dx1, dy1 = p2[0] - p1[0], p2[1] - p1[1]
            dx2, dy2 = p3[0] - p2[0], p3[1] - p2[1]
            
            d1, d2 = math.hypot(dx1, dy1), math.hypot(dx2, dy2)
            if d1 > 0.01 and d2 > 0.01:
                cos_angle = (dx1 * dx2 + dy1 * dy2) / (d1 * d2)
                cos_angle = max(-1, min(1, cos_angle))
                turn_angle = abs(math.acos(cos_angle))
                turn_penalty += turn_angle ** 2
        
        # Score: higher min clearance + higher avg clearance - turn penalty
        score = min_clearance * 5.0 + avg_clearance * 1.0 - turn_penalty * 0.1
        
        return score
    
    def initialize_enhanced_population(self, start_x, start_y, goal_x, goal_y, pop_size, skip_straight=False):
        population = []
        n = max(self.waypoint_count, 2)

        # 1. Straight line (if allowed)
        if not skip_straight:
            straight_path = [
                (start_x + i/(n-1)*(goal_x-start_x),
                 start_y + i/(n-1)*(goal_y-start_y))
                for i in range(n)
            ]
            population.append(straight_path)

        # 2. Obstacle-avoiding path
        avoid_path = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
        if avoid_path:
            population.append(avoid_path)

        # 3. Improved free-space focused path (with gating)
        free_space_path = self.generate_free_space_focused_path(start_x, start_y, goal_x, goal_y)
        if free_space_path:
            # Evaluate quality before adding
            length = self.calculate_path_length(np.array(free_space_path))
            straight_len = math.hypot(goal_x - start_x, goal_y - start_y)
            min_clear = self._get_minimum_clearance_on_path(free_space_path)
            # Only add if not too long and clearance is acceptable
            if length <= 1.5 * max(straight_len, 0.1) and min_clear >= 0.35:
                population.append(free_space_path)
            elif self.debug_mode:
                self.get_logger().debug(f"Free-space seed rejected: length ratio={length/straight_len:.2f}, clearance={min_clear:.2f}")

        # 4. Straight candidate (interpolated)
        straight_candidate = self.generate_straight_path_with_waypoints(start_x, start_y, goal_x, goal_y)
        if straight_candidate:
            population.append(straight_candidate)

        # 5. Fill remaining slots with improved goal-directed random paths
        remaining = pop_size - len(population)
        for i in range(remaining):
            random_path = self._generate_goal_directed_random_path(start_x, start_y, goal_x, goal_y, n)
            population.append(random_path)

        return population
    def mutate_with_free_space(self, path, start_x, start_y, goal_x, goal_y):
        """Mutate with free space"""
        mutated = []
        
        for i, (x, y) in enumerate(path):
            if i == 0:
                mutated.append((start_x, start_y))
            elif i == len(path) - 1:
                mutated.append((goal_x, goal_y))
            elif random.random() < self.mutation_rate:
                current_clearance = self.get_clearance_at_point(x, y)
                current_free_score = self.get_free_space_score_at_point(x, y)
                
                best_x, best_y = x, y
                best_score = current_clearance * 0.6 + current_free_score * 0.4
                
                for _ in range(5):
                    angle = random.uniform(0, 2 * math.pi)
                    dist = random.uniform(0, 0.6)
                    
                    candidate_x = x + dist * math.cos(angle)
                    candidate_y = y + dist * math.sin(angle)
                    
                    clearance = self.get_clearance_at_point(candidate_x, candidate_y)
                    free_score = self.get_free_space_score_at_point(candidate_x, candidate_y)
                    candidate_score = clearance * 0.6 + free_score * 0.4
                    
                    if candidate_score > best_score:
                        best_score = candidate_score
                        best_x, best_y = candidate_x, candidate_y
                
                mutated.append((best_x, best_y))
            else:
                mutated.append((x, y))
        
        return mutated
    
    def get_free_space_score_at_point(self, x, y):
        """Get free space score at point"""
        if self.free_space_grid is None:
            return 0.0
        cell_x, cell_y = self.world_to_grid(x, y)
        if cell_x is None or cell_y is None:
            return 0.0
        
        score = 0.0
        count = 0
        
        for di in range(-3, 4):
            for dj in range(-3, 4):
                ni, nj = cell_x + di, cell_y + dj
                if 0 <= ni < self.obstacle_grid_size and 0 <= nj < self.obstacle_grid_size:
                    score += self.free_space_grid[ni, nj]
                    count += 1
        
        return score / max(count, 1)
    
    def smooth_path_with_free_space(self, path):
        """Smooth path with free space"""
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev = smoothed[-1]
            curr = path[i]
            next_pt = path[i + 1]
            
            smooth_x = 0.7 * curr[0] + 0.15 * (prev[0] + next_pt[0])
            smooth_y = 0.7 * curr[1] + 0.15 * (prev[1] + next_pt[1])
            
            free_score = self.get_free_space_score_at_point(smooth_x, smooth_y)
            orig_free_score = self.get_free_space_score_at_point(curr[0], curr[1])
            
            if free_score > orig_free_score * 0.8:
                smoothed.append((smooth_x, smooth_y))
            else:
                best_x, best_y = curr
                best_score = orig_free_score
                
                for _ in range(3):
                    angle = random.uniform(0, 2 * math.pi)
                    dist = random.uniform(0, 0.3)
                    
                    candidate_x = curr[0] + dist * math.cos(angle)
                    candidate_y = curr[1] + dist * math.sin(angle)
                    
                    candidate_score = self.get_free_space_score_at_point(candidate_x, candidate_y)
                    if candidate_score > best_score:
                        best_score = candidate_score
                        best_x, best_y = candidate_x, candidate_y
                
                smoothed.append((best_x, best_y))
        
        smoothed.append(path[-1])
        return smoothed
    def smooth_path(self, path, iterations=3, kernel=0.5):
        """
        Smooth the path using iterative moving average (low‑pass filter).
        Completely removes zigzag while preserving overall shape.
        """
        if len(path) < 5:
            return path

        smoothed = list(path)
        for _ in range(iterations):
            new_path = []
            for i in range(len(smoothed)):
                if i == 0 or i == len(smoothed) - 1:
                    new_path.append(smoothed[i])
                else:
                    prev = smoothed[i-1]
                    curr = smoothed[i]
                    nxt = smoothed[i+1]
                    # Weighted average: [kernel/2, 1-kernel, kernel/2]
                    w = kernel / 2.0
                    new_x = w * prev[0] + (1.0 - kernel) * curr[0] + w * nxt[0]
                    new_y = w * prev[1] + (1.0 - kernel) * curr[1] + w * nxt[1]
                    new_path.append((new_x, new_y))
            smoothed = new_path
        # Re‑interpolate to original number of points (keeps length)
        return self._interpolate_path_to_count(smoothed, len(path))

    def get_best_individual(self, objectives_list):
        """
        Select best individual using multi-criteria ranking.
        ALWAYS returns an index (0 if no valid candidate) to avoid None.
        """
        if not objectives_list:
            return 0  # fallback index

        valid_candidates = []
        
        for idx, objectives in enumerate(objectives_list):
            if not isinstance(objectives, (list, tuple)) or len(objectives) < 7:
                continue
            
            try:
                objectives = [float(o) for o in objectives]
                
                # Reject infinite or NaN costs
                if any(math.isnan(o) or math.isinf(o) for o in objectives):
                    continue
                
                # Obstacle cost (index 2) – relax threshold to 2.5 * weight
                obstacle_cost = objectives[2]
                if obstacle_cost > self.obstacle_penalty_weight * 2.5:
                    continue
                
                # Path length sanity check
                path_length = objectives[0]
                if path_length < 0 or path_length > 100.0:
                    continue
                
                valid_candidates.append((idx, objectives))
                
            except (TypeError, ValueError):
                continue
        
        if not valid_candidates:
            # No valid candidate → return the first index (0) if possible
            if len(objectives_list) > 0:
                self.get_logger().warn("⚠️ No valid candidates in get_best_individual – returning index 0")
                return 0
            return 0  # fallback
        
        # Multi-tier ranking: obstacle cost → clearance → length → curvature
        def rank_candidate(item):
            idx, objectives = item
            obstacle_cost = objectives[2]
            clearance_cost = objectives[1]
            length_cost = objectives[0]
            curvature_cost = objectives[3]
            return (obstacle_cost, clearance_cost, length_cost, curvature_cost)
        
        valid_candidates.sort(key=rank_candidate)
        best_idx, best_objectives = valid_candidates[0]
        
        if self.debug_mode:
            self.get_logger().info(
                f"🏆 Best individual #{best_idx}: obstacle={best_objectives[2]:.2f}, "
                f"clearance={best_objectives[1]:.2f}, length={best_objectives[0]:.2f}"
            )
        
        return best_idx
    def enhanced_selection(self, population, valid_indices, valid_objectives, pop_size):
        """
        Select individuals using STRICT multi-criteria ranking.
        Only accepts candidates with valid objectives.
        """
        if not valid_indices or not valid_objectives:
            if self.debug_mode:
                self.get_logger().warn("⚠️ Enhanced selection: no valid candidates")
            return population[:pop_size]

        # Build mapping with validation
        obj_map = {}
        for i, orig_idx in enumerate(valid_indices):
            if i < len(valid_objectives):
                objs = valid_objectives[i]
                # Only add if objectives are valid
                if isinstance(objs, (list, tuple)) and len(objs) >= 7:
                    if not any(math.isnan(o) or math.isinf(o) for o in objs):
                        obj_map[orig_idx] = objs

        if not obj_map:
            if self.debug_mode:
                self.get_logger().warn("⚠️ Enhanced selection: all objectives invalid")
            return population[:pop_size]

        def rank_candidate(idx):
            objs = obj_map.get(idx)
            if objs is None:
                return (float('inf'), float('inf'), float('inf'))
            try:
                return (objs[2], objs[1], objs[0])  # obstacle, clearance, length
            except (IndexError, TypeError):
                return (float('inf'), float('inf'), float('inf'))

        sorted_indices = sorted(obj_map.keys(), key=rank_candidate)
        new_population = []
        
        for idx in sorted_indices:
            if idx < len(population):
                new_population.append(population[idx])
            if len(new_population) >= pop_size:
                break

        while len(new_population) < pop_size:
            if obj_map:
                rand_idx = random.choice(list(obj_map.keys()))
                if rand_idx < len(population):
                    new_population.append(population[rand_idx])
            else:
                new_population.append(population[0])

        return new_population[:pop_size]
    
    def crossover_paths(self, path1, path2):
        """Crossover paths"""
        if len(path1) < 3 or len(path2) < 3:
            return path1 if len(path1) > len(path2) else path2
        
        crossover_point1 = random.randint(1, len(path1) - 2)
        crossover_point2 = random.randint(1, len(path2) - 2)
        
        child = path1[:crossover_point1] + path2[crossover_point2:]
        
        child[0] = path1[0]
        child[-1] = path1[-1] if random.random() < 0.5 else path2[-1]
        
        return child
    
    def generate_obstacle_aware_path(self, start_x, start_y, goal_x, goal_y):
        """Generate obstacle aware path"""
        clear_path = self.find_clear_path_around_obstacles(start_x, start_y, goal_x, goal_y)
        
        if clear_path and len(clear_path) > 2:
            return clear_path
        
        return self.generate_waypoint_path_around_obstacles(start_x, start_y, goal_x, goal_y)
    
    def find_clear_path_around_obstacles(self, start_x, start_y, goal_x, goal_y):
        """Find clear path around obstacles"""
        path = []
        path.append((start_x, start_y))
        
        max_attempts = 5
        current_x, current_y = start_x, start_y
        
        for attempt in range(max_attempts):
            clear_directions = self.find_clear_directions_toward_goal(current_x, current_y, goal_x, goal_y)
            
            if not clear_directions:
                break
            
            best_dir, best_clearance = clear_directions[0]
            move_dist = min(1.5, best_clearance * 0.8)
            
            next_x = current_x + move_dist * math.cos(best_dir)
            next_y = current_y + move_dist * math.sin(best_dir)
            
            path.append((next_x, next_y))
            current_x, current_y = next_x, next_y
            
            dist_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)
            if dist_to_goal < 1.0:
                break
        
        path.append((goal_x, goal_y))
        return path
    
    def find_clear_directions_toward_goal(self, x, y, goal_x, goal_y):
        """Find clear directions toward goal"""
        goal_dir = math.atan2(goal_y - y, goal_x - x)
        
        clear_directions = []
        
        for angle_offset in np.arange(-math.pi/2, math.pi/2 + 0.1, math.pi/12):
            angle = goal_dir + angle_offset
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            clearance = self.get_clearance_in_direction(x, y, angle)
            if clearance > 0.5:
                alignment = 1.0 - abs(angle_offset) / (math.pi/2)
                score = clearance * 0.7 + alignment * 0.3
                clear_directions.append((angle, clearance, score))
        
        clear_directions.sort(key=lambda x: x[2], reverse=True)
        
        return [(angle, clearance) for angle, clearance, _ in clear_directions]
    
    def generate_waypoint_path_around_obstacles(self, start_x, start_y, goal_x, goal_y):
        """Generate waypoint path around obstacles"""
        path = []
        path.append((start_x, start_y))
        
        dx_total = goal_x - start_x
        dy_total = goal_y - start_y
        distance = math.hypot(dx_total, dy_total)
        
        if distance < 2.0:
            path.append((goal_x, goal_y))
            return path
        
        num_waypoints = min(3, int(distance / 1.5))
        
        for i in range(1, num_waypoints + 1):
            t = i / (num_waypoints + 1)
            
            base_x = start_x + t * dx_total
            base_y = start_y + t * dy_total
            
            safe_x, safe_y = self.find_safe_position_near(base_x, base_y, 0.5)
            path.append((safe_x, safe_y))
        
        path.append((goal_x, goal_y))
        return path
    
    def find_safe_position_near(self, x, y, radius):
        """Find safe position near"""
        best_x, best_y = x, y
        best_clearance = self.get_clearance_at_point(x, y)
        
        for _ in range(10):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(0, radius)
            
            candidate_x = x + dist * math.cos(angle)
            candidate_y = y + dist * math.sin(angle)
            
            clearance = self.get_clearance_at_point(candidate_x, candidate_y)
            if clearance > best_clearance:
                best_clearance = clearance
                best_x, best_y = candidate_x, candidate_y
        
        return best_x, best_y
    
    def laser_callback(self, msg: LaserScan):
        """Laser callback – update costmap and dynamic tracker."""
        if len(msg.ranges) == 0:
            self.get_logger().warn("⚠️ Empty laser scan received")
            return

        self.laser_scan = msg
        self.laser_frame = msg.header.frame_id

        self._laser_cb_counter = getattr(self, '_laser_cb_counter', 0) + 1

        valid_count = sum(1 for r in msg.ranges if 0.1 < r < 10.0)
        if valid_count == 0:
            self.get_logger().warn("⚠️ No valid ranges in laser scan")
            return

        if self.debug_mode and self._laser_cb_counter % 50 == 0:
            min_range = min(r for r in msg.ranges if 0.1 < r < 10.0) if valid_count else float('nan')
            max_range = max(r for r in msg.ranges if 0.1 < r < 10.0) if valid_count else float('nan')
            self.get_logger().info(
                f"Laser: {len(msg.ranges)} points, {valid_count} valid, "
                f"range=[{min_range:.2f}, {max_range:.2f}]m, frame={msg.header.frame_id}"
            )

        if self.scan_angles is None or len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = [
                float(msg.angle_min + i * msg.angle_increment)
                for i in range(len(msg.ranges))
            ]

        if self.obstacle_tracker is not None and self.robot_pose is not None:
            robot_x = self.robot_pose.pose.pose.position.x
            robot_y = self.robot_pose.pose.pose.position.y
            robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)

            # LiDAR extrinsics (from tf2_echo base_link -> rslidar)
            sensor_x = 0.15
            sensor_y = 0.0
            sensor_yaw = 0.0

            # Extract scan timestamp (ROS time as float seconds)
            scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self.obstacle_tracker.update_from_scan(
                msg.ranges, self.scan_angles,
                robot_x, robot_y, robot_yaw,
                0.0, 0.0,               # robot velocity not available
                sensor_x, sensor_y, sensor_yaw,
                scan_stamp              # new argument
            )
    def _publish_costmap(self, robot_x: float, robot_y: float):
        """Publish obstacle grid as OccupancyGrid for RViz"""
        try:
            # Throttle publication (every 10 publishes)
            self._costmap_pub_counter = getattr(self, '_costmap_pub_counter', 0) + 1
            if self._costmap_pub_counter % 10 != 0:
                return
            
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = 'odom'
            
            grid_msg.info.resolution = self.obstacle_grid_resolution
            grid_msg.info.width = self.obstacle_grid_size
            grid_msg.info.height = self.obstacle_grid_size
            
            # Set origin to grid corner
            grid_msg.info.origin.position.x = self.obstacle_grid_origin[0]
            grid_msg.info.origin.position.y = self.obstacle_grid_origin[1]
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0
            
            # Convert confidence (0.0–1.0) to occupancy (-1 to 100)
            # -1 = unknown, 0 = free, 100 = occupied
            grid_data = []
            for val in self.obstacle_grid.flatten():
                if val < 0.2:
                    grid_data.append(0)      # Free
                elif val > self.occupied_threshold:
                    grid_data.append(100)    # Occupied
                else:
                    grid_data.append(int(val * 100))  # Partially occupied
            
            grid_msg.data = grid_data
            self.costmap_pub.publish(grid_msg)
            
            if self.debug_mode and getattr(self, '_costmap_debug_counter', 0) % 100 == 0:
                occupied = sum(1 for v in grid_data if v > 50)
                self.get_logger().info(
                    f"Costmap: {occupied}/{len(grid_data)} cells occupied, "
                    f"origin=({self.obstacle_grid_origin[0]:.2f}, {self.obstacle_grid_origin[1]:.2f})"
                )
            self._costmap_debug_counter = getattr(self, '_costmap_debug_counter', 0) + 1
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish costmap: {e}")
    def validate_path_clearance(self, path_array, min_clearance=0.35):
        """
        HARD CONSTRAINT: Validate that ALL waypoints maintain minimum clearance.
        Returns True if path is valid, False otherwise.
        Also returns actual minimum clearance found.
        """
        if not hasattr(self, 'obstacle_grid') or self.obstacle_grid is None:
            return True, min_clearance
        
        min_clearance_found = float('inf')
        
        for waypoint_idx, (x, y) in enumerate(path_array):
            # Get clearance at this waypoint
            clearance = self.get_clearance_at_point(x, y)
            
            # Track minimum found
            if clearance < min_clearance_found:
                min_clearance_found = clearance
            
            # HARD CONSTRAINT: reject if below minimum
            if clearance < min_clearance:
                if self.debug_mode:
                    self.get_logger().debug(
                        f"Path validation FAILED: Waypoint {waypoint_idx} "
                        f"clearance={clearance:.3f}m < {min_clearance}m required"
                    )
                return False, min_clearance_found
        
        return True, min_clearance_found
    def validate_path_with_strict_clearance(self, path_array):
        """
        STRICT validation: Check clearance, corridor width, and safety margins.
        Returns: (is_valid, min_clearance, corridor_width, safety_score)
        """
        # First check: hard clearance constraint
        is_safe, min_clearance = self.validate_path_clearance(path_array, self.min_obstacle_distance)
        
        if not is_safe:
            return False, min_clearance, 0.0, 0.0
        
        # Second check: calculate corridor width (now that we know all points are safe)
        corridor_width = self.calculate_corridor_width(path_array)
        
        # Third check: penalty for being too close to constraint boundary
        safety_margin = 0.10  # 10cm safety margin from minimum clearance
        clearance_safety = min_clearance - self.min_obstacle_distance
        
        if clearance_safety < safety_margin:
            # Path is barely meeting constraint - low safety score
            safety_score = clearance_safety / safety_margin
        else:
            # Path has good safety margin - high safety score
            safety_score = min(1.0, (clearance_safety / (self.preferred_clearance - self.min_obstacle_distance)))
        
        return True, min_clearance, corridor_width, safety_score

    def evaluate_path_quality(self, optimizer, path_array, pinn_result=None):
        """
        Calculate path quality with HARD clearance constraint and corridor awareness.
        Maintains all existing features: energy, stability, smoothness, etc.
        """
        # FIRST: Hard clearance check
        is_valid, min_clearance, corridor_width, safety_score = optimizer.validate_path_with_strict_clearance(path_array)
        
        if not is_valid:
            # Return worst possible score for invalid paths
            return {
                'energy': 1000.0,
                'stability': 0.0,
                'obstacle_cost': 1000.0,  # Maximum penalty
                'corridor_width': 0.0,
                'clearance': min_clearance,
                'total_cost': 1000.0
            }
        
        # Calculate standard objectives (EXISTING FEATURES)
        path_length = self.calculate_path_length(path_array)
        smoothness = self.calculate_path_smoothness(path_array)
        obstacle_cost = self.calculate_obstacle_cost(optimizer, path_array, min_clearance)
        
        # Get PINN predictions if available
        energy = 0.0
        stability = 0.5
        if pinn_result and pinn_result.get('success'):
            energy = float(pinn_result.get('energy', 0.0))
            stability = float(pinn_result.get('stability', 0.5))
        
        # ADAPTIVE PENALTY: Increase obstacle penalty if clearance is marginal
        adaptive_obstacle_penalty = obstacle_cost
        if min_clearance < 0.45:  # Slightly above 0.35m minimum
            clearance_ratio = min_clearance / 0.35
            adaptive_obstacle_penalty *= (2.0 - clearance_ratio)  # 2x penalty at exactly 0.35m
        
        # Corridor awareness: reward paths through wider corridors
        corridor_bonus = max(0.0, (corridor_width - 0.5) * 0.1)  # Bonus if corridor > 0.5m
        
        # Calculate total cost
        weights = optimizer.objective_weights  # [0.12, 0.08, 0.35, 0.08, 0.12, 0.15, 0.1]
        
        total_cost = (
            weights[0] * (energy / 100.0) +           # Energy (normalized)
            weights[1] * (1.0 - stability) +           # Stability (inverted)
            weights[2] * adaptive_obstacle_penalty +   # Obstacles (ADAPTIVE)
            weights[3] * smoothness +                  # Smoothness
            weights[4] * (path_length / 10.0) +        # Length (normalized)
            weights[5] * safety_score +                # Safety margin
            weights[6] * max(0.0, corridor_bonus)      # Corridor bonus
        )
        
        return {
            'energy': energy,
            'stability': stability,
            'obstacle_cost': adaptive_obstacle_penalty,
            'corridor_width': corridor_width,
            'clearance': min_clearance,
            'safety_score': safety_score,
            'total_cost': total_cost
        }

    def generate_initial_population_with_validation(self, start_pos, goal_pos, pop_size=6):
        """
        Generate initial population with validation.
        Only adds paths that meet hard clearance constraints.
        """
        population = []
        attempts = 0
        max_attempts = pop_size * 5  # Try up to 5x population size
        
        while len(population) < pop_size and attempts < max_attempts:
            attempts += 1
            
            # Generate candidate path
            candidate_path = self.generate_path_candidate(start_pos, goal_pos)
            
            # HARD VALIDATION: Check clearance before adding to population
            is_valid, min_clearance, corridor_width, _ = self.validate_path_with_strict_clearance(candidate_path)
            
            if is_valid:
                population.append(candidate_path)
                if self.debug_mode and len(population) % 2 == 0:
                    self.get_logger().debug(
                        f"✅ Valid path #{len(population)}: clearance={min_clearance:.3f}m, "
                        f"corridor={corridor_width:.2f}m"
                    )
            else:
                if self.debug_mode and attempts <= 3:
                    self.get_logger().debug(
                        f"❌ Rejected path: clearance={min_clearance:.3f}m (need {self.min_obstacle_distance}m)"
                    )
        
        if len(population) < pop_size:
            self.get_logger().warn(
                f"⚠️ Could only generate {len(population)}/{pop_size} valid paths after {attempts} attempts"
            )
        
        return population

    def publish_costmap(self):
        """Publish obstacle grid as OccupancyGrid message (bottom‑left origin)."""
        try:
            if self.obstacle_grid is None or self.obstacle_grid_origin is None:
                return

            costmap_msg = OccupancyGrid()
            costmap_msg.header.frame_id = 'odom'
            costmap_msg.header.stamp = self.get_clock().now().to_msg()

            costmap_msg.info.resolution = self.obstacle_grid_resolution
            costmap_msg.info.width = self.obstacle_grid_size
            costmap_msg.info.height = self.obstacle_grid_size

            # Origin IS bottom‑left corner
            costmap_msg.info.origin.position.x = float(self.obstacle_grid_origin[0])
            costmap_msg.info.origin.position.y = float(self.obstacle_grid_origin[1])
            costmap_msg.info.origin.position.z = 0.0
            costmap_msg.info.origin.orientation.w = 1.0

            # Convert grid (0.0–1.0) to occupancy (0–100)
            data = (self.obstacle_grid * 100).astype(np.int8).flatten().tolist()
            costmap_msg.data = data

            self.costmap_pub.publish(costmap_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing costmap: {e}")
    
    def _apply_distance_transform(self):
        """Apply simple distance transform to expand obstacle cost."""
        if self.obstacle_grid is None:
            return

        temp_grid = self.obstacle_grid.copy()
        kernel_size = 3

        for i in range(kernel_size, self.obstacle_grid_size - kernel_size):
            for j in range(kernel_size, self.obstacle_grid_size - kernel_size):
                if temp_grid[i, j] > 0.5:
                    for di in range(-2, 3):
                        for dj in range(-2, 3):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < self.obstacle_grid_size and 0 <= nj < self.obstacle_grid_size:
                                dist = math.hypot(di, dj)
                                decay = max(0.0, 1.0 - dist * 0.2)
                                self.obstacle_grid[ni, nj] = max(
                                    self.obstacle_grid[ni, nj],
                                    temp_grid[i, j] * decay
                                )
    def update_obstacle_cells(self, x, y, range_val):
        """Update obstacle cells: mark a small area around the hit point."""
        if self.obstacle_grid_origin is None or self.obstacle_grid is None:
            return

        gx, gy = self.world_to_grid(x, y)
        if gx is None or gy is None:
            return

        radius_cells = 1   # 3x3 area
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx = gx + dx
                ny = gy + dy
                if 0 <= nx < self.obstacle_grid_size and 0 <= ny < self.obstacle_grid_size:
                    self.obstacle_grid[nx, ny] = 0.9
    def update_obstacle_map(self):
        """Fast, frame‑correct obstacle map update."""
        if self.robot_pose is None or self.laser_scan is None:
            return

        robot_x = self.robot_pose.pose.pose.position.x
        robot_y = self.robot_pose.pose.pose.position.y
        robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)

        # Initialize grid origin if not set
        if self.obstacle_grid_origin is None:
            half_size = (self.obstacle_grid_size * self.obstacle_grid_resolution) / 2.0
            self.obstacle_grid_origin = (robot_x - half_size, robot_y - half_size)

        # Recenter grid if robot moves far
        self.recenter_obstacle_grid(robot_x, robot_y)

        # Decay old obstacles
        self.obstacle_grid *= 0.98

        # Process laser scan
        for i, r in enumerate(self.laser_scan.ranges):
            if r < self.laser_scan.range_min or r > self.laser_scan.range_max:
                continue
            angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
            # Laser point in world coordinates (direct calculation, no transform)
            lx = robot_x + r * math.cos(robot_yaw + angle)
            ly = robot_y + r * math.sin(robot_yaw + angle)

            # Convert to grid indices
            grid_x = int((lx - self.obstacle_grid_origin[0]) / self.obstacle_grid_resolution)
            grid_y = int((ly - self.obstacle_grid_origin[1]) / self.obstacle_grid_resolution)

            if 0 <= grid_x < self.obstacle_grid_size and 0 <= grid_y < self.obstacle_grid_size:
                self.obstacle_grid[grid_y, grid_x] = 1.0   # occupied
                self._mark_ray_free(robot_x, robot_y, lx, ly, grid_x, grid_y)

        # Clear robot's own footprint
        self.clear_robot_footprint(robot_x, robot_y, radius=self.robot_radius + 0.2)

        # Publish costmap for visualization
        self.publish_costmap()
    def _mark_ray_free(self, robot_x, robot_y, obs_x, obs_y, grid_obs_x, grid_obs_y):
        """Mark cells along the ray from robot to obstacle as free (0.0). Bottom‑left origin."""
        if self.obstacle_grid_origin is None:
            return

        rgx, rgy = self.world_to_grid(robot_x, robot_y)
        if rgx is None or rgy is None:
            return

        dx = abs(grid_obs_x - rgx)
        dy = abs(grid_obs_y - rgy)
        sx = 1 if rgx < grid_obs_x else -1
        sy = 1 if rgy < grid_obs_y else -1
        err = dx - dy

        x, y = rgx, rgy
        while True:
            if 0 <= x < self.obstacle_grid_size and 0 <= y < self.obstacle_grid_size:
                if self.obstacle_grid[y, x] < 0.3:
                    self.obstacle_grid[y, x] = 0.0
            if x == grid_obs_x and y == grid_obs_y:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    def clear_robot_footprint(self, robot_x, robot_y, radius=0.45):
        """Clear obstacle grid cells within the robot footprint (bottom‑left origin)."""
        if self.obstacle_grid is None or self.obstacle_grid_origin is None:
            return

        gx, gy = self.world_to_grid(robot_x, robot_y)
        if gx is None or gy is None:
            return

        radius_cells = int(radius / self.obstacle_grid_resolution) + 1

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.obstacle_grid_size and 0 <= ny < self.obstacle_grid_size:
                    dist = math.hypot(dx * self.obstacle_grid_resolution,
                                      dy * self.obstacle_grid_resolution)
                    if dist <= radius:
                        self.obstacle_grid[ny, nx] = 0.0
    def get_yaw_from_quaternion(self, quat):
        """Extract yaw angle from quaternion"""
        try:
            # quat is geometry_msgs/Quaternion with x, y, z, w
            x, y, z, w = quat.x, quat.y, quat.z, quat.w
            
            # Using yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
            sin_roll = 2.0 * (w * x + y * z)
            cos_roll = 1.0 - 2.0 * (x * x + y * y)
            
            sin_pitch = 2.0 * (w * y - z * x)
            
            sin_yaw = 2.0 * (w * z + x * y)
            cos_yaw = 1.0 - 2.0 * (y * y + z * z)
            
            yaw = math.atan2(sin_yaw, cos_yaw)
            return yaw
        except Exception as e:
            self.get_logger().debug(f"Error extracting yaw: {e}")
            return 0.0

    def _mark_line_bresenham(self, x0, y0, x1, y1, range_val):
        """Mark line from robot to obstacle using Bresenham algorithm"""
        grid_size = self.obstacle_grid_size
        grid_res = self.obstacle_grid_resolution
        
        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        free_space_confidence = 0.3  # Lower confidence for free space
        obstacle_confidence = 0.8    # Higher confidence near hit point
        
        max_steps = max(dx, dy)
        if max_steps == 0:
            return
        
        for step in range(max_steps):
            if 0 <= x < grid_size and 0 <= y < grid_size:
                # Cells closer to hit point get higher obstacle probability
                distance_ratio = step / max_steps
                confidence = free_space_confidence + (obstacle_confidence - free_space_confidence) * distance_ratio
                
                # Update with exponential smoothing
                self.obstacle_grid[y, x] = max(self.obstacle_grid[y, x], confidence)
            
            if err > 0:
                x += sx
                err -= dy
            else:
                y += sy
                err += dx
    def _inflate_obstacles(self, robot_radius=0.35):
        """Inflate obstacles by robot radius - CONSERVATIVE INFLATION"""
        grid_size = self.obstacle_grid_size
        grid_res = self.obstacle_grid_resolution
        
        # Calculate inflation radius in cells
        inflation_cells = int(math.ceil(robot_radius / grid_res))
        
        # Only inflate detected obstacles (threshold > 0.5)
        inflated = self.obstacle_grid.copy()
        
        for y in range(grid_size):
            for x in range(grid_size):
                if self.obstacle_grid[y, x] > 0.5:  # Only inflate significant obstacles
                    # Circular inflation
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        for dx in range(-inflation_cells, inflation_cells + 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= nx < grid_size and 0 <= ny < grid_size:
                                dist = math.hypot(dx, dy)
                                if dist <= inflation_cells:
                                    inflated[ny, nx] = max(inflated[ny, nx], 0.7)
        
        self.obstacle_grid = inflated

    def _get_clearance_laser_first(self, x, y, angle=None):
        """
        Direct laser ray cast for clearance.
        - If angle is given: return the range in that direction (closest beam).
        - If only (x,y): return the Euclidean distance to the nearest laser point
          within ±120° of the direction to the point.
        """
        if self.laser_scan is None or self.robot_pose is None:
            return None

        robot_x = self.robot_pose.pose.pose.position.x
        robot_y = self.robot_pose.pose.pose.position.y
        robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)

        if angle is not None:
            # Direction‑based: find closest laser beam angle
            rel_angle = angle - robot_yaw
            rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
            best_idx = None
            best_diff = float('inf')
            for i, scan_angle in enumerate(self.scan_angles):
                diff = abs(rel_angle - scan_angle)
                diff = min(diff, 2*math.pi - diff)
                if diff < best_diff:
                    best_diff = diff
                    best_idx = i
            if best_idx is not None and best_diff < math.radians(5):
                r = self.laser_scan.ranges[best_idx]
                if 0.1 < r < self.laser_scan.range_max:
                    return r
            return None

        # Point‑based clearance: find distance to nearest laser point
        dx = x - robot_x
        dy = y - robot_y
        dist_to_point = math.hypot(dx, dy)
        angle_to_point = math.atan2(dy, dx)
        rel_angle = angle_to_point - robot_yaw
        rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))

        # Since we have 360° LiDAR, we can search all beams, but we only care
        # about points that are roughly in the direction of the query point.
        # Use a ±120° window to cover front and sides.
        min_dist = float('inf')
        for i, r in enumerate(self.laser_scan.ranges):
            if r < self.laser_scan.range_min or r > self.laser_scan.range_max:
                continue
            scan_angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
            diff = abs(rel_angle - scan_angle)
            diff = min(diff, 2*math.pi - diff)
            if diff > math.radians(120):   # ignore points too far from direction
                continue
            # Compute world coordinates of this laser point
            laser_world_angle = robot_yaw + scan_angle
            lx = robot_x + r * math.cos(laser_world_angle)
            ly = robot_y + r * math.sin(laser_world_angle)
            d = math.hypot(x - lx, y - ly)
            if d < min_dist:
                min_dist = d

        return min_dist if min_dist != float('inf') else None
    def get_clearance_at_point(self, x, y, lookahead=2.0):
        """
        Fused clearance: tries direct laser ray cast first, then falls back to costmap.
        If costmap is nearly empty (<10 occupied cells), uses laser exclusively.
        Returns a non‑negative distance (minimum 0.01m).
        """
        if self.robot_pose is not None:
            robot_x = self.robot_pose.pose.pose.position.x
            robot_y = self.robot_pose.pose.pose.position.y
            dist_to_robot = math.hypot(x - robot_x, y - robot_y)
            if dist_to_robot < self.robot_radius + 0.1:
                return 2.0

        # Check if costmap has meaningful data
        costmap_empty = True
        if self.obstacle_grid is not None:
            occupied = np.sum(self.obstacle_grid > self.occupied_threshold)
            if occupied > 10:
                costmap_empty = False

        # 1) Direct laser ray cast (always try first)
        laser_clearance = self._get_clearance_laser_first(x, y)
        laser_valid = laser_clearance is not None

        # 2) Costmap clearance (only if costmap is not empty)
        costmap_clearance = 0.5   # default when no obstacle nearby
        if not costmap_empty and self.obstacle_grid is not None and self.obstacle_grid_origin is not None:
            gx, gy = self.world_to_grid(x, y)
            if gx is not None and gy is not None:
                if self.obstacle_grid[gx, gy] > self.occupied_threshold:
                    costmap_clearance = 0.0
                else:
                    min_dist = lookahead
                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            nx, ny = gx + dx, gy + dy
                            if 0 <= nx < self.obstacle_grid_size and 0 <= ny < self.obstacle_grid_size:
                                if self.obstacle_grid[nx, ny] > self.occupied_threshold:
                                    dist = math.hypot(dx * self.obstacle_grid_resolution,
                                                      dy * self.obstacle_grid_resolution)
                                    min_dist = min(min_dist, dist)
                    costmap_clearance = min_dist

        # 3) Decision: prefer laser if available, unless costmap is the only source
        if laser_valid:
            # If costmap is empty, trust laser completely
            if costmap_empty:
                result = max(0.01, laser_clearance)
                self.get_logger().debug(
                    f"📡 get_clearance_at_point({x:.2f},{y:.2f}): costmap empty, laser={result:.2f}m"
                )
            else:
                # Both available: use laser, but cap at costmap if laser is unrealistically small
                if laser_clearance < 0.1 and costmap_clearance > 0.3:
                    result = costmap_clearance
                    self.get_logger().debug(
                        f"📡 get_clearance_at_point({x:.2f},{y:.2f}): laser={laser_clearance:.2f}m (too low), using costmap={costmap_clearance:.2f}m"
                    )
                else:
                    result = max(0.01, laser_clearance)
                    self.get_logger().debug(
                        f"📡 get_clearance_at_point({x:.2f},{y:.2f}): laser={result:.2f}m (costmap={costmap_clearance:.2f}m)"
                    )
        else:
            # No laser: use costmap if available, otherwise lookahead
            if costmap_empty:
                result = lookahead
                self.get_logger().debug(
                    f"📡 get_clearance_at_point({x:.2f},{y:.2f}): no laser, costmap empty → lookahead={result:.2f}m"
                )
            else:
                result = max(0.01, costmap_clearance)
                self.get_logger().debug(
                    f"📡 get_clearance_at_point({x:.2f},{y:.2f}): costmap only={result:.2f}m"
                )
        return result
    def check_trajectory_collision(self, path_array):
        """Check if trajectory collides with obstacles"""
        if self.obstacle_grid is None or path_array is None:
            return False, None, 0.0

        collision_severity = 0.0
        collision_point = None

        for point in path_array:
            x, y = point
            cell_x, cell_y = self.world_to_grid(x, y)

            if cell_x is None or cell_y is None:
                continue

            # Check 3x3 area around point
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    nx, ny = cell_x + dx, cell_y + dy
                    if 0 <= nx < self.obstacle_grid_size and 0 <= ny < self.obstacle_grid_size:
                        if self.obstacle_grid[nx, ny] > 0.5:
                            if collision_point is None:
                                collision_point = point
                            collision_severity = max(collision_severity,
                                                      self.obstacle_grid[nx, ny])

        return collision_severity > 0.5, collision_point, collision_severity

    def get_dynamic_obstacle_cost(self, path_array):
        """
        Enhanced dynamic obstacle cost using predicted positions over time.
        Uses a fixed assumed speed (1.2 m/s) for time‑to‑waypoint estimation.
        """
        if self.obstacle_tracker is None:
            return 0.0

        total_cost = 0.0
        num_waypoints = len(path_array)
        if num_waypoints == 0:
            return 0.0

        # Use max_linear_vel if defined, otherwise default to 1.2
        max_vel = getattr(self, 'max_linear_vel', 1.2)
        assumed_speed = max_vel  # you can also use a fixed 0.8 for prediction

        # Compute cumulative distance and arrival times
        cumulative_dist = 0.0
        waypoint_times = [0.0]
        for i in range(1, num_waypoints):
            dx = path_array[i][0] - path_array[i-1][0]
            dy = path_array[i][1] - path_array[i-1][1]
            cumulative_dist += math.hypot(dx, dy)
            waypoint_times.append(cumulative_dist / assumed_speed)

        high_cost_waypoints = 0
        max_waypoint_cost = 0.0

        for idx, (x, y) in enumerate(path_array):
            t_arrival = waypoint_times[idx]
            if t_arrival > 2.0:  # only predict 2 seconds ahead
                continue

            for obs in self.obstacle_tracker.get_high_confidence_obstacles():
                pred_x, pred_y = obs.predict_position(t_arrival)
                dist = math.hypot(x - pred_x, y - pred_y)

                # Compute threat level (use robot_pose and goal_pose if available)
                if self.robot_pose is not None and self.goal_pose is not None:
                    # No robot speed in optimizer, use a default of 0.5 m/s for threat
                    robot_speed = 0.5
                    threat = self._compute_obstacle_threat(
                        obs,
                        self.robot_pose.pose.pose.position.x,
                        self.robot_pose.pose.pose.position.y,
                        self.goal_pose.pose.position.x,
                        self.goal_pose.pose.position.y,
                        robot_speed
                    )
                else:
                    threat = 0.5

                if dist < 0.8:
                    cost = (0.8 - dist) ** 2 * 100.0 * (1.0 + threat)
                    total_cost += cost
                    high_cost_waypoints += 1
                    if cost > max_waypoint_cost:
                        max_waypoint_cost = cost
                    # Log occasionally (throttled)
                    if self.optimization_count % 10 == 0 and idx % 3 == 0:
                        self.get_logger().warn(
                            f"🚨 DynObs cost: waypoint {idx} dist={dist:.2f}m threat={threat:.2f} cost={cost:.1f}"
                        )
                elif dist < 1.5:
                    cost = (1.5 - dist) * 20.0 * (0.5 + threat * 0.5)
                    total_cost += cost
                    if cost > max_waypoint_cost:
                        max_waypoint_cost = cost

        avg_cost = total_cost / max(num_waypoints, 1)
        if high_cost_waypoints > 0:
            self.get_logger().info(
                f"📊 Dynamic obstacle cost: total={total_cost:.2f}, avg={avg_cost:.2f}, "
                f"max_waypoint={max_waypoint_cost:.2f}, close_waypoints={high_cost_waypoints}/{num_waypoints}"
            )
        return avg_cost
    def _compute_obstacle_threat(self, obs, robot_x, robot_y, goal_x, goal_y, robot_speed):
        """Compute threat level (0-1) of a single obstacle."""
        # Current obstacle position and velocity
        obs_x, obs_y = obs.x, obs.y
        obs_vx, obs_vy = obs.vx, obs.vy

        # Distance from robot to obstacle
        dist_to_robot = math.hypot(obs_x - robot_x, obs_y - robot_y)
        # Speed of obstacle
        obs_speed = math.hypot(obs_vx, obs_vy)

        # Direction from robot to obstacle
        if dist_to_robot > 1e-6:
            dir_to_obs_x = (obs_x - robot_x) / dist_to_robot
            dir_to_obs_y = (obs_y - robot_y) / dist_to_robot
        else:
            dir_to_obs_x, dir_to_obs_y = 1.0, 0.0

        # Closing speed: positive if obstacle moving toward robot
        closing_speed = -(obs_vx * dir_to_obs_x + obs_vy * dir_to_obs_y)

        # Baseline threat from proximity (exponential, 1.0 at 0m, 0 at 2m)
        dist_threat = math.exp(-dist_to_robot / 0.5)   # decays quickly beyond 1m
        # Velocity threat (closing speed normalized)
        vel_threat = min(1.0, max(0.0, closing_speed / 1.5))

        # Directional threat: is obstacle between robot and goal?
        goal_dir_x = goal_x - robot_x
        goal_dir_y = goal_y - robot_y
        goal_dist = math.hypot(goal_dir_x, goal_dir_y)
        if goal_dist > 1e-6:
            goal_dir_x /= goal_dist
            goal_dir_y /= goal_dist
        # Cosine of angle between robot->obstacle and robot->goal
        cos_angle = max(0.0, dir_to_obs_x * goal_dir_x + dir_to_obs_y * goal_dir_y)
        dir_threat = cos_angle   # 1 if obstacle straight ahead, 0 if sideways

        # Combine threats
        threat = 0.4 * dist_threat + 0.3 * vel_threat + 0.3 * dir_threat
        threat = min(1.0, max(0.0, threat))

        # Log high threats (preserve existing logging behaviour)
        if threat > 0.5:
            self.get_logger().warn(
                f"⚠️ High threat obstacle: threat={threat:.2f}, dist={dist_to_robot:.2f}m, "
                f"obs_speed={obs_speed:.2f}m/s, closing={closing_speed:.2f}"
            )
        return threat
    def calculate_adaptive_safety_buffer(self, robot_speed: float = 0.5,
                                        uncertainty: float = 0.1) -> float:
        """
        Calculate adaptive safety buffer based on:
        - Robot speed (faster = larger buffer)
        - Sensor uncertainty (higher = larger buffer)
        - Obstacle dynamics (moving = larger buffer)
        """
        base_buffer = 0.35  # robot radius

        # Speed-based buffer (1m per m/s)
        speed_buffer = max(0.0, min(robot_speed * 1.0, 1.5))

        # Uncertainty buffer
        uncertainty_buffer = uncertainty * 0.5

        # Dynamic obstacle buffer
        dynamic_buffer = 0.0
        if self.obstacle_tracker.get_obstacles():
            avg_speed = np.mean([
                math.hypot(obs.vx, obs.vy)
                for obs in self.obstacle_tracker.get_high_confidence_obstacles()
            ]) if self.obstacle_tracker.get_high_confidence_obstacles() else 0.0
            dynamic_buffer = avg_speed * 0.5

        total_buffer = base_buffer + speed_buffer + uncertainty_buffer + dynamic_buffer
        return min(total_buffer, 2.5)  # Cap at 2.5m   
    def evaluate_all_objectives_with_timeout(self, path_array, obstacle_cost, uncertainty_cost,
                                             straight_line_length, use_pinn=False, timeout=0.5):
        """Evaluate all objectives for a path"""
        if path_array is None or len(path_array) < 2:
            return (float('inf'), obstacle_cost, uncertainty_cost, 0.0, 0.0, 0.0, 0.0)

        # Basic objectives
        path_length = self.calculate_path_length(path_array)
        curvature = self.calculate_curvature(path_array)
        smoothness = self.calculate_smoothness(path_array)

        # PINN energy/stability (0 if disabled)
        pinn_energy = 0.0
        pinn_stability = 1.0

        if use_pinn and self.pinn_service_available:
            try:
                pinn_result = self.call_pinn_service_optimized(path_array)
                if pinn_result and pinn_result.get('success', False):
                    pinn_energy = pinn_result.get('energy', 0.0)
                    pinn_stability = pinn_result.get('stability', 1.0)
                    self.last_pinn_success += 1
                self.last_pinn_calls += 1
            except Exception as e:
                self.get_logger().debug(f"PINN evaluation failed: {e}")

        # Return all objectives as tuple
        return (
            path_length,              # 0: path length
            curvature,                # 1: curvature penalty
            obstacle_cost,            # 2: obstacle avoidance
            smoothness,               # 3: path smoothness
            uncertainty_cost,         # 4: uncertainty penalty
            pinn_energy,              # 5: PINN energy
            pinn_stability            # 6: PINN stability
        )
    def _grid_half_size_m(self) -> float:
        """Half of the grid's side length in meters."""
        return (self.obstacle_grid_size * self.obstacle_grid_resolution) / 2.0

    def _ensure_grid_origin_initialized(self, robot_x: float, robot_y: float):
        """
        Ensure obstacle_grid_origin is ALWAYS bottom‑left corner in world coords.
        """
        if self.obstacle_grid_origin is None:
            half = self._grid_half_size_m()
            self.obstacle_grid_origin = (robot_x - half, robot_y - half)
            if self.debug_mode:
                self.get_logger().info(
                    f"✅ Initialized grid origin (bottom-left) to ({self.obstacle_grid_origin[0]:.2f}, {self.obstacle_grid_origin[1]:.2f})"
                )
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices (bottom‑left origin)."""
        if self.obstacle_grid is None or self.obstacle_grid_origin is None:
            return None, None

        grid_x = int((x - self.obstacle_grid_origin[0]) / self.obstacle_grid_resolution)
        grid_y = int((y - self.obstacle_grid_origin[1]) / self.obstacle_grid_resolution)

        if 0 <= grid_x < self.obstacle_grid_size and 0 <= grid_y < self.obstacle_grid_size:
            return grid_x, grid_y
        return None, None
    def get_perpendicular_obstacle_distances(self, x, y, heading_angle):
        """
        Given a position and heading direction, measure the distance to obstacles
        on the left and right perpendicular directions.
        
        Returns:
            dict with keys: left_obstacle_dist, right_obstacle_dist, corridor_width,
                            left_point, right_point, left_pos, right_pos
        """
        left_angle = heading_angle + math.pi / 2
        right_angle = heading_angle - math.pi / 2
        
        max_search = 2.0
        step = 0.05
        left_dist = max_search
        right_dist = max_search
        left_pt = (x + max_search * math.cos(left_angle), y + max_search * math.sin(left_angle))
        right_pt = (x + max_search * math.cos(right_angle), y + max_search * math.sin(right_angle))
        
        # Raycast left direction
        for distance in np.arange(step, max_search, step):
            test_x = x + distance * math.cos(left_angle)
            test_y = y + distance * math.sin(left_angle)
            # Use a higher threshold for free space detection (0.3)
            if self.get_clearance_at_point(test_x, test_y) < 0.3:
                left_dist = distance - step
                left_pt = (test_x, test_y)
                break
        
        # Raycast right direction
        for distance in np.arange(step, max_search, step):
            test_x = x + distance * math.cos(right_angle)
            test_y = y + distance * math.sin(right_angle)
            if self.get_clearance_at_point(test_x, test_y) < 0.3:
                right_dist = distance - step
                right_pt = (test_x, test_y)
                break
        
        # Ensure positive distances
        left_dist = max(0.0, left_dist)
        right_dist = max(0.0, right_dist)
        
        return {
            'left_obstacle_dist': left_dist,
            'right_obstacle_dist': right_dist,
            'corridor_width': left_dist + right_dist,
            'left_point': left_pt,
            'right_point': right_pt,
            'left_pos': left_pt,
            'right_pos': right_pt
        }


    def enforce_path_clearance(self, path, start_x, start_y, goal_x, goal_y,
                               min_clearance=0.35, max_iterations=3):
        if not path or len(path) < 2:
            return path

        CLEARANCE_THRESHOLD = min_clearance
        MAX_SHIFT_DISTANCE = 1.0
        corrected_path = list(path)

        for _ in range(max_iterations):
            modified = False
            for i in range(1, len(corrected_path) - 1):
                x, y = corrected_path[i]
                min_dist = self.get_min_distance_to_obstacle(x, y)

                if min_dist < CLEARANCE_THRESHOLD:
                    best_shift = None
                    best_distance = min_dist
                    for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
                        for distance in np.linspace(0.05, MAX_SHIFT_DISTANCE, 6):
                            shift_x = x + distance * np.cos(angle)
                            shift_y = y + distance * np.sin(angle)
                            new_dist = self.get_min_distance_to_obstacle(shift_x, shift_y)
                            if new_dist > best_distance:
                                best_distance = new_dist
                                best_shift = (shift_x, shift_y)
                    if best_shift and best_distance >= CLEARANCE_THRESHOLD:
                        corrected_path[i] = best_shift
                        modified = True
                    else:
                        prev = corrected_path[i-1]
                        nxt = corrected_path[i+1]
                        corrected_path[i] = (0.3*prev[0] + 0.7*nxt[0], 0.3*prev[1] + 0.7*nxt[1])
                        modified = True
            if not modified:
                break
        return corrected_path

    def mutate_with_free_space_and_corridor(self, path, start_x, start_y,
                                            goal_x, goal_y, min_clearance=0.35):
        if len(path) < 3:
            return path

        mutated_path = list(path)
        mut_idx = random.randint(1, len(path) - 2)

        prev_x, prev_y = path[mut_idx - 1]
        next_x, next_y = path[mut_idx + 1]

        dir_x = next_x - prev_x
        dir_y = next_y - prev_y
        dir_len = math.hypot(dir_x, dir_y)
        if dir_len < 1e-6:
            return path

        dir_x /= dir_len
        dir_y /= dir_len
        perp_x = -dir_y
        perp_y = dir_x

        best_candidate = None
        best_clearance = min_clearance

        for along_factor in np.linspace(-0.3, 0.3, 5):
            for perp_factor in np.linspace(-0.4, 0.4, 7):
                candidate_x = (prev_x + 0.5 * dir_x * dir_len +
                               along_factor * dir_x * dir_len +
                               perp_factor * 0.5 * perp_x)
                candidate_y = (prev_y + 0.5 * dir_y * dir_len +
                               along_factor * dir_y * dir_len +
                               perp_factor * 0.5 * perp_y)
                clearance = self.get_min_distance_to_obstacle(candidate_x, candidate_y)
                if clearance >= min_clearance and clearance > best_clearance:
                    best_clearance = clearance
                    best_candidate = (candidate_x, candidate_y)

        if best_candidate:
            mutated_path[mut_idx] = best_candidate
        else:
            mutated_path[mut_idx] = (0.4*prev_x + 0.6*next_x, 0.4*prev_y + 0.6*next_y)

        return mutated_path

    def get_min_distance_to_obstacle(self, x, y):
        """Minimum distance to obstacle: laser points first, then costmap."""
        min_dist = float('inf')

        # Laser scan
        if self.laser_scan is not None and self.robot_pose is not None:
            robot_x = self.robot_pose.pose.pose.position.x
            robot_y = self.robot_pose.pose.pose.position.y
            robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
            for r, angle in zip(self.laser_scan.ranges, self.scan_angles):
                if 0.1 < r < self.laser_scan.range_max:
                    laser_angle_world = robot_yaw + angle
                    lx = robot_x + r * math.cos(laser_angle_world)
                    ly = robot_y + r * math.sin(laser_angle_world)
                    dist = math.hypot(x - lx, y - ly)
                    if dist < min_dist:
                        min_dist = dist

        # Costmap
        if self.obstacle_grid is not None and self.obstacle_grid_origin is not None:
            gx, gy = self.world_to_grid(x, y)
            if gx is not None and gy is not None:
                search_radius = int(2.0 / self.obstacle_grid_resolution)
                for dx in range(-search_radius, search_radius + 1):
                    for dy in range(-search_radius, search_radius + 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.obstacle_grid_size and 0 <= ny < self.obstacle_grid_size:
                            if self.obstacle_grid[nx, ny] > self.occupied_threshold:
                                obs_x = (self.obstacle_grid_origin[0] +
                                         (nx - self.obstacle_grid_size/2) * self.obstacle_grid_resolution)
                                obs_y = (self.obstacle_grid_origin[1] +
                                         (ny - self.obstacle_grid_size/2) * self.obstacle_grid_resolution)
                                dist = math.hypot(x - obs_x, y - obs_y)
                                if dist < min_dist:
                                    min_dist = dist

        result = min_dist if min_dist != float('inf') else 5.0
        self.get_logger().debug(f"📏 get_min_distance_to_obstacle({x:.2f},{y:.2f}) = {result:.2f}m")
        return result

    def calculate_corridor_width(self, path_array):
        """
        Calculate the corridor width available for the path at each waypoint.
        This measures the free space around each point on the path.
        """
        if not hasattr(self, 'obstacle_grid') or self.obstacle_grid is None or len(path_array) < 2:
            return 0.5  # Default corridor width if no grid
        
        corridor_widths = []
        
        for x, y in path_array:
            # Convert world coordinates to grid coordinates
            cell_x, cell_y = self.world_to_grid(x, y)
            
            if cell_x is None or cell_y is None:
                corridor_widths.append(0.5)
                continue
            
            # Find maximum clearance in all directions from this point
            max_clearance = 10.0  # Start with large value
            
            # Check clearance in 8 directions (and more for accuracy)
            for angle in np.linspace(0, 2*np.pi, 16):
                # Trace ray from point outward until we hit obstacle
                ray_dist = 0.0
                step_size = self.obstacle_grid_resolution
                
                for step in range(1, 50):  # Max 50 steps = 5m range
                    check_x = cell_x + int((step * step_size * np.cos(angle)) / self.obstacle_grid_resolution)
                    check_y = cell_y + int((step * step_size * np.sin(angle)) / self.obstacle_grid_resolution)
                    
                    # Check bounds
                    if not (0 <= check_x < self.obstacle_grid_size and 0 <= check_y < self.obstacle_grid_size):
                        ray_dist = step * step_size
                        break
                    
                    # Check if obstacle
                    if self.obstacle_grid[check_x, check_y] > self.occupied_threshold:
                        ray_dist = (step - 1) * step_size
                        break
                
                # Take minimum of all rays as the corridor width at this point
                max_clearance = min(max_clearance, ray_dist) if ray_dist > 0 else max_clearance
            
            corridor_widths.append(max_clearance)
        
        # Return average corridor width
        avg_corridor = np.mean(corridor_widths) if corridor_widths else 0.5
        return max(0.1, avg_corridor)  # Ensure minimum value

    def calculate_obstacle_to_obstacle_distance(self, waypoint_x, waypoint_y, left_obstacle_pos, right_obstacle_pos):
        """
        Calculate the direct distance between left and right obstacles using Pythagorean theorem.
        
        If we have:
        - Waypoint C at (waypoint_x, waypoint_y)
        - Left obstacle B at (left_obstacle_pos)
        - Right obstacle A at (right_obstacle_pos)
        
        Using Pythagorean theorem: a² + b² = c²
        Where:
        - a = perpendicular distance from waypoint to line connecting obstacles
        - b = horizontal/vertical distance along corridor
        - c = direct distance between obstacles
        
        Returns:
        {
            'direct_distance': float,         # Direct A-B distance
            'waypoint_to_left': float,        # C-B distance
            'waypoint_to_right': float,       # C-A distance
            'pythagorean_valid': bool         # Check: (C-B)² + (C-A)² should relate to A-B
        }
        """
        if left_obstacle_pos is None or right_obstacle_pos is None:
            return {
                'direct_distance': float('inf'),
                'waypoint_to_left': float('inf'),
                'waypoint_to_right': float('inf'),
                'pythagorean_valid': False
            }
        
        left_x, left_y = left_obstacle_pos
        right_x, right_y = right_obstacle_pos
        
        # Distance from waypoint C to left obstacle B
        dist_c_to_b = math.hypot(waypoint_x - left_x, waypoint_y - left_y)
        
        # Distance from waypoint C to right obstacle A
        dist_c_to_a = math.hypot(waypoint_x - right_x, waypoint_y - right_y)
        
        # Direct distance from left obstacle B to right obstacle A
        dist_a_to_b = math.hypot(right_x - left_x, right_y - left_y)
        
        # Verify Pythagorean relationship (for triangle validation)
        # In a right triangle: a² + b² = c²
        # Check if waypoint forms a right angle
        sum_of_squares = dist_c_to_b**2 + dist_c_to_a**2
        direct_squared = dist_a_to_b**2
        
        # Allow small tolerance for floating point errors
        pythagorean_valid = abs(sum_of_squares - direct_squared) < 0.1
        
        return {
            'direct_distance': dist_a_to_b,
            'waypoint_to_left': dist_c_to_b,
            'waypoint_to_right': dist_c_to_a,
            'pythagorean_valid': pythagorean_valid,
            'sum_of_squares': sum_of_squares,
            'direct_squared': direct_squared
        }
    def validate_waypoint_corridor_safety(self, waypoint_x, waypoint_y, path_angle, min_corridor_width=0.6):
        """
        Check if waypoint has sufficient corridor width.
        Falls back to direct clearance if corridor width is suspicious.
        """
        # First check direct obstacle clearance (most reliable)
        direct_clearance = self.get_clearance_at_point(waypoint_x, waypoint_y)
        if direct_clearance >= 0.35:
            # Robot can safely pass even if corridor width is not measurable
            return {
                'is_safe': True,
                'reason': f'Direct clearance {direct_clearance:.2f}m sufficient',
                'corridor_width': direct_clearance * 2,  # approximate
                'obstacle_distances': None,
                'triangle_info': None
            }
        
        # Get perpendicular obstacles
        perp_obstacles = self.get_perpendicular_obstacle_distances(
            waypoint_x, waypoint_y, path_angle
        )
        
        corridor_width = perp_obstacles['corridor_width']
        left_pos = perp_obstacles['left_pos']
        right_pos = perp_obstacles['right_pos']
        
        # If corridor width is very small but direct clearance is okay, treat as safe
        if corridor_width < min_corridor_width and direct_clearance >= 0.35:
            return {
                'is_safe': True,
                'reason': f'Direct clearance {direct_clearance:.2f}m OK, corridor={corridor_width:.2f}m',
                'corridor_width': corridor_width,
                'obstacle_distances': perp_obstacles,
                'triangle_info': None
            }
        
        # Normal corridor width check
        if corridor_width < min_corridor_width:
            return {
                'is_safe': False,
                'reason': f'Corridor too narrow: {corridor_width:.2f}m < {min_corridor_width:.2f}m',
                'corridor_width': corridor_width,
                'obstacle_distances': perp_obstacles,
                'triangle_info': None
            }
        
        # Optional: Pythagorean distance check (kept for completeness)
        triangle_info = self.calculate_obstacle_to_obstacle_distance(
            waypoint_x, waypoint_y, left_pos, right_pos
        )
        if triangle_info['direct_distance'] < min_corridor_width:
            return {
                'is_safe': False,
                'reason': f'Obstacle gap too small: {triangle_info["direct_distance"]:.2f}m',
                'corridor_width': corridor_width,
                'obstacle_distances': perp_obstacles,
                'triangle_info': triangle_info
            }
        
        return {
            'is_safe': True,
            'reason': f'Safe corridor: {corridor_width:.2f}m wide',
            'corridor_width': corridor_width,
            'obstacle_distances': perp_obstacles,
            'triangle_info': triangle_info
        }
    def validate_path_with_corridor_analysis(self, path, min_corridor_width=0.6):
        """
        Validate entire path using corridor width analysis.
        Returns {'path_safe': True} always (so it never blocks publishing),
        but logs warnings for unsafe waypoints.
        """
        # Check if obstacle grid has meaningful data
        if self.obstacle_grid is not None:
            occupied_cells = np.sum(self.obstacle_grid > self.occupied_threshold)
            if occupied_cells < 10:
                if self.debug_mode:
                    self.get_logger().debug("Obstacle grid nearly empty – skipping corridor validation")
                return {'path_safe': True, 'waypoint_results': []}
        
        validation_results = []
        any_unsafe = False
        
        for idx, waypoint in enumerate(path):
            if idx == 0:
                continue  # Skip first waypoint
            
            # Calculate path angle
            prev_x, prev_y = path[idx - 1]
            curr_x, curr_y = waypoint
            path_angle = math.atan2(curr_y - prev_y, curr_x - prev_x)
            
            safety = self.validate_waypoint_corridor_safety(
                curr_x, curr_y, path_angle, min_corridor_width
            )
            
            validation_results.append({
                'waypoint_idx': idx,
                'waypoint': waypoint,
                'safety_check': safety
            })
            
            if not safety['is_safe']:
                any_unsafe = True
                if self.debug_mode:
                    self.get_logger().warn(
                        f"🚨 Waypoint {idx} UNSAFE (corridor width): {safety['reason']}"
                    )
        
        # Always return path_safe = True – we only warn, never block
        return {
            'path_safe': True,   # CHANGED: always True so publishing continues
            'waypoint_results': validation_results
        }
        
    def calculate_path_length(self, path_array):
        """Calculate total path length"""
        if len(path_array) < 2:
            return 0.0

        length = 0.0
        for i in range(len(path_array) - 1):
            p1 = path_array[i]
            p2 = path_array[i + 1]
            length += np.hypot(p2[0] - p1[0], p2[1] - p1[1])
        return length
    
    def calculate_curvature(self, path_array):
        """Calculate path curvature (sum of angle changes)"""
        if len(path_array) < 3:
            return 0.0

        total_curvature = 0.0
        for i in range(1, len(path_array) - 1):
            p0 = path_array[i - 1]
            p1 = path_array[i]
            p2 = path_array[i + 1]

            # Vectors
            v1 = np.array([p1[0] - p0[0], p1[1] - p0[1]])
            v2 = np.array([p2[0] - p1[0], p2[1] - p1[1]])

            # Angle between vectors
            len1 = np.linalg.norm(v1)
            len2 = np.linalg.norm(v2)

            if len1 > 0.01 and len2 > 0.01:
                cos_angle = np.clip(np.dot(v1, v2) / (len1 * len2), -1.0, 1.0)
                angle = np.arccos(cos_angle)
                total_curvature += angle

        return total_curvature
    def calculate_smoothness(self, path_array):
        """Calculate path smoothness (velocity profile variance)"""
        if len(path_array) < 3:
            return 0.0

        # Calculate distances between consecutive waypoints
        distances = []
        for i in range(len(path_array) - 1):
            p1 = path_array[i]
            p2 = path_array[i + 1]
            dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
            distances.append(dist)

        if len(distances) < 2:
            return 0.0

        # Variance in distances = roughness
        return np.var(distances)
    def validate_path_curvature_for_turn(self, path, required_angle_deg):
        """Validate that a path has adequate curvature for a required turn.

        Args:
            path: List of (x, y) tuples representing the path waypoints.
            required_angle_deg: The required turning angle in degrees.

        Returns:
            Tuple of (is_valid: bool, actual_curvature: float). is_valid is False
            when the path is too straight for the required turn.
        """
        path_array = np.array([[p[0], p[1]] for p in path])
        actual_curvature = self.calculate_curvature(path_array)
        min_curvature = max(
            MIN_TURN_CURVATURE_THRESHOLD,
            required_angle_deg / TURN_ANGLE_CURVATURE_SCALE
        )
        if actual_curvature < min_curvature:
            return False, actual_curvature
        return True, actual_curvature

    def get_min_clearance(self, path_array):
        """Get min clearance"""
        min_clearance = float('inf')
        
        for point in path_array:
            clearance = self.get_clearance_at_point(point[0], point[1])
            if clearance < min_clearance:
                min_clearance = clearance
        
        return min_clearance if min_clearance != float('inf') else self.min_obstacle_distance * 2
    
    def goal_callback(self, msg: PoseStamped):
        """Goal callback - TRACK TIME, but avoid churning if already near goal with safe path."""
        self.goal_pose = msg
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        self.goal_received_time = time.time()
        self.last_path_position = None

        if self.debug_mode:
            self.get_logger().info(f"🎯 New goal: ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f})")

        # If we are already close to the goal and the current remaining path is safe, keep it
        if self.robot_pose is not None and hasattr(self, 'last_optimized_path') and self.last_optimized_path:
            robot_x = self.robot_pose.pose.pose.position.x
            robot_y = self.robot_pose.pose.pose.position.y
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
            if dist_to_goal <= 1.5 and self.is_remaining_path_safe(self.last_optimized_path, robot_x, robot_y, min_clearance=0.45):
                if self.debug_mode:
                    self.get_logger().info("🔒 Keeping current path on goal update (robot already near goal with safe path)")
                # Still reset stuck tracking and trigger optimization if needed
                self.stuck_positions.clear()
                self.stuck_count = 0
                self.consecutive_stuck_checks = 0
                self.last_stuck_time = 0.0
                self.obstacle_nearby_when_stuck = False
                self.escape_attempts.clear()
                self.progress_toward_goal.clear()
                self.position_history.clear()
                self.trigger_optimization()
                return

        # Publish a quick‑response path immediately (original behavior)
        if self.robot_pose is not None:
            current_x = self.robot_pose.pose.pose.position.x
            current_y = self.robot_pose.pose.pose.position.y
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y

            straight_safe = self.is_straight_path_clear(current_x, current_y, goal_x, goal_y)

            if straight_safe:
                quick_path = self.generate_quick_response_path(current_x, current_y, goal_x, goal_y)
                self.get_logger().info("⚡ Straight quick-response path published")
            else:
                quick_path = self.generate_curved_path_for_turning(current_x, current_y, goal_x, goal_y)
                self.get_logger().info("⚡ Curved quick-response path published (obstacle ahead)")

            self.publish_path(quick_path)
            self.last_optimized_path = quick_path

        # Reset stuck tracking
        self.stuck_positions.clear()
        self.stuck_count = 0
        self.consecutive_stuck_checks = 0
        self.last_stuck_time = 0.0
        self.obstacle_nearby_when_stuck = False
        self.escape_attempts.clear()
        self.progress_toward_goal.clear()
        self.position_history.clear()

        # Trigger optimization
        self.trigger_optimization()
    
    def _interpolate_path_to_count(self, path_points, target_count):
        """Interpolate to exactly target_count waypoints using arc‑length."""
        if len(path_points) < 2 or target_count <= 1:
            return path_points
        dists = [0.0]
        for i in range(1, len(path_points)):
            seg = math.hypot(path_points[i][0]-path_points[i-1][0],
                             path_points[i][1]-path_points[i-1][1])
            dists.append(dists[-1] + seg)
        total = dists[-1]
        if total < 1e-6:
            return [path_points[0]] * target_count
        result = []
        j = 0
        for k in range(target_count):
            d = k / (target_count - 1) * total
            while j < len(dists)-1 and dists[j+1] < d:
                j += 1
            if j >= len(path_points)-1:
                result.append(path_points[-1])
            else:
                seg_len = dists[j+1] - dists[j]
                t = (d - dists[j]) / seg_len if seg_len > 1e-6 else 0.0
                x = path_points[j][0] + t*(path_points[j+1][0]-path_points[j][0])
                y = path_points[j][1] + t*(path_points[j+1][1]-path_points[j][1])
                result.append((x, y))
        return result
    def _generate_goal_directed_random_path(self, start_x, start_y, goal_x, goal_y, num_waypoints):
        """
        Generate a random path where each waypoint is placed in free space and progresses toward goal.
        """
        path = [(start_x, start_y)]
        for i in range(1, num_waypoints - 1):
            t = i / (num_waypoints - 1)
            base_x = start_x + t * (goal_x - start_x)
            base_y = start_y + t * (goal_y - start_y)

            # Direction of the straight line at this point
            prev_x, prev_y = path[-1]
            direction = math.atan2(base_y - prev_y, base_x - prev_x)

            best_x, best_y = base_x, base_y
            best_clearance = self.get_clearance_at_point(base_x, base_y)

            for _ in range(10):  # 10 random attempts
                # Perpendicular offset (maintain corridor) + small along-track offset
                perp = random.uniform(-0.5, 0.5)
                along = random.uniform(-0.3, 0.3)
                off_x = along * math.cos(direction) - perp * math.sin(direction)
                off_y = along * math.sin(direction) + perp * math.cos(direction)
                cand_x = base_x + off_x
                cand_y = base_y + off_y

                # Progress constraint: distance to goal should not increase
                if math.hypot(goal_x - cand_x, goal_y - cand_y) > math.hypot(goal_x - base_x, goal_y - base_y) + 0.1:
                    continue

                clearance = self.get_clearance_at_point(cand_x, cand_y)
                if clearance > best_clearance:
                    best_clearance = clearance
                    best_x, best_y = cand_x, cand_y

            path.append((best_x, best_y))
        path.append((goal_x, goal_y))
        return path

    def smooth_path_for_controller(self, path_points, num_points=30):
        """Interpolate path to num_points using arc‑length parameterisation."""
        if len(path_points) < 2:
            return path_points
        # cumulative arc lengths
        dists = [0.0]
        for i in range(1, len(path_points)):
            dx = path_points[i][0] - path_points[i-1][0]
            dy = path_points[i][1] - path_points[i-1][1]
            dists.append(dists[-1] + math.hypot(dx, dy))
        total = dists[-1]
        if total < 1e-6:
            return [path_points[0]] * num_points
        t = np.array(dists) / total
        xs = np.interp(np.linspace(0, 1, num_points), t, [p[0] for p in path_points])
        ys = np.interp(np.linspace(0, 1, num_points), t, [p[1] for p in path_points])
        return list(zip(xs, ys))
    def validate_path_safety(self, path, strict=True):
        """
        IMPROVED: Multi-stage validation that progressively loosens constraints.
        Always returns True/False - let publish_path handle fallbacks.
        """
        if not path or len(path) < 2:
            return False
        
        # Stage 1: Strict validation (0.35m clearance)
        min_clearance_strict = self._get_minimum_clearance_on_path(path)
        if min_clearance_strict >= 0.35:
            if self.debug_mode:
                self.get_logger().info(
                    f"✅ Path SAFE (strict): min_clearance={min_clearance_strict:.2f}m"
                )
            return True
        
        if strict:
            return False
        
        # Stage 2: Moderate validation (0.25m clearance)
        if min_clearance_strict >= 0.25:
            if self.debug_mode:
                self.get_logger().info(
                    f"⚠️ Path ACCEPTABLE (moderate): min_clearance={min_clearance_strict:.2f}m"
                )
            return True
        
        # Stage 3: Permissive validation (0.20m clearance)
        if min_clearance_strict >= 0.20:
            if self.debug_mode:
                self.get_logger().warn(
                    f"⚠️ Path RISKY (permissive): min_clearance={min_clearance_strict:.2f}m - "
                    f"controller will handle"
                )
            return True
        
        if self.debug_mode:
            self.get_logger().warn(
                f"🚨 Path UNSAFE: min_clearance={min_clearance_strict:.2f}m < 0.20m"
            )
        return False
    def _estimate_corridor_width(self, p1, p2, p3):
        """Estimate corridor width at middle point using perpendicular distances."""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        
        # Vector from p1 to p3
        dx = x3 - x1
        dy = y3 - y1
        dist_13 = math.hypot(dx, dy)
        
        if dist_13 < 0.01:  # Points too close
            return self.get_clearance_at_point(x2, y2)
        
        # Perpendicular distance from p2 to line p1-p3
        num = abs((y3 - y1) * x2 - (x3 - x1) * y2 + x3 * y1 - y3 * x1)
        perp_dist = num / dist_13
        
        # Estimate corridor width (simplified: min clearance on both sides)
        left_clearance = self.get_clearance_at_point(
            x2 - 0.25 * dy / dist_13,
            y2 + 0.25 * dx / dist_13
        )
        right_clearance = self.get_clearance_at_point(
            x2 + 0.25 * dy / dist_13,
            y2 - 0.25 * dx / dist_13
        )
        
        return min(left_clearance, right_clearance)
    def get_distance_to_nearest_obstacle(self, robot_x, robot_y):
        """
        Calculate the distance from a point to the nearest obstacle in the grid.
        
        Args:
            robot_x: X coordinate in world frame
            robot_y: Y coordinate in world frame
        
        Returns:
            Distance to nearest obstacle in meters (inf if no obstacle within range)
        """
        if not hasattr(self, 'obstacle_grid') or self.obstacle_grid is None:
            return float('inf')
        
        if not hasattr(self, 'obstacle_grid_origin') or not hasattr(self, 'obstacle_grid_resolution'):
            return float('inf')
        
        try:
            # Convert world coordinates to grid indices
            origin_x, origin_y = self.obstacle_grid_origin
            resolution = self.obstacle_grid_resolution
            
            grid_x = int((robot_x - origin_x) / resolution + self.obstacle_grid_size / 2)
            grid_y = int((robot_y - origin_y) / resolution + self.obstacle_grid_size / 2)
            
            # Get grid dimensions
            grid_height, grid_width = self.obstacle_grid.shape
            
            # Check if point is within grid bounds
            if grid_x < 0 or grid_x >= grid_width or grid_y < 0 or grid_y >= grid_height:
                return float('inf')
            
            # Search for nearest obstacle using expanding square search
            search_radius = int(5.0 / resolution)  # Search up to 5 meters
            min_distance = float('inf')
            
            for dx in range(-search_radius, search_radius + 1):
                for dy in range(-search_radius, search_radius + 1):
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    
                    # Bounds check
                    if check_x < 0 or check_x >= grid_width or check_y < 0 or check_y >= grid_height:
                        continue
                    
                    # If obstacle found, calculate distance
                    if self.obstacle_grid[check_y, check_x] > 0.5:  # Occupied cell threshold
                        # Calculate world distance
                        obs_world_x = origin_x + (check_x - self.obstacle_grid_size / 2) * resolution
                        obs_world_y = origin_y + (check_y - self.obstacle_grid_size / 2) * resolution
                        
                        dist = math.hypot(robot_x - obs_world_x, robot_y - obs_world_y)
                        min_distance = min(min_distance, dist)
            
            return min_distance
        
        except Exception as e:
            self.get_logger().warn(f"Error calculating obstacle distance: {e}")
            return float('inf')
    def relax_unsafe_path(self, path_array, failed_idx):
        """
        When a path is rejected, try to relax it by bypassing the problematic waypoint.
        Create a straighter path around obstacles.
        """
        if failed_idx is None or failed_idx >= len(path_array):
            return path_array
        
        try:
            relaxed_path = []
            num_waypoints = len(path_array)
            
            for i in range(num_waypoints):
                if i == failed_idx:
                    # Skip problematic waypoint, interpolate between neighbors
                    if failed_idx > 0 and failed_idx < num_waypoints - 1:
                        prev_wp = path_array[failed_idx - 1]
                        next_wp = path_array[failed_idx + 1]
                        
                        # Create shifted waypoint away from detected obstacle
                        mid_x = (prev_wp[0] + next_wp[0]) / 2.0
                        mid_y = (prev_wp[1] + next_wp[1]) / 2.0
                        
                        # Shift perpendicular to line
                        dx = next_wp[0] - prev_wp[0]
                        dy = next_wp[1] - prev_wp[1]
                        length = np.sqrt(dx**2 + dy**2)
                        
                        if length > 0.01:
                            # Perpendicular offset
                            perp_x = -dy / length * 0.3
                            perp_y = dx / length * 0.3
                            
                            shifted_x = mid_x + perp_x
                            shifted_y = mid_y + perp_y
                            
                            # Verify shifted point has clearance
                            if self.get_distance_to_nearest_obstacle(shifted_x, shifted_y) > 0.35:
                                relaxed_path.append([shifted_x, shifted_y])
                            else:
                                relaxed_path.append([mid_x, mid_y])
                        else:
                            relaxed_path.append([mid_x, mid_y])
                else:
                    relaxed_path.append(path_array[i])
            
            return relaxed_path
        except Exception as e:
            self.get_logger().warn(f"Path relaxation failed: {e}")
            return path_array
    def optimize_path_for_goal(self, goal_x, goal_y):
        """
        Optimize path with fallback and relaxation strategies.
        """
        start_time = time.time()
        best_path = None
        best_cost = float('inf')
        unsafe_count = 0
        
        try:
            # Quick-response path first
            quick_path = self.create_quick_response_path(goal_x, goal_y)
            is_safe, failed_idx = self.validate_path_safety(quick_path)
            
            if is_safe:
                self.publish_path(quick_path)
                self.last_published_path = quick_path
                self.get_logger().info("⚡ Quick-response path published; launching optimization")
            else:
                # Try to relax the path
                relaxed_path = self.relax_unsafe_path(quick_path, failed_idx)
                is_safe, failed_idx = self.validate_path_safety(relaxed_path)
                
                if is_safe:
                    self.publish_path(relaxed_path)
                    self.last_published_path = relaxed_path
                    self.get_logger().info("✅ Relaxed path published (obstacle bypass)")
                else:
                    self.get_logger().warn("⚠️ Quick path unsafe and relaxation failed, keeping previous")
            
            # Generate and test candidate paths
            for attempt in range(self.num_candidates):
                candidate = self.generate_candidate_path(goal_x, goal_y, attempt)
                
                # Validate safety first (cheap check)
                is_safe, failed_idx = self.validate_path_safety(candidate)
                
                if not is_safe:
                    # Try relaxation before giving up
                    candidate = self.relax_unsafe_path(candidate, failed_idx)
                    is_safe, failed_idx = self.validate_path_safety(candidate)
                    
                    if not is_safe:
                        unsafe_count += 1
                        continue
                
                # Calculate cost
                cost = self.calculate_path_cost(candidate)
                
                if cost < best_cost:
                    best_cost = cost
                    best_path = candidate
            
            # Publish best path found
            if best_path is not None:
                self.publish_path(best_path)
                self.last_published_path = best_path
            elif self.last_published_path is not None:
                self.get_logger().warn("⚠️ No safe paths found, keeping previous path")
            else:
                self.get_logger().error("❌ No path available!")
            
            elapsed = time.time() - start_time
            self.get_logger().info(
                f"✅ Optimization in {elapsed:.2f}s "
                f"({unsafe_count} unsafe paths, best_cost={best_cost:.0f})"
            )
            
        except Exception as e:
            self.get_logger().error(f"Optimization error: {e}")
            import traceback
            traceback.print_exc()
    def calculate_path_cost(self, path_array, optimizer=None):
        """
        Calculate comprehensive path cost.
        NOW INCLUDES: corridor_width (fixes KeyError)
        MAINTAINS: all existing features
        """
        if len(path_array) < 2:
            return float('inf')
        
        # Calculate all metrics
        path_length = self.calculate_path_length(path_array)
        smoothness = self.calculate_smoothness(path_array)
        
        # NEW: Calculate corridor width (was missing!)
        corridor_width = 0.5
        if optimizer:
            corridor_width = optimizer.calculate_corridor_width(path_array)
        
        # Get obstacle cost with minimum clearance
        min_clearance = self.get_min_clearance(path_array)
        obstacle_cost = self.calculate_obstacle_cost_adaptive(min_clearance, optimizer)
        
        # Combine all costs
        weights = [0.12, 0.08, 0.35, 0.08, 0.12, 0.15, 0.1]
        total_cost = (
            weights[0] * (path_length / 10.0) +
            weights[1] * smoothness +
            weights[2] * obstacle_cost +
            weights[3] * (corridor_width > 0.5 and 0.0 or 0.1) +  # Prefer wider corridors
            weights[4] * 0.1 +
            weights[5] * (min_clearance / 1.0) +
            weights[6] * 0.05
        )
        
        return total_cost


    def publish_path(self, path):
        """
        IMPROVED: ALWAYS publish a path - never leave controller waiting.
        Refuse to publish if:
          - minimum clearance < 0.35m (robot radius + margin)
          - OR minimum corridor width < 0.8m (robot width + margin)
        """
        if not path or len(path) < 2:
            if self.debug_mode:
                self.get_logger().warn("⚠️ Cannot publish empty path")
            return

        # --- HARD SAFETY CHECK 1: minimum clearance ---
        min_clearance = self._get_minimum_clearance_on_path(path)
        if min_clearance < 0.35:
            self.get_logger().error(
                f"🚨 REFUSING to publish UNSAFE path: min_clearance={min_clearance:.2f}m < 0.35m"
            )
            return

        # --- HARD SAFETY CHECK 2: minimum corridor width ---
        min_width, _ = self._get_minimum_corridor_width_on_path(path)
        if min_width < 0.8:
            self.get_logger().error(
                f"🚨 REFUSING to publish UNSAFE path: corridor width {min_width:.2f}m < 0.8m"
            )
            return
        # --------------------------------------------------

        # Validate path safety (logs warnings but does not block)
        if not self.validate_path_safety(path, strict=False):
            if self.debug_mode:
                self.get_logger().warn(
                    f"⚠️ Publishing path with tight clearance (controller will handle)"
                )

        # Convert to ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(path):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.last_published_path = path

        if self.debug_mode:
            self.get_logger().info(
                f"📤 Published path: {len(path)} points, min_clearance={min_clearance:.2f}m, min_corridor_width={min_width:.2f}m"
            )


    def _publish_path_message(self, path_points):
        """Internal method to publish Path message without validation."""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # Calculate yaw
            if i < len(path_points) - 1:
                next_x, next_y = path_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif i > 0:
                prev_x, prev_y = path_points[i - 1]
                yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0
            
            pose.pose.orientation = self.yaw_to_quaternion(yaw)
            path.poses.append(pose)
        
        self.path_pub.publish(path)
        self.last_published_path = path_points
        
        if self.debug_mode:
            path_length = self.calculate_path_length(np.array(path_points))
            self.get_logger().debug(f"Path published: {len(path.poses)} waypoints, length={path_length:.2f}m")
    def yaw_to_quaternion(self, yaw):
        """Yaw to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def quat_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        import math
        sin_half = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cos_half = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(sin_half, cos_half)


def main():
    rclpy.init()
    node = CompletePathOptimizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Exception in spin: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
