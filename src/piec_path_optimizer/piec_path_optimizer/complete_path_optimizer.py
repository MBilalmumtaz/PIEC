
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
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
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
PATH_START_DEVIATION_THRESHOLD = 0.1  # meters - threshold for auto-correcting path start
PATH_START_WARNING_THRESHOLD = 0.2  # meters - threshold for logging warnings
# When the nearest waypoint on the stale path exceeds this distance from the robot,
# generate a fresh quick path instead of trimming the stale path.
PATH_REPLAN_NEAREST_WAYPOINT_DIST = 0.5  # metres

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
        # Path update throttling - BUG FIX to prevent frequent regeneration
        self.last_path_position = None  # Track last position when path was generated
        self.path_update_distance_threshold = 0.3  # Only regenerate if moved > 30cm
        # ADD THESE ATTRIBUTES (PINN tracking):
        self.pinn_call_count = 0
        self.pinn_timeout_count = 0
        self.pinn_success_count = 0
        self.pinn_response_times = deque(maxlen=100)  # For tracking performance
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
        self.obstacle_grid_resolution = 0.1
        self.obstacle_grid = None
        self.free_space_grid = None
        self.free_space_waypoints = []
        
        # Escape paths - IMPROVED
        self.escape_paths_generated = 0
        self.last_escape_time = 0.0
        self.last_escape_pose = None
        self.last_escape_direction = None
        self.escape_attempts = defaultdict(int)
        self.escape_success_history = deque(maxlen=5)
        self.escape_path_history = deque(maxlen=5)  # FIXED: Added missing attribute
        
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
        # In __init__, add a publisher
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create a BEST_EFFORT QoS profile for laser scan
        scan_qos = QoSProfile(
     
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
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
            'objective_weights': [0.12, 0.08, 0.35, 0.08, 0.12, 0.15, 0.1],
            "obstacle_penalty_weight": 100.0,        # Increased from 6.0,12
            "min_obstacle_distance": 0.6,           # Reduced from 0.6
            'escape_clearance': 0.7,
            'max_escape_attempts': 3,
            'escape_cooldown': 6.0,
            'significant_turning_threshold_deg': 30.0,  # Threshold for curved path generation
            'obstacle_grid_size': 150,
            'obstacle_grid_resolution': 0.1,
            'debug_mode': True,
            'goal_completion_distance': 0.25,
            'robot_radius': 0.35,
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
    
    def initialize_obstacle_grid(self):
        """Initialize obstacle and free space grids"""
        size = self.obstacle_grid_size
        self.obstacle_grid = np.zeros((size, size), dtype=np.float32)
        self.free_space_grid = np.ones((size, size), dtype=np.float32)
        self.get_logger().info(f"✅ Grids initialized: {size}x{size} (resolution: {self.obstacle_grid_resolution}m)")
    
    def setup_pinn_client(self):
        """Setup PINN service client - FIXED VERSION"""
        try:
            self.get_logger().info(f"Creating PINN client for service: {self.pinn_service_name}")
            
            if not PINN_SERVICE_AVAILABLE:
                self.get_logger().warn("PINN service type not available, disabling PINN")
                self.use_pinn = False
                self.pinn_ready_event.set()
                return
            
            # Create client
            self.pinn_client = self.create_client(
                PINN_SERVICE_TYPE,
                self.pinn_service_name
            )
            
            self.get_logger().info(f"✅ Created PINN client with service type: {PINN_SERVICE_TYPE.__name__}")
            
            # Wait for service with timeout
            self.get_logger().info(f"Waiting for PINN service: {self.pinn_service_name} (timeout: {self.pinn_timeout}s)")
            
            start_time = time.time()
            service_ready = False
            
            while time.time() - start_time < self.pinn_timeout and rclpy.ok():
                if self.pinn_client.wait_for_service(timeout_sec=0.5):
                    service_ready = True
                    break
                
                self.get_logger().debug(f"Waiting for PINN service... ({time.time() - start_time:.1f}s)")
                time.sleep(0.1)
            
            if service_ready:
                self.pinn_service_available = True
                self.pinn_ready_event.set()
                self.get_logger().info("✅ PINN service is available!")
            else:
                self.get_logger().warn(f"❌ PINN service not available after {self.pinn_timeout}s")
                self.pinn_client = None
                self.pinn_service_available = False
                self.pinn_ready_event.set()
                
        except Exception as e:
            self.get_logger().error(f"Failed to setup PINN client: {e}")
            self.pinn_client = None
            self.pinn_service_available = False
            self.pinn_ready_event.set()
    
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
        """Call PINN service with proper timeout handling - FIXED VERSION"""
        if not self.use_pinn or not self.pinn_service_available or self.pinn_client is None:
            if self.debug_mode:
                self.get_logger().debug("PINN disabled or client not available")
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
                future.cancel()
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
        """
        Shift the obstacle grid so that the robot is at the center.
        Called when robot moves beyond a threshold.
        """
        if self.obstacle_grid is None:
            return

        half_size = (self.obstacle_grid_size * self.obstacle_grid_resolution) / 2.0
        current_center_x, current_center_y = self.obstacle_grid_origin

        # If robot is still within the central region (e.g., 2 m), do nothing
        if math.hypot(robot_x - current_center_x, robot_y - current_center_y) < 2.0:
            return

        # Compute offset in grid cells
        dx_cells = int((robot_x - current_center_x) / self.obstacle_grid_resolution)
        dy_cells = int((robot_y - current_center_y) / self.obstacle_grid_resolution)

        # Create a new zeroed grid
        new_grid = np.zeros((self.obstacle_grid_size, self.obstacle_grid_size), dtype=np.float32)

        # Copy overlapping region from old grid to new grid
        for i in range(self.obstacle_grid_size):
            for j in range(self.obstacle_grid_size):
                old_i = i - dx_cells
                old_j = j - dy_cells
                if 0 <= old_i < self.obstacle_grid_size and 0 <= old_j < self.obstacle_grid_size:
                    new_grid[i, j] = self.obstacle_grid[old_i, old_j]

        self.obstacle_grid = new_grid
        self.obstacle_grid_origin = (robot_x, robot_y)

        if self.debug_mode:
            self.get_logger().info(f"Re‑centered costmap to ({robot_x:.2f}, {robot_y:.2f})")
    def odom_callback(self, msg: Odometry):
        """Update robot pose"""
        self.robot_pose = msg
        
        if self.obstacle_grid_origin is None:
            self.obstacle_grid_origin = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )
        
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        current_time = time.time()
        self.stuck_positions.append((current_pos, current_time))
        self.position_history.append(current_pos)
        
        # Track progress toward goal
        if self.goal_pose:
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            current_distance = math.hypot(goal_x - current_pos[0], goal_y - current_pos[1])
            self.progress_toward_goal.append((current_time, current_distance))
            self.last_goal_distance = current_distance
        
        # Update uncertainty planner
        self.uncertainty_planner.update_uncertainty_map(msg)
        self.last_position = current_pos

        # Re‑center costmap if needed
        if self.obstacle_grid_origin is not None:
            self.recenter_obstacle_grid(msg.pose.pose.position.x, msg.pose.pose.position.y)
    def is_straight_path_clear(self, start_x, start_y, goal_x, goal_y, check_distance=3.0):
        """Check if straight line to goal is clear for a certain distance"""
        if self.obstacle_grid is None:
            return True
        
        distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)
        check_dist = min(check_distance, distance_to_goal)
        
        if check_dist < 0.5:
            return True
        
        # Check points along straight line
        num_checks = int(check_dist / 0.2) + 1
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)
        
        for i in range(1, num_checks + 1):
            dist = i * (check_dist / num_checks)
            check_x = start_x + dist * math.cos(goal_dir)
            check_y = start_y + dist * math.sin(goal_dir)
            
            clearance = self.get_clearance_at_point(check_x, check_y)
            if clearance < self.min_obstacle_distance + 0.3:
                return False
        
        return True
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
        """Check if path should be updated based on robot movement - BUG FIX for frequent regeneration"""
        if self.last_path_position is not None:
            dist_moved = math.hypot(
                start_x - self.last_path_position[0],
                start_y - self.last_path_position[1]
            )
            if dist_moved < self.path_update_distance_threshold:
                # Robot hasn't moved enough, don't regenerate path
                return False
        
        # Update last path position
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
        # This prevents stuck detection when robot is manually locked or before goal
        if not hasattr(self, 'goal_received_time') or self.goal_received_time is None:
            return
        
        current_time = time.time()
        current_pos = (self.robot_pose.pose.pose.position.x, 
                      self.robot_pose.pose.pose.position.y)
        
        # CRITICAL: Cooldown after goal received or last stuck detection
        time_since_goal = current_time - self.goal_received_time
        if time_since_goal < 8.0:  # MUST wait 8 seconds after new goal
            return
        
        # Cooldown after last stuck detection
        if current_time - self.last_stuck_time < 20.0:  # Increased to 20s
            return
        
        # REQUIRE sufficient data (50 samples = ~5 seconds at 10Hz)
        if len(self.stuck_positions) < 50:  # Increased from 20
            return
        
        # Calculate metrics over LONGER window
        time_window = 15.0  # Increased from 12.0
        relevant_positions = [(pos, t) for pos, t in self.stuck_positions 
                             if current_time - t <= time_window]
        
        # Need at least 40 samples in the window
        if len(relevant_positions) < 40:
            return
        
        positions = [pos for pos, _ in relevant_positions]
        
        # Calculate total linear movement
        total_movement = sum(
            math.hypot(positions[i+1][0] - positions[i][0],
                      positions[i+1][1] - positions[i][1])
            for i in range(len(positions) - 1)
        )
        
        # Calculate radius (oscillation detection)
        center = np.mean(positions, axis=0)
        radius = np.mean([math.hypot(p[0]-center[0], p[1]-center[1]) 
                         for p in positions])
        
        # Get distance to goal
        goal_dist = math.hypot(
            self.goal_pose.pose.position.x - current_pos[0],
            self.goal_pose.pose.position.y - current_pos[1]
        )
        
        # Determine if stuck
        is_stuck = False
        reasons = []
        
        # NEVER mark as stuck if:
        # 1. Very close to goal (< 0.35m)
        if goal_dist < 0.35:
            return
        
        # 2. Must wait at least 12 seconds total before checking stuck
        time_since_goal = current_time - self.goal_received_time
        if time_since_goal < 12.0:
            return
        
        # **NEW: Before declaring stuck, verify forward is actually blocked**
        if self.robot_pose and self.goal_pose:
            rx = self.robot_pose.pose.pose.position.x
            ry = self.robot_pose.pose.pose.position.y
            gx = self.goal_pose.pose.position.x
            gy = self.goal_pose.pose.position.y

            forward_blocked = self.is_forward_path_blocked(rx, ry, gx, gy, min_clearance=0.5)

            if not forward_blocked:
                # Forward is clear - NOT stuck, just slow progress
                if self.debug_mode and current_time - self.last_stuck_time > 10.0:
                    self.get_logger().info("✅ Forward path is CLEAR - no escape needed")
                self.stuck_positions.clear()  # Reset stuck tracking
                return

        # STRICTER thresholds for stuck detection
        # Only mark stuck if BOTH conditions are true:
        # 1. Very low movement (< 0.15m in 15 seconds - more strict)
        # 2. Small radius (< 0.30m) indicating spinning
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
            
            self.generate_adaptive_escape_path()
            self.stuck_positions.clear()  # Reset after generating escape
        else:
            # Reset consecutive checks if we're moving
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
            return
        
        x = self.robot_pose.pose.pose.position.x
        y = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y

        # **NEW: Check if forward is actually blocked before generating escape**
        if not self.is_forward_path_blocked(x, y, goal_x, goal_y, min_clearance=0.5):
            if self.debug_mode:
                self.get_logger().info("✅ Forward path CLEAR - skipping escape path generation")
            return

        # Forward IS blocked - continue with escape logic
        self.last_escape_time = current_time
        self.escape_paths_generated += 1
        
        # Find ALL clear directions (not just best one)
        clear_directions = []
        for angle in np.arange(0, 2*math.pi, math.pi/8):
            clearance = self.get_clearance_in_direction(x, y, angle)
            if clearance > 0.6:  # Require good clearance
                # Score based on alignment with goal
                goal_angle = math.atan2(goal_y - y, goal_x - x)
                angle_diff = abs(self.normalize_angle(angle - goal_angle))
                score = clearance * (1.0 - angle_diff / math.pi)
                clear_directions.append((angle, clearance, score))
        
        if not clear_directions:
            # No clear directions - generate backward escape
            self.generate_backward_escape(x, y, goal_x, goal_y)
            return
        
        # Sort by score
        clear_directions.sort(key=lambda x: x[2], reverse=True)
        best_angle, best_clearance, _ = clear_directions[0]
        
        # Generate path in best direction, then arc toward goal
        path = [(x, y)]
        
        # Move in clear direction
        escape_dist = min(1.5, best_clearance * 0.7)
        mid_x = x + escape_dist * math.cos(best_angle)
        mid_y = y + escape_dist * math.sin(best_angle)
        path.append((mid_x, mid_y))
        
        # Add arc toward goal
        goal_angle = math.atan2(goal_y - mid_y, goal_x - mid_x)
        arc_steps = 3
        for i in range(1, arc_steps + 1):
            t = i / arc_steps
            angle = best_angle + t * self.normalize_angle(goal_angle - best_angle)
            dist = 0.5 * t
            px = mid_x + dist * math.cos(angle)
            py = mid_y + dist * math.sin(angle)
            path.append((px, py))
        
        path.append((goal_x, goal_y))
        
        self.publish_path(path)
        self.get_logger().info(f"🔄 Escape path: angle={math.degrees(best_angle):.0f}°, clearance={best_clearance:.2f}m")
        
        # Store in history
        self.escape_path_history.append({
            'time': current_time,
            'path': path,
            'type': 'smart_escape'
        })

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
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
        """Get clearance in direction"""
        if self.obstacle_grid is None:
            return 5.0
        
        max_distance = 2.0
        step = 0.1
        
        for distance in np.arange(0.1, max_distance, step):
            check_x = x + distance * math.cos(angle)
            check_y = y + distance * math.sin(angle)
            
            cell_x, cell_y = self.world_to_grid(check_x, check_y)
            if cell_x is not None and cell_y is not None:
                if self.obstacle_grid[cell_x, cell_y] > 0.3:
                    return distance - 0.1
        
        return max_distance

    def is_forward_path_blocked(self, start_x, start_y, goal_x, goal_y, min_clearance=0.5):
        """
        Check if forward path toward goal is actually blocked
        Returns True if blocked, False if clear
        """
        if self.obstacle_grid is None:
            return False

        # Calculate direction toward goal
        goal_dir = math.atan2(goal_y - start_y, goal_x - start_x)

        # Check clearance in forward direction (toward goal)
        forward_clearance = self.get_clearance_in_direction(start_x, start_y, goal_dir)

        # Also check slight angles left/right of goal direction
        left_clearance = self.get_clearance_in_direction(start_x, start_y, goal_dir + 0.3)
        right_clearance = self.get_clearance_in_direction(start_x, start_y, goal_dir - 0.3)

        # If ANY of these directions has good clearance, forward is NOT blocked
        max_clearance = max(forward_clearance, left_clearance, right_clearance)

        is_blocked = max_clearance < min_clearance

        if self.debug_mode and is_blocked:
            self.get_logger().info(
                f"🚧 Forward path blocked: clearances = "
                f"center={forward_clearance:.2f}m, left={left_clearance:.2f}m, right={right_clearance:.2f}m"
            )

        return is_blocked
    
    def optimization_timer_callback(self):
        """Optimization timer"""
        if self.optimization_active or self.goal_pose is None:
            return
        
        if self.robot_pose is None:
            return
        
        current_time = time.time()
        time_since_last = current_time - self.last_optimization_time
        if time_since_last < self.planning_rate:
            return
        
        self.trigger_optimization()
    
    def trigger_optimization(self):
        """Start optimization"""
        if self.optimization_active:
            return

        # Publish a quick current-pose path immediately so the controller always
        # has a fresh path while the slower optimizer runs in the background.
        # Use a curved Bezier path when the goal requires significant turning.
        if self.robot_pose is not None and self.goal_pose is not None:
            current_x = self.robot_pose.pose.pose.position.x
            current_y = self.robot_pose.pose.pose.position.y
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            if self.requires_significant_turning(
                    current_x, current_y, goal_x, goal_y,
                    threshold_degrees=self.significant_turning_threshold_deg):
                quick_path = self.generate_curved_path_for_turning(
                    current_x, current_y, goal_x, goal_y
                )
            else:
                quick_path = self.generate_quick_response_path(
                    current_x, current_y, goal_x, goal_y
                )
            self.publish_path(quick_path)

        self.last_optimization_time = time.time()
        self.optimization_active = True

        thread = threading.Thread(target=self.run_enhanced_optimization)
        thread.daemon = True
        thread.start()
    
    def generate_free_space_focused_path(self, start_x, start_y, goal_x, goal_y):
        """Generate free space focused path"""
        free_directions = self.find_free_space_directions(start_x, start_y, goal_x, goal_y)
        
        if free_directions:
            best_dir, best_clearance, best_score = free_directions[0]
            
            path = []
            path.append((start_x, start_y))
            
            first_dist = min(best_clearance * 0.7, 1.5)
            first_x = start_x + first_dist * math.cos(best_dir)
            first_y = start_y + first_dist * math.sin(best_dir)
            path.append((first_x, first_y))
            
            second_dir = math.atan2(goal_y - first_y, goal_x - first_x)
            second_dist = min(1.0, math.hypot(goal_x - first_x, goal_y - first_y) * 0.7)
            second_x = first_x + second_dist * math.cos(second_dir)
            second_y = first_y + second_dist * math.sin(second_dir)
            path.append((second_x, second_y))
            
            path.append((goal_x, goal_y))
            
            return path
        
        return self.generate_obstacle_aware_path(start_x, start_y, goal_x, goal_y)
    
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
        """Generate quick response path - OPTIMIZED FOR REAL ROBOT"""
        distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)
        
        # REAL ROBOT: Always use straight line for < 5m
        if distance_to_goal < 5.0:
            return [(start_x, start_y), (goal_x, goal_y)]
        
        # Check if straight path is clear
        if self.is_straight_path_clear(start_x, start_y, goal_x, goal_y, min(distance_to_goal, 3.0)):
            return [(start_x, start_y), (goal_x, goal_y)]
        
        # Simple curve if needed
        path = []
        path.append((start_x, start_y))
        
        # Add one intermediate point
        mid_x = start_x + (goal_x - start_x) * 0.5
        mid_y = start_y + (goal_y - start_y) * 0.5
        
        # Offset perpendicularly
        dx = goal_x - start_x
        dy = goal_y - start_y
        dist = math.hypot(dx, dy)
        
        if dist > 0:
            # Normalize
            dx /= dist
            dy /= dist
            
            # Perpendicular offset (left or right)
            offset_dist = min(0.8, dist * 0.2)  # Max 0.8m offset
            offset_x = -dy * offset_dist
            offset_y = dx * offset_dist
            
            mid_x += offset_x
            mid_y += offset_y
        
        path.append((mid_x, mid_y))
        path.append((goal_x, goal_y))
        
        return path
    
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
    
    def generate_curved_path_for_turning(self, start_x, start_y, goal_x, goal_y):
        """Generate a cubic Bezier curve path for goals requiring turning.

        Uses the formula B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3 where
        P0 is the start, P1/P2 are control points, and P3 is the goal.
        """
        # Get robot orientation
        if self.robot_pose is not None:
            robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
        else:
            robot_yaw = 0.0

        goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)
        distance = math.hypot(goal_x - start_x, goal_y - start_y)

        # Control point offset: fraction of distance along each tangent
        cp_offset = distance * BEZIER_CONTROL_POINT_RATIO

        # P1: depart from start along robot heading
        p1_x = start_x + cp_offset * math.cos(robot_yaw)
        p1_y = start_y + cp_offset * math.sin(robot_yaw)

        # P2: arrive at goal from the direction of the straight-line approach
        p2_x = goal_x - cp_offset * math.cos(goal_angle)
        p2_y = goal_y - cp_offset * math.sin(goal_angle)

        # Sample the Bezier curve; at least 10 waypoints for smooth curvature
        num_waypoints = max(10, int(distance / BEZIER_WAYPOINT_SPACING))

        path = []
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            mt = 1.0 - t
            # Cubic Bezier: B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
            bx = (mt ** 3 * start_x
                  + 3.0 * mt ** 2 * t * p1_x
                  + 3.0 * mt * t ** 2 * p2_x
                  + t ** 3 * goal_x)
            by = (mt ** 3 * start_y
                  + 3.0 * mt ** 2 * t * p1_y
                  + 3.0 * mt * t ** 2 * p2_y
                  + t ** 3 * goal_y)
            path.append((bx, by))

        return path
    
    def run_enhanced_optimization(self):
        """Main optimization - WITH ALL FEATURES - FIXED PINN STATS"""
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
            
            # Throttle path updates - only regenerate if robot moved significantly
            if not self.should_update_path(start_x, start_y):
                return
            
            distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)
            if distance_to_goal < self.goal_completion_distance:
                self.get_logger().info(f"🎯 Already at goal ({distance_to_goal:.2f}m)")
                self.optimization_active = False
                return
            
            # Check if we should use simple straight path
            if self.should_use_simple_straight_path(start_x, start_y, goal_x, goal_y):
                if self.debug_mode:
                    self.get_logger().info("📏 Using simple straight path (clear path detected)")
                straight_path = self.generate_straight_path_with_waypoints(start_x, start_y, goal_x, goal_y)
                self.publish_path(straight_path)
                self.optimization_active = False
                # No PINN calls were made
                self.last_pinn_calls = 0
                self.last_pinn_success = 0
                return
            
            # Use curved path as seed for PINN optimization
            curved_path_seed = None
            if self.requires_significant_turning(start_x, start_y, goal_x, goal_y, threshold_degrees=60):
                robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
                goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)
                angle_diff = abs(math.atan2(math.sin(goal_angle - robot_yaw), math.cos(goal_angle - robot_yaw)))

                if self.debug_mode:
                    self.get_logger().info(
                        f"🔄 Large turn required (angle={math.degrees(angle_diff):.1f}°) - "
                        f"using curved path as seed for PINN optimization"
                    )
                curved_path_seed = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
                if self.debug_mode:
                    seed_array = np.array([[p[0], p[1]] for p in curved_path_seed])
                    seed_curvature = self.calculate_curvature(seed_array)
                    self.get_logger().info(
                        f"🎨 Generated Bezier curve: curvature={seed_curvature:.2f}"
                    )

            if self.stuck_count >= self.max_escape_attempts:
                if self.debug_mode:
                    self.get_logger().warn("🚨 Multiple stuck attempts - using free-space focused path")
                best_path = self.generate_free_space_focused_path(start_x, start_y, goal_x, goal_y)
                self.last_pinn_calls = 0
                self.last_pinn_success = 0
            else:
                # Pass curved path seed if available
                best_path = self.run_enhanced_nsga2_with_free_space(
                    start_x, start_y, goal_x, goal_y,
                    self.population_size,
                    self.generations,
                    seed_path=curved_path_seed
                )
                # The NSGA‑II function has already set self.last_pinn_calls and self.last_pinn_success
            
            if best_path is None or len(best_path) < 2:
                best_path = self.generate_quick_response_path(start_x, start_y, goal_x, goal_y)
                self.last_pinn_calls = 0
                self.last_pinn_success = 0

            # ALWAYS publish the optimized path after corrections
            deviation_start = math.hypot(best_path[0][0] - start_x, best_path[0][1] - start_y)
            deviation_end = math.hypot(best_path[-1][0] - goal_x, best_path[-1][1] - goal_y)

            if deviation_start > 0.1:
                self.get_logger().error(
                    f"❌ Path start error: {deviation_start:.3f}m. Forcing correction."
                )
                best_path[0] = (start_x, start_y)

            if deviation_end > 0.1:
                self.get_logger().warn(
                    f"⚠️ Path end error: {deviation_end:.3f}m. Forcing correction."
                )
                best_path[-1] = (goal_x, goal_y)

            elapsed = time.time() - start_time
            if elapsed > self.optimization_timeout:
                self.get_logger().warn(
                    f"⚠️ Optimization took {elapsed:.1f}s "
                    f"(>{self.optimization_timeout:.1f}s timeout). "
                    f"Publishing optimized path anyway (may be slightly stale)."
                )

            self.publish_path(best_path)

            # Use the stored local counts from the NSGA‑II run
            pinn_calls_this_optimization = self.last_pinn_calls
            pinn_success_this_optimization = self.last_pinn_success
            
            if self.debug_mode:
                if pinn_calls_this_optimization > 0:
                    pinn_success_rate = (pinn_success_this_optimization / pinn_calls_this_optimization * 100) if pinn_calls_this_optimization > 0 else 0
                    self.get_logger().info(
                        f"✅ Optimization #{self.optimization_count} in {elapsed:.2f}s | "
                        f"PINN: {pinn_calls_this_optimization} calls, {pinn_success_rate:.1f}% success"
                    )
                else:
                    self.get_logger().info(f"✅ Optimization #{self.optimization_count} in {elapsed:.2f}s (no PINN calls)")
            elif pinn_calls_this_optimization == 0:
                self.get_logger().warn(
                    "⚠️ PINN NOT USED in this optimization (early return or restrictive conditions)"
                )
        
        except Exception as e:
            self.get_logger().error(f"Optimization error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
            if self.robot_pose and self.goal_pose:
                fallback = self.generate_quick_response_path(
                    self.robot_pose.pose.pose.position.x,
                    self.robot_pose.pose.pose.position.y,
                    self.goal_pose.pose.position.x,
                    self.goal_pose.pose.position.y
                )
                self.publish_path(fallback)
        
        finally:
            self.optimization_active = False
    def run_enhanced_nsga2_with_free_space(self, start_x, start_y, goal_x, goal_y, pop_size, generations, seed_path=None):
        """NSGA-II with free space - RELAXED PINN condition for higher usage."""
        HARD_TIMEOUT = max(1.0, self.optimization_timeout - 1.0)  # seconds
        optimization_start = time.time()

        population = self.initialize_enhanced_population(
            start_x, start_y, goal_x, goal_y, pop_size
        )

        # Seed handling (unchanged)
        seed_path_corrected = None
        requires_turn = seed_path is not None
        if seed_path is not None:
            if self.debug_mode:
                self.get_logger().info("🌱 Seeding population with curved path")
            seed_path_corrected = (
                [(start_x, start_y)] + list(seed_path[1:-1]) + [(goal_x, goal_y)]
            )
            population[0] = seed_path_corrected
            for i in range(1, min(MAX_CURVED_SEED_REPLACEMENTS + 1, len(population))):
                arr = np.array([[p[0], p[1]] for p in population[i]])
                if self.calculate_curvature(arr) < MIN_TURN_CURVATURE_THRESHOLD:
                    population[i] = seed_path_corrected

        required_angle_deg = 0.0
        if requires_turn and self.robot_pose is not None:
            robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
            goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)
            required_angle_deg = math.degrees(
                abs(math.atan2(
                    math.sin(goal_angle - robot_yaw),
                    math.cos(goal_angle - robot_yaw)
                ))
            )

        straight_line_length = math.hypot(goal_x - start_x, goal_y - start_y)

        evaluated_individuals = []      # list of paths
        evaluated_objectives = []       # list of objective tuples

        total_pinn_calls = 0
        total_pinn_success = 0
        timeout_occurred = False

        for gen in range(generations):
            if time.time() - optimization_start > HARD_TIMEOUT:
                if self.debug_mode:
                    self.get_logger().debug(f"Early exit at generation {gen} (hard timeout {HARD_TIMEOUT}s)")
                break

            all_objectives = []
            valid_individuals = []
            pinn_calls_this_gen = 0   # track PINN calls per generation

            for idx, individual in enumerate(population):
                elapsed = time.time() - self.last_optimization_time
                if elapsed > self.optimization_timeout:
                    timeout_occurred = True
                    break

                remaining = self._time_remaining()
                if remaining < 0.8:   # still need some time to evaluate an individual
                    timeout_occurred = True
                    break

                path_array = np.array([[p[0], p[1]] for p in individual])

                obstacle_cost = self.get_enhanced_obstacle_cost(path_array)
                free_space_bonus = self.get_free_space_bonus(path_array)

                if obstacle_cost > self.obstacle_penalty_weight * 10:
                    continue

                path_length = self.calculate_path_length(path_array)
                curvature_cost = self.calculate_curvature(path_array)
                uncertainty_cost = self.uncertainty_planner.get_uncertainty_cost(path_array)
                deviation = abs(path_length - straight_line_length) / max(straight_line_length, 0.1)

                # RELAXED PINN condition: only require enough time for the call itself
                use_pinn_for_this = False
                if (self.use_pinn and self.pinn_service_available and
                        obstacle_cost < self.obstacle_penalty_weight * 15 and
                        pinn_calls_this_gen < self.max_pinn_calls_per_generation):
                    use_pinn_for_this = True

                objectives = self.objective_evaluator.evaluate_all_objectives_with_timeout(
                    path_array,
                    obstacle_cost,
                    uncertainty_cost,
                    straight_line_length,
                    use_pinn=use_pinn_for_this,
                    timeout=0.5
                )

                if use_pinn_for_this:
                    pinn_calls_this_gen += 1
                    total_pinn_calls += 1
                    if objectives[5] > 0.0 and objectives[5] < 1000.0:
                        total_pinn_success += 1
                        if self.debug_mode:
                            self.get_logger().debug(f"PINN successful for path {idx}: energy={objectives[5]:.2f}")

                if requires_turn:
                    path_curvature = self.calculate_curvature(path_array)
                    if path_curvature < MIN_TURN_CURVATURE_THRESHOLD:
                        angle_scale = max(1.0, required_angle_deg / ANGLE_SCALE_DIVISOR_DEG)
                        straight_penalty = (
                            (MIN_TURN_CURVATURE_THRESHOLD - path_curvature)
                            * STRAIGHT_PATH_PENALTY_MULTIPLIER
                            * angle_scale
                        )
                        objectives = list(objectives)
                        objectives[1] += straight_penalty

                all_objectives.append(objectives)
                valid_individuals.append(idx)

            if timeout_occurred:
                break

            for vi in valid_individuals:
                evaluated_individuals.append(population[vi])
                evaluated_objectives.append(all_objectives[valid_individuals.index(vi)])

            if self.debug_mode and total_pinn_calls > 0:
                success_rate = (total_pinn_success / total_pinn_calls * 100) if total_pinn_calls > 0 else 0
                self.get_logger().info(
                    f"📈 PINN Stats after gen {gen}: {total_pinn_calls} calls, "
                    f"{total_pinn_success} successful ({success_rate:.1f}%)"
                )

            if not all_objectives:
                population = self.initialize_enhanced_population(
                    start_x, start_y, goal_x, goal_y, pop_size
                )
                continue

            if time.time() - self.last_optimization_time > self.optimization_timeout:
                break

            # Selection and reproduction (unchanged, with timeout checks)
            if len(all_objectives) >= 3:
                try:
                    selected_indices = nsga2_selection(
                        [population[i] for i in valid_individuals],
                        all_objectives,
                        pop_size
                    )
                    selected_original_indices = [valid_individuals[i] for i in selected_indices]
                    new_population = []
                    for idx in selected_original_indices:
                        if idx < len(population):
                            new_population.append(population[idx])

                    while len(new_population) < pop_size:
                        if time.time() - self.last_optimization_time > self.optimization_timeout:
                            timeout_occurred = True
                            break
                        if random.random() < self.crossover_rate and len(new_population) >= 2:
                            parent1 = random.choice(new_population)
                            parent2 = random.choice(new_population)
                            child = self.crossover_paths(parent1, parent2)
                        else:
                            child = random.choice(new_population)
                        child = self.mutate_with_free_space(child, start_x, start_y, goal_x, goal_y)
                        new_population.append(child)

                    if timeout_occurred:
                        break
                    population = new_population
                except Exception as e:
                    if self.debug_mode:
                        self.get_logger().warn(f"NSGA-II failed: {e}, using enhanced selection")
                    population = self.enhanced_selection(population, valid_individuals, all_objectives, pop_size)
            else:
                population = self.enhanced_selection(population, valid_individuals, all_objectives, pop_size)

            if timeout_occurred:
                break

            if requires_turn and seed_path_corrected is not None and gen < int(generations * SEED_PRESERVATION_RATIO):
                population[0] = seed_path_corrected

            if time.time() - self.last_optimization_time > self.optimization_timeout:
                if self.debug_mode:
                    self.get_logger().debug(f"Optimization timeout after {gen+1} generations")
                break

        # Log total PINN usage
        if total_pinn_calls > 0 and self.debug_mode:
            overall_success_rate = (total_pinn_success / total_pinn_calls * 100) if total_pinn_calls > 0 else 0
            self.get_logger().info(
                f"🔬 NSGA-II PINN Summary: {total_pinn_calls} total calls, "
                f"{total_pinn_success} successful ({overall_success_rate:.1f}%) "
                f"across {generations} generations"
            )

        # Select best individual (same as before)
                # Select best individual using weighted sum of all objectives
        # Select best individual using weighted sum of all objectives
        if evaluated_individuals:
            best_idx = 0
            best_score = float('inf')
            # weights for objectives (match order from evaluate_all_objectives)
            # [length, curvature, obstacle_cost, uncertainty, deviation, pinn_energy, 1-stability]
            weights = [0.1, 0.1, 0.4, 0.1, 0.1, 0.1, 0.1]   # obstacle weight increased
            for i, ind in enumerate(evaluated_individuals):
                obj = evaluated_objectives[i]
                score = sum(w * obj[j] for j, w in enumerate(weights))
                if score < best_score:
                    best_score = score
                    best_idx = i
            best_path = evaluated_individuals[best_idx]
        elif population:
            best_path = population[0]
        elif seed_path_corrected is not None:
            best_path = seed_path_corrected
        else:
            best_path = self.generate_quick_response_path(start_x, start_y, goal_x, goal_y)
        # Curvature validation (unchanged)
        if requires_turn and seed_path_corrected is not None:
            required_angle_deg = 0.0
            if self.robot_pose is not None:
                robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
                current_rx = self.robot_pose.pose.pose.position.x
                current_ry = self.robot_pose.pose.pose.position.y
                goal_angle = math.atan2(goal_y - current_ry, goal_x - current_rx)
                required_angle_deg = math.degrees(
                    abs(math.atan2(
                        math.sin(goal_angle - robot_yaw),
                        math.cos(goal_angle - robot_yaw)
                    ))
                )
            is_valid, actual_curv = self.validate_path_curvature_for_turn(best_path, required_angle_deg)
            if not is_valid:
                self.get_logger().warn(
                    f"⚠️ Selected path too straight (curvature={actual_curv:.3f}) "
                    f"for required turn. Using curved seed instead."
                )
                best_path = seed_path_corrected

        if self.path_smoothing:
            best_path = self.smooth_path_with_free_space(best_path)
        # Store PINN stats for this optimization
        self.last_pinn_calls = total_pinn_calls
        self.last_pinn_success = total_pinn_success
        return best_path
    def get_free_space_bonus(self, path_array):
        """Calculate free space bonus"""
        if self.free_space_grid is None:
            return 0.0
        bonus = 0.0
        
        for i in range(len(path_array) - 1):
            x1, y1 = path_array[i]
            x2, y2 = path_array[i + 1]
            
            num_samples = max(3, int(math.hypot(x2 - x1, y2 - y1) / 0.2))
            for j in range(num_samples):
                t = j / (num_samples - 1) if num_samples > 1 else 0.5
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                
                cell_x, cell_y = self.world_to_grid(x, y)
                if cell_x is not None and cell_y is not None:
                    if self.free_space_grid[cell_x, cell_y] > 0.7:
                        bonus += 0.5
        
        return bonus / max(len(path_array), 1)
    
    def get_enhanced_obstacle_cost(self, path_array):
        """Calculate enhanced obstacle cost"""
        if self.obstacle_grid is None or len(path_array) < 2:
            return 0.0
        
        total_cost = 0.0
        
        for i, (x, y) in enumerate(path_array):
            clearance = self.get_clearance_at_point(x, y)
            
            if clearance < self.min_obstacle_distance:
                penalty = self.obstacle_penalty_weight * (
                    1.0 + (self.min_obstacle_distance - clearance) ** 2
                )
                total_cost += penalty
            elif clearance < self.preferred_clearance:
                penalty = self.obstacle_penalty_weight * 0.5 * (
                    1.0 - clearance / self.preferred_clearance
                )
                total_cost += penalty
            
            cell_x, cell_y = self.world_to_grid(x, y)
            if cell_x is not None and cell_y is not None:
                if self.obstacle_grid[cell_x, cell_y] > 0.5:
                    total_cost += self.obstacle_penalty_weight * 0.3
        
        return total_cost
    
    def generate_obstacle_avoiding_path(self, start_x, start_y, goal_x, goal_y):
        """
        Generate an initial path that explicitly avoids obstacles detected in the costmap.
        Returns a list of (x, y) waypoints (at least 3) that go around the obstacle.
        """
        # If no costmap yet, fallback to straight line
        if self.obstacle_grid is None or self.obstacle_grid_origin is None:
            return self.generate_straight_path_with_waypoints(start_x, start_y, goal_x, goal_y)

        # Compute direction and distance
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.hypot(dx, dy)
        if distance < 0.5:
            return [(start_x, start_y), (goal_x, goal_y)]

        # Normalised direction vector
        if distance > 0:
            ux = dx / distance
            uy = dy / distance
        else:
            ux, uy = 1.0, 0.0

        # Perpendicular vectors (left and right)
        left_x = -uy
        left_y = ux
        right_x = uy
        right_y = -ux

        # Number of samples along the straight line
        num_samples = max(10, int(distance / 0.2))
        samples = []
        for i in range(1, num_samples):
            t = i / num_samples
            x = start_x + t * dx
            y = start_y + t * dy
            clearance = self.get_clearance_at_point(x, y)
            samples.append((x, y, t, clearance))

        # Find the point with minimum clearance (most obstructed)
        min_clearance = float('inf')
        worst_idx = -1
        for idx, (x, y, t, clr) in enumerate(samples):
            if clr < min_clearance:
                min_clearance = clr
                worst_idx = idx

        # If the entire line is clear enough, return straight line
        if min_clearance > self.preferred_clearance:
            return self.generate_straight_path_with_waypoints(start_x, start_y, goal_x, goal_y)

        # The worst point is where we need to deviate
        wx, wy, wt, _ = samples[worst_idx]

        # Search for a safe offset (left and right) at that point
        shift_distance = self.robot_radius * 2 + 0.3   # ~0.9 m
        search_steps = 5
        best_offset = None
        best_clearance = 0.0
        best_is_left = None

        # Try left
        for step in range(1, search_steps + 1):
            cand_x = wx + left_x * shift_distance * step / search_steps
            cand_y = wy + left_y * shift_distance * step / search_steps
            cand_clearance = self.get_clearance_at_point(cand_x, cand_y)
            if cand_clearance > best_clearance:
                best_clearance = cand_clearance
                best_offset = (cand_x, cand_y)
                best_is_left = True

        # Try right
        for step in range(1, search_steps + 1):
            cand_x = wx + right_x * shift_distance * step / search_steps
            cand_y = wy + right_y * shift_distance * step / search_steps
            cand_clearance = self.get_clearance_at_point(cand_x, cand_y)
            if cand_clearance > best_clearance:
                best_clearance = cand_clearance
                best_offset = (cand_x, cand_y)
                best_is_left = False

        if best_offset is None:
            # No safe deviation found – fallback to straight line
            return self.generate_straight_path_with_waypoints(start_x, start_y, goal_x, goal_y)

        dev_x, dev_y = best_offset

        # Create a path with three waypoints: start -> deviation -> goal
        # (Later interpolation will add intermediate points)
        path = [
            (start_x, start_y),
            (dev_x, dev_y),
            (goal_x, goal_y)
        ]

        return path
    
    def initialize_enhanced_population(self, start_x, start_y, goal_x, goal_y, pop_size):
        population = []
        n = max(self.waypoint_count, 2)

        # Always include a straight line path
        straight_path = [
            (start_x + i/(n-1)*(goal_x-start_x),
             start_y + i/(n-1)*(goal_y-start_y))
            for i in range(n)
        ]
        population.append(straight_path)

        # If obstacles are present, add an obstacle‑avoiding path
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        # Check if straight path is clear (quick check)
        straight_clear = self.is_straight_path_clear(start_x, start_y, goal_x, goal_y, min(distance, 5.0))
        if not straight_clear:
            # Obstacles detected – add a path that tries to avoid them
            avoid_path = self.generate_obstacle_avoiding_path(start_x, start_y, goal_x, goal_y)
            if avoid_path:
                population.append(avoid_path)

        # Add free space path (existing)
        free_space_path = self.generate_free_space_focused_path(start_x, start_y, goal_x, goal_y)
        if free_space_path:
            population.append(free_space_path)

        # Use generate_straight_path_with_waypoints instead of generate_quick_response_path
        # so the population always has multi-waypoint paths.
        straight_candidate = self.generate_straight_path_with_waypoints(
            start_x, start_y, goal_x, goal_y
        )
        if straight_candidate:
            population.append(straight_candidate)

        # Fill the rest with variations, but reduce exploration for clear paths
        remaining = pop_size - len(population)

        for i in range(remaining):
            path = []
            path.append((start_x, start_y))

            # Use waypoint_count waypoints for all population members
            num_waypoints = n

            for j in range(1, num_waypoints - 1):
                t = j / (num_waypoints - 1)

                base_x = start_x + t * (goal_x - start_x)
                base_y = start_y + t * (goal_y - start_y)

                # Keep first waypoint (j==1) at exact linear position to avoid
                # start-region random offsets that cause path-start mismatches
                if j == 1:
                    waypoint_x = base_x
                    waypoint_y = base_y
                elif random.random() < self.exploration_factor * 0.5:  # Reduce exploration
                    free_directions = self.find_free_space_directions(base_x, base_y, goal_x, goal_y)
                    if free_directions:
                        angle, clearance, _ = random.choice(free_directions[:2])
                        max_dev = min(clearance * 0.3, 0.5)  # Reduce deviation
                        dev_dist = random.uniform(0, max_dev)

                        waypoint_x = base_x + dev_dist * math.cos(angle)
                        waypoint_y = base_y + dev_dist * math.sin(angle)
                    else:
                        angle = random.uniform(0, 2 * math.pi)
                        max_dev = distance * 0.1  # Reduce deviation
                        dev_dist = random.uniform(0, max_dev)

                        waypoint_x = base_x + dev_dist * math.cos(angle)
                        waypoint_y = base_y + dev_dist * math.sin(angle)
                else:
                    max_dev = distance * 0.1  # Reduce deviation
                    angle = random.uniform(0, 2 * math.pi)
                    dev_dist = random.uniform(0, max_dev)

                    waypoint_x = base_x + dev_dist * math.cos(angle)
                    waypoint_y = base_y + dev_dist * math.sin(angle)

                path.append((waypoint_x, waypoint_y))

            path.append((goal_x, goal_y))

            population.append(path)

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
    
    def enhanced_selection(self, population, valid_indices, objectives, pop_size):
        """Enhanced selection"""
        new_population = []
        tournament_size = 4
        
        while len(new_population) < pop_size:
            best_idx = -1
            best_fitness = float('inf')
            
            for _ in range(tournament_size):
                if not valid_indices:
                    break
                idx = random.choice(valid_indices)
                if idx < len(objectives):
                    fitness = self.objective_evaluator.calculate_weighted_fitness(objectives[idx])
                    if fitness < best_fitness:
                        best_fitness = fitness
                        best_idx = idx
            
            if best_idx != -1 and best_idx < len(population):
                selected = population[best_idx]
                # Preserve all waypoints: interpolate if path is shorter than waypoint_count
                if len(selected) < self.waypoint_count:
                    selected = self._interpolate_path_to_count(selected, self.waypoint_count)
                new_population.append(selected)

                if random.random() < self.mutation_rate:
                    mutated = self.mutate_with_free_space(
                        new_population[-1],
                        new_population[-1][0][0],
                        new_population[-1][0][1],
                        new_population[-1][-1][0],
                        new_population[-1][-1][1]
                    )
                    new_population[-1] = mutated
            else:
                if valid_indices:
                    idx = random.choice(valid_indices)
                    if idx < len(population):
                        selected = population[idx]
                        if len(selected) < self.waypoint_count:
                            selected = self._interpolate_path_to_count(selected, self.waypoint_count)
                        new_population.append(selected)
        
        return new_population
    
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
        """Laser callback"""
        self.laser_scan = msg.ranges
        self.laser_frame = msg.header.frame_id   # store the frame_id from the message

        # Debug: print the laser frame every 2 seconds
        self.get_logger().info(f"Laser frame: {self.laser_frame}", throttle_duration_sec=2.0)

        if self.scan_angles is None:
            self.scan_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(len(msg.ranges))
            ]
    
    def publish_costmap(self):
        if self.obstacle_grid is None or self.obstacle_grid_origin is None:
            return

        ox, oy = self.obstacle_grid_origin
        resolution = self.obstacle_grid_resolution
        grid_size = self.obstacle_grid_size
        half_size = (grid_size * resolution) / 2.0

        origin_x = ox - half_size
        origin_y = oy - half_size

        # Debug info (optional)
        if self.debug_mode:
            max_val = float(np.max(self.obstacle_grid))
            self.get_logger().info(
                f"Costmap origin: ({origin_x:.2f}, {origin_y:.2f}), max occupancy: {max_val:.2f}",
                throttle_duration_sec=2.0
            )
            if self.robot_pose:
                rx = self.robot_pose.pose.pose.position.x
                ry = self.robot_pose.pose.pose.position.y
                self.get_logger().info(
                    f"Map origin: ({origin_x:.2f}, {origin_y:.2f}) | "
                    f"Robot: ({rx:.2f}, {ry:.2f}) | "
                    f"Max grid value: {max_val:.3f}",
                    throttle_duration_sec=2.0
                )

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'odom'
        grid_msg.info.resolution = float(resolution)
        grid_msg.info.width = int(grid_size)
        grid_msg.info.height = int(grid_size)
        grid_msg.info.origin.position.x = float(origin_x)
        grid_msg.info.origin.position.y = float(origin_y)
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # Flatten correctly: index = x + y * width (x fastest)
        threshold = 0.1
        data = [0] * (grid_size * grid_size)

        for gx in range(grid_size):          # x index
            for gy in range(grid_size):      # y index
                val = float(self.obstacle_grid[gx, gy])
                if val > threshold:
                    occ = 100
                elif val > 0.05:
                    occ = int(max(0.0, min(1.0, val)) * 100.0)
                else:
                    occ = 0
                idx = gx + gy * grid_size
                data[idx] = occ

        grid_msg.data = data
        self.costmap_pub.publish(grid_msg)
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
        """Update obstacle map by transforming laser points to odom using TF."""
        if self.robot_pose is None or self.laser_scan is None or self.laser_frame is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
                self.laser_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)  # timeout 0.1 s
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup from {self.laser_frame} to odom failed: {e}",
                throttle_duration_sec=1.0
            )
            return

        # Decay previous obstacles (slower decay)
        self.obstacle_grid *= 0.8   # was 0.8
        for i, range_val in enumerate(self.laser_scan):
            if 0.1 < range_val < 10.0:
                angle = self.scan_angles[i]
                local_x = range_val * math.cos(angle)
                local_y = range_val * math.sin(angle)
                local_z = 0.0

                p = PointStamped()
                p.header.frame_id = self.laser_frame
                p.header.stamp = self.get_clock().now().to_msg()
                p.point.x = local_x
                p.point.y = local_y
                p.point.z = local_z

                try:
                    p_odom = do_transform_point(p, trans)
                except Exception as e:
                    self.get_logger().warn(f"Transform failed: {e}", throttle_duration_sec=1.0)
                    continue

                world_x = p_odom.point.x
                world_y = p_odom.point.y

                self.update_obstacle_cells(world_x, world_y, range_val)

        self.publish_costmap()
    

    
    def get_clearance_at_point(self, x, y):
        """Get clearance at point"""
        if self.obstacle_grid is None:
            return float('inf')
        
        cell_x, cell_y = self.world_to_grid(x, y)
        if cell_x is None or cell_y is None:
            return float('inf')
        
        max_search_radius = int(1.5 / self.obstacle_grid_resolution)
        min_distance = float('inf')
        
        for dx in range(-max_search_radius, max_search_radius + 1):
            for dy in range(-max_search_radius, max_search_radius + 1):
                gx = cell_x + dx
                gy = cell_y + dy
                
                if 0 <= gx < self.obstacle_grid_size and 0 <= gy < self.obstacle_grid_size:
                    if self.obstacle_grid[gx, gy] > 0.2:
                        distance = math.hypot(dx, dy) * self.obstacle_grid_resolution
                        if distance < min_distance:
                            min_distance = distance
        
        return min_distance if min_distance != float('inf') else 2.0
    
    def world_to_grid(self, x, y):
        """Convert world (odom) coordinates to grid indices (gx, gy)."""
        if self.obstacle_grid_origin is None:
            return None, None

        cx, cy = self.obstacle_grid_origin          # grid center in world
        res = self.obstacle_grid_resolution
        size = self.obstacle_grid_size
        half = size // 2

        gx = int((x - cx) / res) + half
        gy = int((y - cy) / res) + half

        if 0 <= gx < size and 0 <= gy < size:
            return gx, gy
        return None, None
    
    def calculate_path_length(self, path_array):
        """Calculate path length"""
        if len(path_array) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path_array) - 1):
            dx = path_array[i+1, 0] - path_array[i, 0]
            dy = path_array[i+1, 1] - path_array[i, 1]
            length += math.hypot(dx, dy)
        
        return length
    
    def calculate_curvature(self, path_array):
        """Calculate curvature"""
        if len(path_array) < 3:
            return 0.0

        total_curvature = 0.0
        for i in range(1, len(path_array) - 1):
            p1 = path_array[i-1]
            p2 = path_array[i]
            p3 = path_array[i+1]

            v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
            v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])

            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)

            if norm1 > 0 and norm2 > 0:
                v1_norm = v1 / norm1
                v2_norm = v2 / norm2
                dot_product = np.dot(v1_norm, v2_norm)
                dot_product = np.clip(dot_product, -1.0, 1.0)
                total_curvature += math.acos(dot_product)

        return total_curvature / max(len(path_array) - 2, 1)

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
        """Goal callback - TRACK TIME"""
        self.goal_pose = msg
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        self.goal_received_time = time.time()  # ADD THIS - track when goal was received
        # Clear last path position to force path regeneration on new goals
        self.last_path_position = None

        if self.debug_mode:
            self.get_logger().info(f"🎯 New goal: ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f})")

        # Publish a quick-response path immediately so the controller has
        # something to follow while the optimizer runs in the background.
        # Use a curved Bezier path when the goal requires significant turning
        # so the controller gets a realistic curved path right away.
        if self.robot_pose is not None:
            current_x = self.robot_pose.pose.pose.position.x
            current_y = self.robot_pose.pose.pose.position.y
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            if self.requires_significant_turning(
                    current_x, current_y, goal_x, goal_y,
                    threshold_degrees=self.significant_turning_threshold_deg):
                quick_path = self.generate_curved_path_for_turning(
                    current_x, current_y, goal_x, goal_y
                )
                self.get_logger().info("⚡ Curved quick-response path published; launching optimization")
            else:
                quick_path = self.generate_quick_response_path(
                    current_x, current_y, goal_x, goal_y
                )
                self.get_logger().info("⚡ Quick-response path published; launching optimization")
            self.publish_path(quick_path)

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
        """Interpolate a path to have exactly target_count evenly-spaced waypoints."""
        if len(path_points) < 2 or target_count <= 1:
            return path_points

        # Build cumulative distance array
        distances = [0.0]
        for i in range(1, len(path_points)):
            seg = math.hypot(
                path_points[i][0] - path_points[i - 1][0],
                path_points[i][1] - path_points[i - 1][1]
            )
            distances.append(distances[-1] + seg)

        total = distances[-1]
        if total < 1e-6:
            return path_points

        result = []
        j = 0
        for k in range(target_count):
            d = k / (target_count - 1) * total
            while j < len(distances) - 1 and distances[j + 1] < d:
                j += 1
            if j >= len(path_points) - 1:
                result.append(path_points[-1])
            else:
                seg_len = distances[j + 1] - distances[j]
                t = (d - distances[j]) / seg_len if seg_len > 1e-6 else 0.0
                x = path_points[j][0] + t * (path_points[j + 1][0] - path_points[j][0])
                y = path_points[j][1] + t * (path_points[j + 1][1] - path_points[j][1])
                result.append((x, y))

        return result

    def smooth_path_for_controller(self, path_points, num_points=30):
        """Interpolate path to num_points evenly-spaced waypoints for smooth controller tracking."""
        return self._interpolate_path_to_count(path_points, num_points)

    def publish_path(self, path_points):
        """Publish path - FIXED to always use actual goal WITH VELOCITY INFO"""
        if path_points is None or len(path_points) == 0:
            return
        
        # CRITICAL FIX 1: Ensure path starts from CURRENT robot position (not stale position)
        if self.robot_pose is not None:
            current_x = self.robot_pose.pose.pose.position.x
            current_y = self.robot_pose.pose.pose.position.y
            start_x, start_y = path_points[0]

            # Check if path start is too far from current position (staleness check)
            start_deviation = math.hypot(start_x - current_x, start_y - current_y)
            if start_deviation > PATH_START_DEVIATION_THRESHOLD:
                # Find the waypoint on the path closest to the current robot position.
                # Trimming from that waypoint avoids shifting the entire path into
                # unknown/potentially unsafe space.
                nearest_idx = 0
                nearest_dist = float('inf')
                for i, (px, py) in enumerate(path_points):
                    d = math.hypot(px - current_x, py - current_y)
                    if d < nearest_dist:
                        nearest_dist = d
                        nearest_idx = i

                # If the nearest waypoint is very far away, the stale path has
                # diverged too much from the robot's current position.  Generate a
                # fresh direct path to the goal rather than trying to re-use the
                # stale trimmed segment.
                if nearest_dist > PATH_REPLAN_NEAREST_WAYPOINT_DIST and self.goal_position:
                    goal_x_fb, goal_y_fb = self.goal_position
                    path_points = self.generate_quick_response_path(
                        current_x, current_y, goal_x_fb, goal_y_fb
                    )
                elif nearest_idx < len(path_points) - 1:
                    # Keep the remaining path from the nearest waypoint onward and
                    # anchor the start to the exact current robot position.
                    # Including path_points[nearest_idx] (not nearest_idx + 1) ensures
                    # the first segment follows the intended path direction instead of
                    # potentially skipping over a waypoint that curves around an obstacle.
                    if nearest_dist < 0.1:
                        # Nearest waypoint is virtually at the robot; skip it to avoid
                        # a near-zero-length duplicate segment.
                        path_points = [(current_x, current_y)] + list(path_points[nearest_idx + 1:])
                    else:
                        path_points = [(current_x, current_y)] + list(path_points[nearest_idx:])
                else:
                    # Robot is at or past the final waypoint.
                    # Reconnect current position directly to the goal endpoint.
                    # CRITICAL FIX 2 below will still enforce the exact goal coordinates.
                    goal_pt = path_points[-1]
                    path_points = [(current_x, current_y), goal_pt]

                if self.debug_mode:
                    self.get_logger().warn(
                        f"⚠️ Path start mismatch: deviation={start_deviation:.3f}m. "
                        f"Trimmed to waypoint {nearest_idx} (nearest dist={nearest_dist:.3f}m), "
                        f"anchored at ({current_x:.3f}, {current_y:.3f})"
                    )
            elif start_deviation > PATH_START_WARNING_THRESHOLD:
                # Minor deviation: snap path[0] to current position without trimming
                path_points = [(current_x, current_y)] + list(path_points[1:])
                if self.debug_mode:
                    self.get_logger().warn(
                        f"Path start deviation: {start_deviation:.3f}m "
                        f"(minor correction applied)"
                    )
        
        # CRITICAL FIX 2: Ensure last point is actual goal
        if self.goal_position and len(path_points) > 0:
            goal_x, goal_y = self.goal_position
            last_x, last_y = path_points[-1]
            if math.hypot(last_x - goal_x, last_y - goal_y) > 0.1:
                path_points[-1] = (goal_x, goal_y)

        # Interpolate to 30 evenly-spaced waypoints for smooth controller tracking
        path_points = self.smooth_path_for_controller(path_points, num_points=30)

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Calculate orientation
            if i < len(path_points) - 1:
                next_x, next_y = path_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                # For last point, use orientation from previous segment
                if i > 0:
                    prev_x, prev_y = path_points[i - 1]
                    yaw = math.atan2(y - prev_y, x - prev_x)
                else:
                    yaw = 0.0

            pose.pose.orientation = self.yaw_to_quaternion(yaw)

            path.poses.append(pose)
        
        self.path_pub.publish(path)
        
        if self.debug_mode:
            path_length = self.calculate_path_length(np.array([[p[0], p[1]] for p in path_points]))
            self.get_logger().debug(f"Published path: {len(path.poses)} waypoints, length={path_length:.2f}m")
            
            # Log start and goal for verification
            start_x, start_y = path_points[0]
            goal_x, goal_y = path_points[-1]
            self.get_logger().debug(f"  Start: ({start_x:.2f}, {start_y:.2f}) → Goal: ({goal_x:.2f}, {goal_y:.2f})")
    
    def yaw_to_quaternion(self, yaw):
        """Yaw to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def quat_to_yaw(self, q):
        """Quaternion to yaw"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


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
