
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

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray, Pose
from sensor_msgs.msg import LaserScan

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
        self.odom_sub = self.create_subscription(Odometry, '/ukf/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan_fixed', self.laser_callback, 10)
        
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
        self.obstacle_timer = self.create_timer(0.15, self.update_obstacle_map)
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
    
    def load_parameters(self):
        """Load parameters from YAML"""
        default_params = {
            'goal_topic': '/goal_pose',
            'population_size': 40,
            'generations': 12,
            'crossover_rate': 0.8,
            'mutation_rate': 0.5,
            'optimization_timeout': 2.0,
            'planning_rate': 1.0,
            'waypoint_count': 8,
            'max_curvature': 2.5,
            'path_smoothing': True,
            'use_pinn_predictions': True,
            'pinn_service_name': '/evaluate_trajectory',
            'pinn_timeout': 1.5,
            'pinn_timeout': 2.0,  # Initial connection timeout
            'pinn_call_timeout': 2.0,  # INCREASED from 0.5 to 5.0 seconds
            'objective_weights': [0.12, 0.08, 0.35, 0.08, 0.12, 0.15, 0.1],
            'obstacle_penalty_weight': 6.0,
            'min_obstacle_distance': 0.4,
            'escape_clearance': 0.7,
            'max_escape_attempts': 3,
            'escape_cooldown': 6.0,
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
        }
        
        # Declare all parameters
        for key, value in default_params.items():
            self.declare_parameter(key, value)
        
        # Store parameters
        self.goal_topic = self.get_parameter('goal_topic').value
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
        """Test PINN connection with a simple path"""
        if not self.use_pinn or not self.pinn_service_available:
            self.get_logger().warn("PINN not enabled or not available")
            return False
        
        # Create a simple test path
        test_path = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        
        self.get_logger().info("🧪 Testing PINN connection...")
        
        try:
            # Use LONGER timeout for test
            original_timeout = self.pinn_call_timeout
            self.pinn_call_timeout = 2.0  # Increase to 2 seconds for test
            
            result = self.call_pinn_service_optimized(test_path)
            
            # Restore original timeout
            self.pinn_call_timeout = original_timeout
            
            if result and result.get('success', False):
                energy = result.get('energy', 0.0)
                stability = result.get('stability', 0.0)
                response_time = result.get('response_time', 0.0)
                
                self.get_logger().info(f"✅ PINN test SUCCESSFUL!")
                self.get_logger().info(f"   Energy: {energy:.2f} J")
                self.get_logger().info(f"   Stability: {stability:.3f}")
                self.get_logger().info(f"   Response time: {response_time:.3f} s")
                
                # Also test if we can get the PINN service output
                self.get_logger().info(f"   Looking for PINN service output like: 'PINN Service: Request #X, Energy=X.XXJ'")
                
                return True
            else:
                if result and result.get('timeout', False):
                    self.get_logger().warn("❌ PINN test TIMEOUT")
                else:
                    self.get_logger().warn("❌ PINN test FAILED")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ PINN test ERROR: {e}")
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
            
            # Use ALL points for better accuracy
            for i in range(len(path_array)):
                x, y = path_array[i]
                xs.append(float(x))
                ys.append(float(y))
                
                # Calculate yaw
                if i < len(path_array) - 1:
                    next_x, next_y = path_array[i + 1]
                    yaw = math.atan2(next_y - y, next_x - x)
                else:
                    yaw = 0.0
                yaws.append(float(yaw))
                velocities.append(0.5)
            
            request.xs = xs
            request.ys = ys
            request.yaws = yaws
            request.velocities = velocities
            
            # DEBUG: Log request details
            if self.debug_mode:
                self.get_logger().debug(f"PINN call #{call_id}: {len(xs)} points")
            
            start_time = time.time()
            
            # Call service with timeout
            future = self.pinn_client.call_async(request)
            timeout_sec = self.pinn_call_timeout
            
            # Wait for response
            wait_start = time.time()
            while not future.done():
                if time.time() - wait_start > timeout_sec:
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
                
                # Process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
            
            # Get response
            response = future.result()
            response_time = time.time() - start_time
            
            if response is not None:
                self.pinn_success_count += 1
                self.pinn_response_times.append(response_time)
                
                # CRITICAL: Log the PINN service output like you want to see
                energy = float(response.energy)
                stability = float(response.stability)
                
                # Calculate average response time
                avg_time = np.mean(self.pinn_response_times) if self.pinn_response_times else 0.0
                success_rate = (self.pinn_success_count / self.pinn_call_count * 100) if self.pinn_call_count > 0 else 0
                
                # Log in the format you want to see
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
        """Determine if we should use a simple straight path - ALWAYS TRUE (curved paths are broken)"""
        # BUG FIX: Always use straight paths - curved path generation goes in wrong direction
        return True
    
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
        
        # STRICTER thresholds for stuck detection
        # Only mark stuck if BOTH conditions are true:
        # 1. Very low movement (< 0.25m in 15 seconds)
        # 2. Small radius (< 0.35m) indicating spinning
        if total_movement < 0.25 and radius < 0.35:
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
        
        self.last_escape_time = current_time
        self.escape_paths_generated += 1
        
        x = self.robot_pose.pose.pose.position.x
        y = self.robot_pose.pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
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

    def generate_backward_escape(self, x, y, goal_x, goal_y):
        """Generate backward escape when surrounded"""
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
        
        self.last_optimization_time = time.time()
        self.optimization_active = True
        
        thread = threading.Thread(target=self.run_enhanced_optimization)
        thread.daemon = True
        thread.start()
    
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
            
            # BUG FIX: Throttle path updates - only regenerate if robot moved significantly
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
                # BUG FIX: Use straight path with intermediate waypoints
                straight_path = self.generate_straight_path_with_waypoints(start_x, start_y, goal_x, goal_y)
                self.publish_path(straight_path)
                self.optimization_active = False
                return
            
            # Get current PINN stats before optimization
            pinn_stats_before = self.objective_evaluator.pinn_usage
            pinn_success_before = self.objective_evaluator.pinn_success
            pinn_failures_before = self.objective_evaluator.pinn_failures
            
            # Also get the node's PINN stats
            node_pinn_calls_before = self.pinn_call_count if hasattr(self, 'pinn_call_count') else 0
            node_pinn_success_before = self.pinn_success_count if hasattr(self, 'pinn_success_count') else 0
            
            if self.stuck_count >= self.max_escape_attempts:
                if self.debug_mode:
                    self.get_logger().warn("🚨 Multiple stuck attempts - using free-space focused path")
                best_path = self.generate_free_space_focused_path(start_x, start_y, goal_x, goal_y)
            else:
                best_path = self.run_enhanced_nsga2_with_free_space(
                    start_x, start_y, goal_x, goal_y,
                    self.population_size,
                    self.generations
                )
            
            if best_path is not None:
                self.publish_path(best_path)
            else:
                fallback = self.generate_quick_response_path(start_x, start_y, goal_x, goal_y)
                self.publish_path(fallback)
            
            elapsed = time.time() - start_time
            
            # Calculate PINN usage during this optimization - FIXED
            pinn_calls_this_optimization = 0
            pinn_success_this_optimization = 0
            
            # Use BOTH sources for accurate tracking
            if hasattr(self.objective_evaluator, 'pinn_usage'):
                pinn_calls_this_optimization = self.objective_evaluator.pinn_usage - pinn_stats_before
                pinn_success_this_optimization = self.objective_evaluator.pinn_success - pinn_success_before
            
            # Also check node's stats
            node_pinn_calls = (self.pinn_call_count - node_pinn_calls_before) if hasattr(self, 'pinn_call_count') else 0
            node_pinn_success = (self.pinn_success_count - node_pinn_success_before) if hasattr(self, 'pinn_success_count') else 0
            
            # Use the larger of the two counts
            if node_pinn_calls > pinn_calls_this_optimization:
                pinn_calls_this_optimization = node_pinn_calls
                pinn_success_this_optimization = node_pinn_success
            
            if self.debug_mode:
                if pinn_calls_this_optimization > 0:
                    pinn_success_rate = (pinn_success_this_optimization / pinn_calls_this_optimization * 100) if pinn_calls_this_optimization > 0 else 0
                    self.get_logger().info(
                        f"✅ Optimization #{self.optimization_count} in {elapsed:.2f}s | "
                        f"PINN: {pinn_calls_this_optimization} calls, {pinn_success_rate:.1f}% success"
                    )
                else:
                    self.get_logger().info(f"✅ Optimization #{self.optimization_count} in {elapsed:.2f}s (no PINN calls)")
        
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
            t = i / (num_waypoints - 1) if num_waypoints > 1 else 1.0
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            path.append((x, y))
        
        return path
    
    def generate_curved_path_for_turning(self, start_x, start_y, goal_x, goal_y):
        """Generate a path with intermediate waypoints for goals requiring turning"""
        path = []
        path.append((start_x, start_y))
        
        # Get robot orientation
        if self.robot_pose is not None:
            robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
        else:
            robot_yaw = 0.0
        
        goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        
        # Add intermediate waypoints along a curve
        # Note: For short distances, uses minimum of 3 waypoints. For longer distances,
        # spacing is approximately 0.5m but varies based on total distance
        num_waypoints = max(3, int(distance / 0.5))  # At least 3 waypoints
        
        for i in range(1, num_waypoints):
            t = i / num_waypoints
            
            # Interpolate angle from robot_yaw to goal_angle
            angle_diff = math.atan2(math.sin(goal_angle - robot_yaw), math.cos(goal_angle - robot_yaw))
            current_angle = robot_yaw + t * angle_diff
            current_distance = t * distance
            
            wp_x = start_x + current_distance * math.cos(current_angle)
            wp_y = start_y + current_distance * math.sin(current_angle)
            
            path.append((wp_x, wp_y))
        
        path.append((goal_x, goal_y))
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
            
            # BUG FIX: Throttle path updates - only regenerate if robot moved significantly
            if not self.should_update_path(start_x, start_y):
                return
            
            distance_to_goal = math.hypot(goal_x - start_x, goal_y - start_y)
            if distance_to_goal < self.goal_completion_distance:
                self.get_logger().info(f"🎯 Already at goal ({distance_to_goal:.2f}m)")
                self.optimization_active = False
                return
            
            # NEW: Check if we should use simple straight path
            if self.should_use_simple_straight_path(start_x, start_y, goal_x, goal_y):
                if self.debug_mode:
                    self.get_logger().info("📏 Using simple straight path (clear path detected)")
                # BUG FIX: Use straight path with intermediate waypoints
                straight_path = self.generate_straight_path_with_waypoints(start_x, start_y, goal_x, goal_y)
                self.publish_path(straight_path)
                self.optimization_active = False
                return
            
            # NEW: Check if goal requires significant turning - use curved path
            if self.requires_significant_turning(start_x, start_y, goal_x, goal_y, threshold_degrees=30):
                # Calculate angle for logging
                robot_yaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
                goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)
                angle_diff = abs(math.atan2(math.sin(goal_angle - robot_yaw), math.cos(goal_angle - robot_yaw)))
                
                if self.debug_mode:
                    self.get_logger().info(f"🔄 Using curved path for turning (angle={math.degrees(angle_diff):.1f}°)")
                curved_path = self.generate_curved_path_for_turning(start_x, start_y, goal_x, goal_y)
                self.publish_path(curved_path)
                self.optimization_active = False
                return
            
            # Get current PINN stats before optimization
            pinn_stats_before = self.objective_evaluator.pinn_usage
            pinn_success_before = self.objective_evaluator.pinn_success
            
            if self.stuck_count >= self.max_escape_attempts:
                if self.debug_mode:
                    self.get_logger().warn("🚨 Multiple stuck attempts - using free-space focused path")
                best_path = self.generate_free_space_focused_path(start_x, start_y, goal_x, goal_y)
            else:
                best_path = self.run_enhanced_nsga2_with_free_space(
                    start_x, start_y, goal_x, goal_y,
                    self.population_size,
                    self.generations
                )
            
            if best_path is not None:
                self.publish_path(best_path)
            else:
                fallback = self.generate_quick_response_path(start_x, start_y, goal_x, goal_y)
                self.publish_path(fallback)
            
            elapsed = time.time() - start_time
            
            # Calculate PINN usage during this optimization - FIXED
            pinn_calls_this_optimization = self.objective_evaluator.pinn_usage - pinn_stats_before
            pinn_success_this_optimization = self.objective_evaluator.pinn_success - pinn_success_before
            
            if self.debug_mode:
                if pinn_calls_this_optimization > 0:
                    pinn_success_rate = (pinn_success_this_optimization / pinn_calls_this_optimization * 100) if pinn_calls_this_optimization > 0 else 0
                    self.get_logger().info(
                        f"✅ Optimization #{self.optimization_count} in {elapsed:.2f}s | "
                        f"PINN: {pinn_calls_this_optimization} calls, {pinn_success_rate:.1f}% success"
                    )
                else:
                    self.get_logger().info(f"✅ Optimization #{self.optimization_count} in {elapsed:.2f}s (no PINN calls)")
        
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
    def run_enhanced_nsga2_with_free_space(self, start_x, start_y, goal_x, goal_y, pop_size, generations):
        """NSGA-II with free space - USING YOUR NSGA2 MODULE - FIXED PINN calls"""
        population = self.initialize_enhanced_population(
            start_x, start_y, goal_x, goal_y, pop_size
        )
        
        straight_line_length = math.hypot(goal_x - start_x, goal_y - start_y)
        
        pinn_calls_this_generation = 0
        pinn_success_this_generation = 0
        
        for gen in range(generations):
            all_objectives = []
            valid_individuals = []
            
            for idx, individual in enumerate(population):
                path_array = np.array([[p[0], p[1]] for p in individual])
                
                obstacle_cost = self.get_enhanced_obstacle_cost(path_array)
                free_space_bonus = self.get_free_space_bonus(path_array)
                
                if obstacle_cost > self.obstacle_penalty_weight * 15:
                    continue
                
                # Calculate all required metrics
                path_length = self.calculate_path_length(path_array)
                curvature_cost = self.calculate_curvature(path_array)
                
                # FIXED: Calculate uncertainty_cost before using it
                uncertainty_cost = self.uncertainty_planner.get_uncertainty_cost(path_array)
                
                deviation = abs(path_length - straight_line_length) / max(straight_line_length, 0.1)
                
                # FIXED: Only call PINN for promising paths
                use_pinn_for_this = False
                
                # Conditions for using PINN:
                if (self.use_pinn and self.pinn_service_available and 
                    obstacle_cost < self.obstacle_penalty_weight * 8 and
                    pinn_calls_this_generation < 12):
                    
                    # Use PINN for 50% of eligible paths
                    if random.random() < 0.5:
                        use_pinn_for_this = True
                
                # Use the FIXED objective evaluator
                objectives = self.objective_evaluator.evaluate_all_objectives_with_timeout(
                    path_array,
                    obstacle_cost - free_space_bonus,
                    uncertainty_cost,  # Now this variable is defined
                    straight_line_length,
                    use_pinn=use_pinn_for_this,
                    timeout=0.5
                )
                
                # Track PINN usage
                if use_pinn_for_this:
                    pinn_calls_this_generation += 1
                    # Check if PINN provided valid results
                    if objectives[5] > 0.0 and objectives[5] < 1000.0:
                        pinn_success_this_generation += 1
                        if self.debug_mode:
                            self.get_logger().debug(f"PINN successful for path {idx}: energy={objectives[5]:.2f}")
                    else:
                        if self.debug_mode:
                            self.get_logger().debug(f"PINN failed for path {idx}")
                
                all_objectives.append(objectives)
                valid_individuals.append(idx)
            
            # After each generation, log PINN stats
            if pinn_calls_this_generation > 0 and self.debug_mode:
                success_rate = (pinn_success_this_generation / pinn_calls_this_generation * 100) if pinn_calls_this_generation > 0 else 0
                self.get_logger().info(
                    f"📈 PINN Stats for gen {gen}: {pinn_calls_this_generation} calls, "
                    f"{pinn_success_this_generation} successful ({success_rate:.1f}%)"
                )
            
            if not all_objectives:
                population = self.initialize_enhanced_population(
                    start_x, start_y, goal_x, goal_y, pop_size
                )
                continue
            
            # Selection and reproduction using YOUR NSGA2 module
            if len(all_objectives) >= 3:
                try:
                    # Use your nsga2_selection function
                    selected_indices = nsga2_selection(
                        [population[i] for i in valid_individuals],
                        all_objectives,
                        pop_size
                    )
                    
                    # Map back to original indices
                    selected_original_indices = [valid_individuals[i] for i in selected_indices]
                    
                    new_population = []
                    for idx in selected_original_indices:
                        if idx < len(population):
                            new_population.append(population[idx])
                    
                    while len(new_population) < pop_size:
                        if random.random() < self.crossover_rate and len(new_population) >= 2:
                            parent1 = random.choice(new_population)
                            parent2 = random.choice(new_population)
                            child = self.crossover_paths(parent1, parent2)
                        else:
                            child = random.choice(new_population)
                        
                        child = self.mutate_with_free_space(child, start_x, start_y, goal_x, goal_y)
                        new_population.append(child)
                    
                    population = new_population
                except Exception as e:
                    if self.debug_mode:
                        self.get_logger().warn(f"NSGA-II failed: {e}, using enhanced selection")
                    population = self.enhanced_selection(population, valid_individuals, all_objectives, pop_size)
            else:
                population = self.enhanced_selection(population, valid_individuals, all_objectives, pop_size)
            
            # Early exit if timeout
            if time.time() - self.last_optimization_time > self.optimization_timeout:
                if self.debug_mode:
                    self.get_logger().debug(f"Optimization timeout after {gen+1} generations")
                break
        
        # Log PINN usage for this optimization
        if pinn_calls_this_generation > 0 and self.debug_mode:
            success_rate = (pinn_success_this_generation / pinn_calls_this_generation * 100) if pinn_calls_this_generation > 0 else 0
            self.get_logger().info(
                f"📈 PINN Stats: {pinn_calls_this_generation} calls, "
                f"{pinn_success_this_generation} successful ({success_rate:.1f}%)"
            )
        
        # Select best individual
        if population:
            best_idx = 0
            best_score = float('inf')
            best_pinn_energy = 0.0
            best_pinn_stability = 0.0
            
            for idx, individual in enumerate(population):
                path_array = np.array([[p[0], p[1]] for p in individual])
                obstacle_cost = self.get_enhanced_obstacle_cost(path_array)
                free_space_bonus = self.get_free_space_bonus(path_array)
                path_length = self.calculate_path_length(path_array)
                min_clearance = self.get_min_clearance(path_array)
                
                # Enhanced scoring with free space
                clearance_factor = max(0.1, min_clearance / self.min_obstacle_distance)
                free_space_factor = 1.0 + free_space_bonus
                score = (path_length + obstacle_cost * 2.5 / clearance_factor) / free_space_factor
                
                if score < best_score:
                    best_score = score
                    best_idx = idx
            
            best_path = population[best_idx]
            
            # Evaluate best path with PINN for logging (non-blocking)
            best_path_array = np.array([[p[0], p[1]] for p in best_path])
            if self.use_pinn and self.pinn_service_available:
                try:
                    # Use a thread to avoid blocking
                    pinn_future = self.thread_pool.submit(self.call_pinn_service_optimized, best_path_array)
                    
                    # Try to get result quickly, but don't wait too long
                    try:
                        pinn_result = pinn_future.result(timeout=0.5)
                        if pinn_result:
                            best_pinn_energy = pinn_result.get('energy', 0.0)
                            best_pinn_stability = pinn_result.get('stability', 0.0)
                    except TimeoutError:
                        if self.debug_mode:
                            self.get_logger().debug("PINN evaluation for best path timed out")
                except Exception as e:
                    if self.debug_mode:
                        self.get_logger().debug(f"PINN evaluation for best path failed: {e}")
            
            if self.debug_mode:
                path_array = np.array([[p[0], p[1]] for p in best_path])
                obstacle_cost = self.get_enhanced_obstacle_cost(path_array)
                free_space_bonus = self.get_free_space_bonus(path_array)
                path_length = self.calculate_path_length(path_array)
                min_clearance = self.get_min_clearance(path_array)
                
                log_message = (
                    f"🏆 Best path: Length={path_length:.2f}m, "
                    f"ObstacleCost={obstacle_cost:.2f}, "
                    f"FreeSpaceBonus={free_space_bonus:.2f}, "
                    f"Clearance={min_clearance:.2f}m"
                )
                
                if best_pinn_energy > 0:
                    log_message += f", PINN Energy={best_pinn_energy:.1f}J, Stability={best_pinn_stability:.3f}"
                
                self.get_logger().info(log_message)
            
            # Smooth path
            if self.path_smoothing:
                best_path = self.smooth_path_with_free_space(best_path)
            
            return best_path
        
        return None
    
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
    
    def initialize_enhanced_population(self, start_x, start_y, goal_x, goal_y, pop_size):
        """Initialize enhanced population"""
        population = []
        
        # Always include a straight line path
        straight_path = [(start_x, start_y), (goal_x, goal_y)]
        population.append(straight_path)
        
        # Check if path is mostly clear
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        is_clear_path = self.is_straight_path_clear(start_x, start_y, goal_x, goal_y, min(distance, 3.0))
        
        # Add more straight paths if clear
        if is_clear_path:
            for i in range(min(3, pop_size - 1)):
                population.append(straight_path)
        
        free_space_path = self.generate_free_space_focused_path(start_x, start_y, goal_x, goal_y)
        if free_space_path:
            population.append(free_space_path)
        
        quick_path = self.generate_quick_response_path(start_x, start_y, goal_x, goal_y)
        if quick_path:
            population.append(quick_path)
        
        # Fill the rest with variations, but reduce exploration for clear paths
        remaining = pop_size - len(population)
        
        for i in range(remaining):
            path = []
            path.append((start_x, start_y))
            
            # Reduce exploration for clear paths
            if is_clear_path:
                # Mostly straight paths with minor deviations
                num_waypoints = 2  # Just start and goal for clear paths
                path.append((goal_x, goal_y))
            else:
                num_waypoints = random.randint(2, min(4, self.waypoint_count))
                
                for j in range(1, num_waypoints - 1):
                    t = j / (num_waypoints - 1)
                    
                    base_x = start_x + t * (goal_x - start_x)
                    base_y = start_y + t * (goal_y - start_y)
                    
                    if random.random() < self.exploration_factor * 0.5:  # Reduce exploration
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
                new_population.append(population[best_idx])
                
                if random.random() < self.mutation_rate:
                    mutated = self.mutate_with_free_space(
                        new_population[-1],
                        population[best_idx][0][0],
                        population[best_idx][0][1],
                        population[best_idx][-1][0],
                        population[best_idx][-1][1]
                    )
                    new_population[-1] = mutated
            else:
                if valid_indices:
                    idx = random.choice(valid_indices)
                    if idx < len(population):
                        new_population.append(population[idx])
        
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
        
        if self.scan_angles is None:
            self.scan_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(len(msg.ranges))
            ]
    
    def update_obstacle_map(self):
        """Update obstacle map"""
        if self.robot_pose is None or self.laser_scan is None:
            return
        
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        ryaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
        
        self.obstacle_grid *= 0.8
        
        for i, range_val in enumerate(self.laser_scan):
            if 0.1 < range_val < 10.0:
                angle = self.scan_angles[i]
                
                local_x = range_val * math.cos(angle)
                local_y = range_val * math.sin(angle)
                
                world_x = rx + local_x * math.cos(ryaw) - local_y * math.sin(ryaw)
                world_y = ry + local_x * math.sin(ryaw) + local_y * math.cos(ryaw)
                
                self.update_obstacle_cells(world_x, world_y, range_val)
    
    def update_obstacle_cells(self, x, y, range_val):
        """Update obstacle cells"""
        grid_size = self.obstacle_grid_size
        resolution = self.obstacle_grid_resolution
        
        if self.obstacle_grid_origin is None:
            self.obstacle_grid_origin = (x, y)
            ox, oy = 0, 0
        else:
            ox, oy = self.obstacle_grid_origin
        
        grid_x = int((x - ox) / resolution) + grid_size // 2
        grid_y = int((y - oy) / resolution) + grid_size // 2
        
        obstacle_radius_cells = max(2, int(self.robot_radius / resolution))
        
        for dx in range(-obstacle_radius_cells, obstacle_radius_cells + 1):
            for dy in range(-obstacle_radius_cells, obstacle_radius_cells + 1):
                gx = grid_x + dx
                gy = grid_y + dy
                
                if 0 <= gx < grid_size and 0 <= gy < grid_size:
                    dist = math.hypot(dx, dy) * resolution
                    if dist < self.robot_radius * 1.5:
                        value = 1.2 - (dist / (self.robot_radius * 1.5))
                        value = min(value, 1.0)
                        if value > self.obstacle_grid[gx, gy]:
                            self.obstacle_grid[gx, gy] = value
    
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
        """World to grid"""
        if self.obstacle_grid_origin is None:
            return None, None
        
        ox, oy = self.obstacle_grid_origin
        grid_x = int((x - ox) / self.obstacle_grid_resolution) + self.obstacle_grid_size // 2
        grid_y = int((y - oy) / self.obstacle_grid_resolution) + self.obstacle_grid_size // 2
        
        if 0 <= grid_x < self.obstacle_grid_size and 0 <= grid_y < self.obstacle_grid_size:
            return grid_x, grid_y
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
        
        if self.debug_mode:
            self.get_logger().info(f"🎯 New goal: ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f})")
        
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
                if self.debug_mode:
                    self.get_logger().warn(
                        f"⚠️ Path start mismatch detected: deviation={start_deviation:.3f}m. "
                        f"Correcting path[0] from ({start_x:.3f}, {start_y:.3f}) to ({current_x:.3f}, {current_y:.3f})"
                    )
                # Force path to start from current position
                path_points[0] = (current_x, current_y)
            elif start_deviation > PATH_START_WARNING_THRESHOLD:
                # Log warning for moderate deviations without correction
                if self.debug_mode:
                    self.get_logger().warn(
                        f"Path start deviation: {start_deviation:.3f}m "
                        f"(within tolerance, no correction needed)"
                    )
        
        # CRITICAL FIX 2: Ensure last point is actual goal
        if self.goal_position and len(path_points) > 0:
            goal_x, goal_y = self.goal_position
            last_x, last_y = path_points[-1]
            if math.hypot(last_x - goal_x, last_y - goal_y) > 0.1:
                path_points[-1] = (goal_x, goal_y)
        
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        
        total_length = self.calculate_path_length(np.array([[p[0], p[1]] for p in path_points]))
        
        for i, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            
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
            
            # Add velocity information in pose (store in orientation if needed, or use covariance)
            # We'll add a small marker in the pose for velocity
            if i == 0:
                # Start with moderate speed
                pose.pose.position.z = 0.8  # Use z for velocity indicator (0.8 m/s)
            elif i == len(path_points) - 1:
                # Slow down at goal
                pose.pose.position.z = 0.1
            else:
                # Calculate speed based on straightness
                if i < len(path_points) - 1:
                    segment_length = math.hypot(path_points[i+1][0] - x, path_points[i+1][1] - y)
                    if total_length > 0:
                        # Higher speed for straighter paths
                        curvature = 0.0
                        if i > 0 and i < len(path_points) - 1:
                            # Simple curvature calculation
                            p1 = path_points[i-1]
                            p2 = path_points[i]
                            p3 = path_points[i+1]
                            v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
                            v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
                            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                                v1_norm = v1 / np.linalg.norm(v1)
                                v2_norm = v2 / np.linalg.norm(v2)
                                curvature = math.acos(np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0))
                        
                        # Higher speed for lower curvature
                        speed = 1.2 - min(curvature, 1.0)  # 1.2 m/s max, reduce with curvature
                        pose.pose.position.z = max(0.3, min(speed, 1.2))  # Clamp between 0.3 and 1.2 m/s
            
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
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

