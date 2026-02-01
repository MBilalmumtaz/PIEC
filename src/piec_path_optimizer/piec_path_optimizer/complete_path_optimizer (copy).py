#!/usr/bin/env python3
"""
Enhanced NSGA-II Path Optimizer with Obstacle Awareness
Includes laser scan data for obstacle-aware path optimization
"""
import rclpy
from rclpy.node import Node
import numpy as np
import math
import random
import time
import yaml
import os
import threading
from ament_index_python.packages import get_package_share_directory
from concurrent.futures import ThreadPoolExecutor, as_completed

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan

# Import local modules
from piec_path_optimizer.objectives import ObjectiveEvaluator
from piec_path_optimizer.uncertainty_aware import UncertaintyAwarePlanner
from piec_path_optimizer.nsga2 import fast_non_dominated_sort, crowding_distance

class CompletePathOptimizer(Node):
    def __init__(self):
        super().__init__('enhanced_nsga2_path_optimizer')
        
        # Load parameters from YAML
        self.load_parameters_from_yaml()
        
        # State variables
        self.robot_pose = None
        self.goal_pose = None
        self.current_ukf = None
        self.laser_scan = None
        self.scan_angles = None
        self.optimization_active = False
        self.optimization_count = 0
        self.last_optimization_time = 0.0
        self.last_robot_position = None
        
        # Obstacle map for global planning
        self.obstacle_grid = None
        self.obstacle_grid_resolution = self.get_parameter('obstacle_grid_resolution').value
        self.obstacle_grid_size = self.get_parameter('obstacle_grid_size').value
        self.initialize_obstacle_grid()
        
        # Initialize components
        self.objective_evaluator = ObjectiveEvaluator(self)
        self.uncertainty_planner = UncertaintyAwarePlanner(
            self, 
            grid_size=self.grid_size,
            resolution=self.resolution
        )
        
        # Thread pool for parallel evaluation
        self.max_workers = self.get_parameter('max_workers').value
        self.thread_pool = ThreadPoolExecutor(max_workers=self.max_workers)
        
        # Subscribers
        self.create_subscription(Odometry, '/ukf/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        
        # FIXED: Use correct QoS for laser subscription - Subscribe to /scan_fixed from laser_bridge
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        
        # Define QoS profile matching laser_bridge publisher (RELIABLE)
        laser_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,  # Matches laser_bridge publisher
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to the processed scan from laser_bridge
        self.create_subscription(LaserScan, '/scan_fixed', self.laser_callback, laser_qos)
        
        # Publisher for optimized path
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, '/piec/path', qos_profile)
        
        # Optional: Publisher for obstacle visualization
        self.declare_parameter('visualize_obstacles', False)
        if self.get_parameter('visualize_obstacles').value:
            from visualization_msgs.msg import MarkerArray, Marker
            self.obstacle_pub = self.create_publisher(MarkerArray, '/optimizer/obstacles', 10)
        
        # Timer for periodic optimization
        self.create_timer(self.planning_rate, self.optimization_timer_callback)
        
        # Timer for obstacle map maintenance
        self.create_timer(1.0, self.update_obstacle_map)
        
        self.get_logger().info("🚀 Enhanced NSGA-II Path Optimizer initialized")
        self.get_logger().info(f"Population: {self.population_size}, Generations: {self.generations}")
        self.get_logger().info(f"Obstacle grid: {self.obstacle_grid_size}x{self.obstacle_grid_size}, "
                             f"res: {self.obstacle_grid_resolution}m")
    
    def load_parameters_from_yaml(self):
        """Load parameters from YAML configuration file"""
        # Try to load from YAML first
        config_path = os.path.join(
            get_package_share_directory('piec_path_optimizer'),
            'config',
            'optimizer_params.yaml'
        )
        #before 'population_size': 30,
            #'generations': 15,
        # Default parameters
        params = {
            # Optimization Parameters
            'goal_topic': '/goal_pose',
            'population_size': 10,
            'generations': 5,
            'crossover_rate': 0.8,
            'mutation_rate': 0.2,
            'optimization_timeout': 2.0,
            'planning_rate': 1.0,
            
            # Path Generation
            'waypoint_count': 8,
            'max_curvature': 0.3,
            'path_smoothing': True,
            'adaptive_sampling': True,
            
            # NSGA-II Parameters
            'nsga_elite_count': 5,
            'nsga_tournament_size': 3,
            'nsga_mutation_power': 0.1,
            
            # PINN Integration
            'use_pinn_predictions': True,
            'pinn_timeout': 1.0,
            'pinn_retry_count': 3,
            
            # Uncertainty Parameters
            'uncertainty_weight': 0.3,
            'update_uncertainty_map': True,
            'uncertainty_decay': 0.9,
            'grid_size': 100,
            'resolution': 0.1,
            
            # Obstacle Parameters
            'obstacle_penalty_weight': 2.0,
            'min_obstacle_distance': 0.5,
            'obstacle_grid_size': 50,
            'obstacle_grid_resolution': 0.2,
            'obstacle_decay_rate': 0.95,
            
            # Performance
            'use_multithreading': True,
            'max_workers': 4,
            
            # Debug
            'debug_mode': True,
            'log_fitness': False,
            'visualize_paths': False,
        }
        
        # Load from YAML if file exists
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as file:
                    yaml_params = yaml.safe_load(file)
                    if yaml_params and 'ros__parameters' in yaml_params:
                        params.update(yaml_params['ros__parameters'])
                    elif yaml_params and '/**' in yaml_params and 'ros__parameters' in yaml_params['/**']:
                        params.update(yaml_params['/**']['ros__parameters'])
                self.get_logger().info(f"✅ Loaded parameters from: {config_path}")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to load YAML: {e}, using defaults")
        
        # Declare all parameters
        for key, value in params.items():
            self.declare_parameter(key, value)
        
        # Store as instance variables
        self.goal_topic = self.get_parameter('goal_topic').value
        self.population_size = self.get_parameter('population_size').value
        self.generations = self.get_parameter('generations').value
        self.crossover_rate = self.get_parameter('crossover_rate').value
        self.mutation_rate = self.get_parameter('mutation_rate').value
        self.optimization_timeout = self.get_parameter('optimization_timeout').value
        self.planning_rate = self.get_parameter('planning_rate').value
        self.waypoint_count = self.get_parameter('waypoint_count').value
        self.max_curvature = self.get_parameter('max_curvature').value
        self.use_pinn = self.get_parameter('use_pinn_predictions').value
        self.grid_size = self.get_parameter('grid_size').value
        self.resolution = self.get_parameter('resolution').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.obstacle_penalty_weight = self.get_parameter('obstacle_penalty_weight').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
    
    def initialize_obstacle_grid(self):
        """Initialize obstacle grid"""
        size = self.get_parameter('obstacle_grid_size').value
        self.obstacle_grid = np.zeros((size, size), dtype=np.float32)
        self.obstacle_grid_origin = None  # Will be set to robot position
    
    def odom_callback(self, msg: Odometry):
        """Update robot position and uncertainty"""
        self.robot_pose = msg
        self.uncertainty_planner.update_uncertainty_map(msg)
        
        # Update obstacle grid origin if not set
        if self.obstacle_grid_origin is None:
            self.obstacle_grid_origin = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )
    
    def goal_callback(self, msg: PoseStamped):
        """Handle new goal"""
        self.goal_pose = msg
        
        if self.debug_mode:
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            self.get_logger().info(f"🎯 New goal received: ({goal_x:.2f}, {goal_y:.2f})")
        
        # Trigger optimization if conditions are met
        current_time = time.time()
        if (self.robot_pose is not None and 
            current_time - self.last_optimization_time > 1.0 and
            not self.optimization_active):
            
            self.trigger_optimization()
    
    def laser_callback(self, msg: LaserScan):
        """Process laser scan for obstacle mapping"""
        self.laser_scan = msg.ranges
        
        # Initialize scan angles if needed
        if self.scan_angles is None or len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(len(msg.ranges))
            ]
        
        # Update obstacle grid
        if self.robot_pose is not None:
            self.update_obstacle_grid_from_scan(msg)
    
    def update_obstacle_grid_from_scan(self, scan_msg: LaserScan):
        """Update obstacle grid from laser scan"""
        if self.robot_pose is None or self.obstacle_grid_origin is None:
            return
        
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        ryaw = self.quat_to_yaw(self.robot_pose.pose.pose.orientation)
        
        for i, range_val in enumerate(scan_msg.ranges):
            if 0.1 < range_val < 5.0:  # Valid range
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                
                # Convert to world coordinates
                local_x = range_val * math.cos(angle)
                local_y = range_val * math.sin(angle)
                
                world_x = rx + local_x * math.cos(ryaw) - local_y * math.sin(ryaw)
                world_y = ry + local_x * math.sin(ryaw) + local_y * math.cos(ryaw)
                
                # Convert to grid coordinates
                grid_x = int((world_x - self.obstacle_grid_origin[0]) / self.obstacle_grid_resolution) + self.obstacle_grid.shape[0] // 2
                grid_y = int((world_y - self.obstacle_grid_origin[1]) / self.obstacle_grid_resolution) + self.obstacle_grid.shape[1] // 2
                
                # Update grid if within bounds
                if 0 <= grid_x < self.obstacle_grid.shape[0] and 0 <= grid_y < self.obstacle_grid.shape[1]:
                    self.obstacle_grid[grid_x, grid_y] = 1.0  # Mark as occupied
    
    def update_obstacle_map(self):
        """Maintain obstacle map (decay old obstacles)"""
        decay_rate = self.get_parameter('obstacle_decay_rate').value
        self.obstacle_grid *= decay_rate
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.obstacle_grid_origin is None:
            return None, None
        
        grid_x = int((x - self.obstacle_grid_origin[0]) / self.obstacle_grid_resolution) + self.obstacle_grid.shape[0] // 2
        grid_y = int((y - self.obstacle_grid_origin[1]) / self.obstacle_grid_resolution) + self.obstacle_grid.shape[1] // 2
        
        return grid_x, grid_y
    
    def get_obstacle_cost(self, path_array):
        """Calculate obstacle cost for a path"""
        if self.obstacle_grid is None or len(path_array) < 2:
            return 0.0
        
        total_cost = 0.0
        points_checked = 0
        
        for i in range(len(path_array) - 1):
            # Sample points along segment
            p1 = path_array[i]
            p2 = path_array[i + 1]
            segment_length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            
            if segment_length < 1e-6:
                continue
            
            # Sample points along segment
            num_samples = max(2, int(segment_length / self.obstacle_grid_resolution))
            for j in range(num_samples):
                t = j / (num_samples - 1) if num_samples > 1 else 0.5
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                
                # Convert to grid
                grid_x, grid_y = self.world_to_grid(x, y)
                if grid_x is None:
                    continue
                
                # Check if within bounds
                if (0 <= grid_x < self.obstacle_grid.shape[0] and 
                    0 <= grid_y < self.obstacle_grid.shape[1]):
                    
                    # Get obstacle value (0 = free, 1 = occupied)
                    obstacle_value = self.obstacle_grid[grid_x, grid_y]
                    
                    # Add cost proportional to obstacle presence
                    if obstacle_value > 0.1:
                        # Higher cost for closer obstacles
                        distance_to_obstacle = max(0.1, 1.0 - obstacle_value)
                        cost = self.obstacle_penalty_weight / distance_to_obstacle
                        total_cost += cost
                    
                    points_checked += 1
        
        # Normalize by number of points checked
        if points_checked > 0:
            return total_cost / points_checked
        return 0.0
    
    def trigger_optimization(self):
        """Start optimization"""
        if self.optimization_active:
            return
            
        self.optimization_active = True
        self.last_optimization_time = time.time()
        
        # Run optimization in background thread
        thread = threading.Thread(target=self.run_optimization_thread)
        thread.daemon = True
        thread.start()
    
    def run_optimization_thread(self):
        """Main optimization thread with obstacle awareness"""
        try:
            start_time = time.time()
            self.optimization_count += 1
            
            # Extract start and goal positions
            start_x = self.robot_pose.pose.pose.position.x
            start_y = self.robot_pose.pose.pose.position.y
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            
            # Calculate distance for adaptive parameters
            distance = math.hypot(goal_x - start_x, goal_y - start_y)
            
            # Adaptive parameters based on distance and obstacles
            obstacle_density = self.get_area_obstacle_density(start_x, start_y, goal_x, goal_y)
            
            if distance < 2.0 or obstacle_density > 0.7:
                # Short distance or high obstacles - faster optimization
                actual_generations = max(3, self.generations // 3)
                actual_population = max(10, self.population_size // 3)
            elif distance < 5.0 or obstacle_density > 0.3:
                # Medium distance or moderate obstacles
                actual_generations = max(5, self.generations // 2)
                actual_population = max(15, self.population_size // 2)
            else:
                # Long distance with few obstacles - full optimization
                actual_generations = self.generations
                actual_population = self.population_size
            
            if self.debug_mode:
                self.get_logger().info(f"Optimization {self.optimization_count}: "
                                     f"distance={distance:.2f}m, "
                                     f"obstacle_density={obstacle_density:.2f}, "
                                     f"gens={actual_generations}, pop={actual_population}")
            
            # Get UKF covariance for uncertainty
            ukf_cov = self.extract_ukf_covariance()
            
            # Initialize population with obstacle-aware heuristic
            population = self.initialize_obstacle_aware_population(
                start_x, start_y, goal_x, goal_y, actual_population, obstacle_density
            )
            
            # Evolution loop with early termination
            best_solution = None
            best_fitness = float('inf')
            generation_times = []
            
            for gen in range(actual_generations):
                gen_start_time = time.time()
                
                # Check timeout
                if time.time() - start_time > self.optimization_timeout:
                    if gen > 0 and self.debug_mode:
                        self.get_logger().debug(f"⏰ Optimization timeout at generation {gen}")
                    break
                
                # Evaluate all objectives for each individual (parallel if enabled)
                if self.get_parameter('use_multithreading').value and actual_population > 10:
                    all_objectives = self.evaluate_population_parallel(population, ukf_cov, distance)
                else:
                    all_objectives = []
                    for individual in population:
                        path_array = np.array([[p[0], p[1]] for p in individual])
                        objectives = self.evaluate_all_objectives(path_array, ukf_cov, distance)
                        all_objectives.append(objectives)
                
                # Fast non-dominated sort
                fronts = fast_non_dominated_sort(all_objectives)
                
                # Calculate crowding distance for each front
                front_individuals = []
                for front in fronts:
                    # Get crowding distances for this front
                    distances = crowding_distance(front, all_objectives)
                    
                    # Sort by crowding distance (descending)
                    front_sorted = sorted(front, key=lambda i: distances[i], reverse=True)
                    
                    # Add individuals from this front
                    for idx in front_sorted:
                        front_individuals.append(population[idx])
                
                # Create next generation using NSGA-II operators
                new_population = []
                
                # Keep best individuals from fronts
                keep_count = min(len(front_individuals), actual_population // 2)
                new_population.extend(front_individuals[:keep_count])
                
                # Generate offspring with obstacle awareness
                while len(new_population) < actual_population:
                    # Binary tournament selection from parent population
                    parent1 = self.binary_tournament_selection(population, all_objectives)
                    parent2 = self.binary_tournament_selection(population, all_objectives)
                    
                    # Crossover
                    if random.random() < self.crossover_rate:
                        child = self.obstacle_aware_crossover(parent1, parent2, obstacle_density)
                    else:
                        child = parent1 if random.random() < 0.5 else parent2
                    
                    # Polynomial mutation with obstacle awareness
                    if random.random() < self.mutation_rate:
                        child = self.obstacle_aware_mutation(child, obstacle_density)
                    
                    # Repair and validate with obstacle avoidance
                    child = self.repair_path_with_obstacles(child, start_x, start_y, goal_x, goal_y)
                    
                    new_population.append(child)
                
                # Update population
                population = new_population[:actual_population]
                
                # Find best solution (minimum of combined objectives)
                if len(population) > 0:
                    best_idx = 0
                    best_combined_score = float('inf')
                    for i, individual in enumerate(population):
                        path_array = np.array([[p[0], p[1]] for p in individual])
                        objectives = all_objectives[i]
                        # Combine objectives with weights
                        combined_score = (
                            objectives[0] +  # Path length
                            objectives[1] * 0.5 +  # Curvature
                            objectives[2] * 2.0 +  # Obstacle cost
                            objectives[3] * 0.3  # Deviation
                        )
                        if combined_score < best_combined_score:
                            best_combined_score = combined_score
                            best_idx = i
                    
                    best_solution = population[best_idx]
                    best_fitness = best_combined_score
                
                generation_time = time.time() - gen_start_time
                generation_times.append(generation_time)
                
                # Log progress occasionally
                if self.debug_mode and (gen % 5 == 0 or gen == actual_generations - 1):
                    avg_gen_time = np.mean(generation_times) if generation_times else 0
                    self.get_logger().debug(
                        f"Gen {gen}: Best score = {best_fitness:.4f}, "
                        f"Population = {len(population)}, "
                        f"Time/gen = {avg_gen_time:.3f}s"
                    )
            
            # Use best solution or generate fallback
            if best_solution is not None:
                # Apply uncertainty-aware smoothing if enabled
                if self.get_parameter('path_smoothing').value:
                    best_solution = self.smooth_path_with_obstacles(best_solution)
                
                self.publish_path(best_solution)
                elapsed = time.time() - start_time
                
                if self.debug_mode:
                    self.get_logger().info(
                        f"✅ Optimization {self.optimization_count} completed in {elapsed:.2f}s, "
                        f"{len(generation_times)} generations"
                    )
                    path_array = np.array([[p[0], p[1]] for p in best_solution])
                    path_length = self.calculate_path_length(path_array)
                    obstacle_cost = self.get_obstacle_cost(path_array)
                    self.get_logger().info(
                        f"Best path: length={path_length:.2f}m, "
                        f"obstacle_cost={obstacle_cost:.3f}, "
                        f"distance={distance:.2f}m"
                    )
            else:
                self.generate_obstacle_aware_fallback_path(start_x, start_y, goal_x, goal_y)
                if self.debug_mode:
                    self.get_logger().info("🔄 Using obstacle-aware fallback path")
                
        except Exception as e:
            self.get_logger().error(f"❌ Optimization error: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
            # Generate fallback path
            if self.robot_pose is not None and self.goal_pose is not None:
                self.generate_obstacle_aware_fallback_path(
                    self.robot_pose.pose.pose.position.x,
                    self.robot_pose.pose.pose.position.y,
                    self.goal_pose.pose.position.x,
                    self.goal_pose.pose.position.y
                )
        finally:
            self.optimization_active = False
    
    def evaluate_population_parallel(self, population, ukf_cov, total_distance):
        """Evaluate population in parallel"""
        all_objectives = [None] * len(population)
        
        # Create tasks
        futures = []
        for i, individual in enumerate(population):
            future = self.thread_pool.submit(
                self.evaluate_individual_objectives,
                individual, ukf_cov, total_distance
            )
            futures.append((i, future))
        
        # Collect results
        for i, future in futures:
            try:
                all_objectives[i] = future.result(timeout=0.5)
            except Exception as e:
                self.get_logger().warn(f"Parallel evaluation failed for individual {i}: {e}")
                # Use penalty values
                path_array = np.array([[p[0], p[1]] for p in population[i]])
                all_objectives[i] = [float('inf'), float('inf'), float('inf'), float('inf')]
        
        return all_objectives
    
    def evaluate_individual_objectives(self, individual, ukf_cov, total_distance):
        """Evaluate objectives for a single individual"""
        path_array = np.array([[p[0], p[1]] for p in individual])
        return self.evaluate_all_objectives(path_array, ukf_cov, total_distance)
    
    def evaluate_all_objectives(self, path_array, ukf_cov, total_distance):
        """Evaluate all five objectives for NSGA-II (now includes obstacle cost)"""
        if len(path_array) < 2:
            return [float('inf'), float('inf'), float('inf'), float('inf'), float('inf')]
        
        objectives = []
        
        # 1. Path length (minimize)
        path_length = self.calculate_path_length(path_array)
        objectives.append(path_length)
        
        # 2. Smoothness (minimize curvature)
        curvature_penalty = self.calculate_curvature(path_array)
        objectives.append(curvature_penalty)
        
        # 3. Obstacle cost (minimize) - NEW OBJECTIVE
        obstacle_cost = self.get_obstacle_cost(path_array)
        objectives.append(obstacle_cost)
        
        # 4. Uncertainty cost (minimize)
        uncertainty_cost = self.uncertainty_planner.get_uncertainty_cost(path_array)
        objectives.append(uncertainty_cost)
        
        # 5. Deviation from straight line (minimize)
        straight_line_length = total_distance
        length_ratio = abs(path_length - straight_line_length) / max(straight_line_length, 0.1)
        objectives.append(length_ratio)
        
        return objectives
    
    def get_area_obstacle_density(self, start_x, start_y, goal_x, goal_y):
        """Get obstacle density in the area between start and goal"""
        if self.obstacle_grid is None:
            return 0.0
        
        # Define bounding box
        min_x = min(start_x, goal_x) - 1.0
        max_x = max(start_x, goal_x) + 1.0
        min_y = min(start_y, goal_y) - 1.0
        max_y = max(start_y, goal_y) + 1.0
        
        # Convert to grid coordinates
        grid_min_x, grid_min_y = self.world_to_grid(min_x, min_y)
        grid_max_x, grid_max_y = self.world_to_grid(max_x, max_y)
        
        if None in [grid_min_x, grid_min_y, grid_max_x, grid_max_y]:
            return 0.0
        
        # Clamp to grid bounds
        grid_min_x = max(0, grid_min_x)
        grid_min_y = max(0, grid_min_y)
        grid_max_x = min(self.obstacle_grid.shape[0] - 1, grid_max_x)
        grid_max_y = min(self.obstacle_grid.shape[1] - 1, grid_max_y)
        
        # Calculate density
        if grid_max_x <= grid_min_x or grid_max_y <= grid_min_y:
            return 0.0
        
        area = (grid_max_x - grid_min_x + 1) * (grid_max_y - grid_min_y + 1)
        obstacle_count = np.sum(self.obstacle_grid[grid_min_x:grid_max_x+1, grid_min_y:grid_max_y+1] > 0.1)
        
        return obstacle_count / area if area > 0 else 0.0
    
    def initialize_obstacle_aware_population(self, start_x, start_y, goal_x, goal_y, population_size, obstacle_density):
        """Initialize population with obstacle-aware heuristic methods"""
        population = []
        
        # 1. Straight line path (always include)
        straight_path = []
        for i in range(self.waypoint_count):
            t = i / (self.waypoint_count - 1)
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            straight_path.append((x, y))
        population.append(straight_path)
        
        # 2. Obstacle-aware variations
        for i in range(population_size - 1):
            if obstacle_density < 0.3:
                # Low obstacles - use Bezier curves
                path = self.generate_bezier_path(start_x, start_y, goal_x, goal_y, i)
            elif obstacle_density < 0.7:
                # Medium obstacles - use offset paths
                path = self.generate_offset_path(start_x, start_y, goal_x, goal_y, i, obstacle_density)
            else:
                # High obstacles - use waypoint navigation
                path = self.generate_waypoint_path(start_x, start_y, goal_x, goal_y, i)
            
            population.append(path)
        
        return population
    
    def generate_bezier_path(self, start_x, start_y, goal_x, goal_y, seed):
        """Generate Bezier curve path"""
        random.seed(seed)
        path = []
        path.append((start_x, start_y))
        
        # Generate control points
        mid_x = (start_x + goal_x) / 2
        mid_y = (start_y + goal_y) / 2
        
        # Random offset based on distance
        max_offset = min(1.0, math.hypot(goal_x - start_x, goal_y - start_y) * 0.3)
        offset_x = random.uniform(-max_offset, max_offset)
        offset_y = random.uniform(-max_offset, max_offset)
        
        control_x = mid_x + offset_x
        control_y = mid_y + offset_y
        
        # Generate quadratic Bezier curve points
        for j in range(1, self.waypoint_count - 1):
            t = j / (self.waypoint_count - 1)
            # Quadratic Bezier formula
            x = (1-t)**2 * start_x + 2*(1-t)*t * control_x + t**2 * goal_x
            y = (1-t)**2 * start_y + 2*(1-t)*t * control_y + t**2 * goal_y
            path.append((x, y))
        
        path.append((goal_x, goal_y))
        return path
    
    def generate_offset_path(self, start_x, start_y, goal_x, goal_y, seed, obstacle_density):
        """Generate offset path to avoid obstacles"""
        random.seed(seed)
        path = []
        path.append((start_x, start_y))
        
        # Calculate base direction
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.hypot(dx, dy)
        
        if distance < 1e-6:
            return [(start_x, start_y)] * self.waypoint_count
        
        # Apply offset perpendicular to direction
        angle = math.atan2(dy, dx)
        perpendicular = angle + math.pi / 2
        
        # Offset magnitude based on obstacle density
        max_offset = min(1.0, distance * 0.4)
        offset_mag = random.uniform(-max_offset, max_offset) * (1.0 + obstacle_density)
        
        # Generate offset waypoints
        for j in range(1, self.waypoint_count - 1):
            t = j / (self.waypoint_count - 1)
            
            # Base point on straight line
            base_x = start_x + t * dx
            base_y = start_y + t * dy
            
            # Apply offset
            offset_x = base_x + offset_mag * math.cos(perpendicular) * (1.0 - abs(2*t - 1))
            offset_y = base_y + offset_mag * math.sin(perpendicular) * (1.0 - abs(2*t - 1))
            
            path.append((offset_x, offset_y))
        
        path.append((goal_x, goal_y))
        return path
    
    def generate_waypoint_path(self, start_x, start_y, goal_x, goal_y, seed):
        """Generate path with intermediate waypoints for obstacle-rich environments"""
        random.seed(seed)
        path = []
        path.append((start_x, start_y))
        
        # Create 1-2 intermediate waypoints
        num_intermediate = random.randint(1, 2)
        intermediate_points = []
        
        for i in range(num_intermediate):
            # Random point between start and goal
            t = random.uniform(0.2, 0.8)
            base_x = start_x + t * (goal_x - start_x)
            base_y = start_y + t * (goal_y - start_y)
            
            # Random offset
            max_offset = min(1.0, math.hypot(goal_x - start_x, goal_y - start_y) * 0.2)
            offset_x = random.uniform(-max_offset, max_offset)
            offset_y = random.uniform(-max_offset, max_offset)
            
            intermediate_points.append((base_x + offset_x, base_y + offset_y))
        
        # Sort intermediate points by distance from start
        intermediate_points.sort(key=lambda p: math.hypot(p[0] - start_x, p[1] - start_y))
        
        # Combine all points
        all_points = [(start_x, start_y)] + intermediate_points + [(goal_x, goal_y)]
        
        # Interpolate to required number of waypoints
        for i in range(self.waypoint_count):
            t = i / (self.waypoint_count - 1)
            segment_idx = int(t * (len(all_points) - 1))
            segment_t = t * (len(all_points) - 1) - segment_idx
            
            if segment_idx < len(all_points) - 1:
                p1 = all_points[segment_idx]
                p2 = all_points[segment_idx + 1]
                x = p1[0] + segment_t * (p2[0] - p1[0])
                y = p1[1] + segment_t * (p2[1] - p1[1])
            else:
                x, y = all_points[-1]
            
            if i == 0:
                path[0] = (x, y)  # Update start (should be same)
            elif i == self.waypoint_count - 1:
                path.append((x, y))  # Add goal
            else:
                path.append((x, y))
        
        return path
    
    def obstacle_aware_crossover(self, parent1, parent2, obstacle_density):
        """Obstacle-aware crossover operator"""
        if len(parent1) != len(parent2):
            return parent1 if random.random() < 0.5 else parent2
        
        child = []
        
        # More aggressive crossover in high obstacle areas
        crossover_aggression = 0.5 + obstacle_density * 0.5
        
        for i in range(len(parent1)):
            if random.random() < crossover_aggression:
                # Blend parents
                blend_factor = random.random()
                x1, y1 = parent1[i]
                x2, y2 = parent2[i]
                
                child_x = x1 + blend_factor * (x2 - x1)
                child_y = y1 + blend_factor * (y2 - y1)
                
                child.append((child_x, child_y))
            else:
                child.append(parent1[i] if random.random() < 0.5 else parent2[i])
        
        return child
    
    def obstacle_aware_mutation(self, individual, obstacle_density):
        """Obstacle-aware mutation operator"""
        mutated = list(individual)
        
        # Higher mutation rate in high obstacle areas
        mutation_rate = self.mutation_rate * (1.0 + obstacle_density)
        
        for i in range(1, len(mutated) - 1):  # Don't mutate start and end
            if random.random() < mutation_rate:
                x, y = mutated[i]
                
                # Mutation magnitude based on obstacle density
                mutation_power = self.get_parameter('nsga_mutation_power').value * (1.0 + obstacle_density)
                
                # Polynomial mutation
                u = random.random()
                if u < 0.5:
                    delta = (2 * u) ** (1.0 / (mutation_power + 1)) - 1
                else:
                    delta = 1 - (2 * (1 - u)) ** (1.0 / (mutation_power + 1))
                
                # Apply mutation with obstacle-aware direction
                # Try to move away from obstacles
                obstacle_direction = self.get_obstacle_gradient(x, y)
                if obstacle_direction is not None:
                    # Move away from obstacles
                    dx = -obstacle_direction[0] * delta * mutation_power
                    dy = -obstacle_direction[1] * delta * mutation_power
                else:
                    # Random mutation
                    dx = delta * mutation_power
                    dy = delta * mutation_power
                
                mutated[i] = (x + dx, y + dy)
        
        return mutated
    
    def get_obstacle_gradient(self, x, y):
        """Get gradient of obstacle density at a point"""
        if self.obstacle_grid is None:
            return None
        
        grid_x, grid_y = self.world_to_grid(x, y)
        if grid_x is None:
            return None
        
        # Check bounds
        if not (1 <= grid_x < self.obstacle_grid.shape[0] - 1 and 
                1 <= grid_y < self.obstacle_grid.shape[1] - 1):
            return None
        
        # Calculate gradient using finite differences
        grad_x = (self.obstacle_grid[grid_x + 1, grid_y] - self.obstacle_grid[grid_x - 1, grid_y]) / 2
        grad_y = (self.obstacle_grid[grid_x, grid_y + 1] - self.obstacle_grid[grid_x, grid_y - 1]) / 2
        
        magnitude = math.hypot(grad_x, grad_y)
        if magnitude < 1e-6:
            return None
        
        # Normalize
        return grad_x / magnitude, grad_y / magnitude
    
    def repair_path_with_obstacles(self, path, start_x, start_y, goal_x, goal_y):
        """Ensure path starts and ends at correct positions and avoids obstacles"""
        repaired = list(path)
        
        # Ensure start and goal are correct
        repaired[0] = (start_x, start_y)
        repaired[-1] = (goal_x, goal_y)
        
        # Repair intermediate waypoints
        for i in range(1, len(repaired) - 1):
            x, y = repaired[i]
            
            # Simple bounds check
            min_x = min(start_x, goal_x) - 2.0
            max_x = max(start_x, goal_x) + 2.0
            min_y = min(start_y, goal_y) - 2.0
            max_y = max(start_y, goal_y) + 2.0
            
            x = max(min_x, min(max_x, x))
            y = max(min_y, min(max_y, y))
            
            # Check for obstacles and adjust if needed
            grid_x, grid_y = self.world_to_grid(x, y)
            if grid_x is not None:
                if (0 <= grid_x < self.obstacle_grid.shape[0] and 
                    0 <= grid_y < self.obstacle_grid.shape[1]):
                    
                    if self.obstacle_grid[grid_x, grid_y] > 0.5:
                        # Point is in obstacle, move it
                        # Try to find a nearby free space
                        for radius in range(1, 4):
                            found = False
                            for dx in range(-radius, radius + 1):
                                for dy in range(-radius, radius + 1):
                                    new_grid_x = grid_x + dx
                                    new_grid_y = grid_y + dy
                                    
                                    if (0 <= new_grid_x < self.obstacle_grid.shape[0] and 
                                        0 <= new_grid_y < self.obstacle_grid.shape[1]):
                                        
                                        if self.obstacle_grid[new_grid_x, new_grid_y] < 0.1:
                                            # Convert back to world coordinates
                                            x = self.obstacle_grid_origin[0] + (new_grid_x - self.obstacle_grid.shape[0] // 2) * self.obstacle_grid_resolution
                                            y = self.obstacle_grid_origin[1] + (new_grid_y - self.obstacle_grid.shape[1] // 2) * self.obstacle_grid_resolution
                                            found = True
                                            break
                                if found:
                                    break
                            if found:
                                break
            
            repaired[i] = (x, y)
        
        return repaired
    
    def smooth_path_with_obstacles(self, path):
        """Apply obstacle-aware path smoothing"""
        if len(path) < 3:
            return path
        
        # Convert to numpy array
        path_array = np.array([[p[0], p[1]] for p in path])
        
        # Apply uncertainty-aware smoothing
        start = (path_array[0, 0], path_array[0, 1])
        goal = (path_array[-1, 0], path_array[-1, 1])
        smoothed = self.uncertainty_planner.suggest_low_uncertainty_path(start, goal, path_array)
        
        # Additional obstacle-aware smoothing
        smoothed = self.apply_obstacle_aware_smoothing(smoothed)
        
        # Convert back to list format
        return [(x, y) for x, y in smoothed]
    
    def apply_obstacle_aware_smoothing(self, path_array, iterations=3):
        """Apply obstacle-aware Laplacian smoothing"""
        if len(path_array) < 3:
            return path_array
        
        smoothed = path_array.copy()
        
        for _ in range(iterations):
            for i in range(1, len(smoothed) - 1):
                # Laplacian smoothing
                new_x = 0.5 * (smoothed[i-1, 0] + smoothed[i+1, 0])
                new_y = 0.5 * (smoothed[i-1, 1] + smoothed[i+1, 1])
                
                # Check if new position is obstacle-free
                grid_x, grid_y = self.world_to_grid(new_x, new_y)
                if grid_x is not None:
                    if (0 <= grid_x < self.obstacle_grid.shape[0] and 
                        0 <= grid_y < self.obstacle_grid.shape[1]):
                        
                        if self.obstacle_grid[grid_x, grid_y] < 0.1:
                            # Free space, apply smoothing
                            smoothed[i, 0] = 0.8 * smoothed[i, 0] + 0.2 * new_x
                            smoothed[i, 1] = 0.8 * smoothed[i, 1] + 0.2 * new_y
        
        return smoothed
    
    def generate_obstacle_aware_fallback_path(self, start_x, start_y, goal_x, goal_y):
        """Generate fallback path that tries to avoid obstacles"""
        # Try to find a path around obstacles
        if self.obstacle_grid is not None:
            # Simple heuristic: offset perpendicular to direct line
            dx = goal_x - start_x
            dy = goal_y - start_y
            distance = math.hypot(dx, dy)
            
            if distance > 1e-6:
                angle = math.atan2(dy, dx)
                
                # Try both sides
                for side in [-1, 1]:
                    offset_angle = angle + side * math.pi / 4  # 45 degrees offset
                    offset_distance = min(1.0, distance * 0.3)
                    
                    # Create offset path
                    path = []
                    for i in range(self.waypoint_count):
                        t = i / (self.waypoint_count - 1)
                        
                        if t < 0.5:
                            # First half: move to offset
                            offset_t = t * 2
                            x = start_x + offset_t * offset_distance * math.cos(offset_angle)
                            y = start_y + offset_t * offset_distance * math.sin(offset_angle)
                        else:
                            # Second half: move to goal
                            offset_t = (t - 0.5) * 2
                            offset_end_x = start_x + offset_distance * math.cos(offset_angle)
                            offset_end_y = start_y + offset_distance * math.sin(offset_angle)
                            x = offset_end_x + offset_t * (goal_x - offset_end_x)
                            y = offset_end_y + offset_t * (goal_y - offset_end_y)
                        
                        path.append((x, y))
                    
                    # Check if this path is better
                    path_array = np.array([[p[0], p[1]] for p in path])
                    obstacle_cost = self.get_obstacle_cost(path_array)
                    
                    if obstacle_cost < 0.5:  # Reasonably obstacle-free
                        self.publish_path(path)
                        if self.debug_mode:
                            self.get_logger().info(f"🔄 Published obstacle-aware fallback path (side={side})")
                        return
        
        # Default: straight line
        self.generate_fallback_path(start_x, start_y, goal_x, goal_y)
    
    def generate_fallback_path(self, start_x, start_y, goal_x, goal_y):
        """Generate simple straight line path as fallback"""
        path = []
        
        # Generate waypoints along straight line
        for i in range(self.waypoint_count):
            t = i / (self.waypoint_count - 1)
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            path.append((x, y))
        
        # Apply uncertainty-aware smoothing
        path_array = np.array([[p[0], p[1]] for p in path])
        smoothed = self.uncertainty_planner.suggest_low_uncertainty_path(
            (start_x, start_y), (goal_x, goal_y), path_array
        )
        
        # Convert to list and publish
        final_path = [(x, y) for x, y in smoothed]
        self.publish_path(final_path)
        
        if self.debug_mode:
            self.get_logger().info(f"🔄 Published fallback path with {len(final_path)} waypoints")
    
    # [KEEP ALL OTHER METHODS FROM ORIGINAL FILE - calculate_path_length, calculate_curvature,
    # binary_tournament_selection, dominates, extract_ukf_covariance, 
    # optimization_timer_callback, has_robot_moved_significantly, etc.]
    
    # Add these missing methods from the original:
    
    def calculate_path_length(self, path_array):
        """Calculate total path length"""
        if len(path_array) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path_array) - 1):
            dx = path_array[i+1, 0] - path_array[i, 0]
            dy = path_array[i+1, 1] - path_array[i, 1]
            total_length += math.hypot(dx, dy)
        
        return total_length
    
    def calculate_curvature(self, path_array):
        """Calculate curvature penalty"""
        if len(path_array) < 3:
            return 0.0
        
        total_curvature = 0.0
        for i in range(1, len(path_array) - 1):
            p0 = path_array[i-1]
            p1 = path_array[i]
            p2 = path_array[i+1]
            
            # Calculate vectors
            v1 = p1 - p0
            v2 = p2 - p1
            
            # Calculate angle between vectors
            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)
            
            if norm_v1 > 1e-6 and norm_v2 > 1e-6:
                cos_angle = np.dot(v1, v2) / (norm_v1 * norm_v2)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle = math.acos(cos_angle)
                total_curvature += angle
        
        return total_curvature
    
    def binary_tournament_selection(self, population, objectives, tournament_size=2):
        """Binary tournament selection for NSGA-II"""
        # Randomly select tournament participants
        tournament_indices = random.sample(range(len(population)), 
                                          min(tournament_size, len(population)))
        
        # Select the non-dominated individual
        best_idx = tournament_indices[0]
        for idx in tournament_indices[1:]:
            # Check if current individual dominates the best
            if self.dominates(objectives[idx], objectives[best_idx]):
                best_idx = idx
        
        return population[best_idx]
    
    def dominates(self, a, b):
        """Check if objective vector a dominates b"""
        # Minimization: a dominates b if a[i] <= b[i] for all i and a[i] < b[i] for at least one i
        all_less_equal = all(x <= y for x, y in zip(a, b))
        any_less = any(x < y for x, y in zip(a, b))
        return all_less_equal and any_less
    
    def extract_ukf_covariance(self):
        """Extract position covariance from UKF"""
        if self.robot_pose is None:
            return 0.0
        
        try:
            cov = np.array(self.robot_pose.pose.covariance).reshape(6, 6)
            pos_cov = np.trace(cov[:2, :2])  # Sum of x,y variances
            return float(pos_cov)
        except:
            return 0.0
    
    def publish_path(self, path_points):
        """Convert and publish path to ROS topic"""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            # Calculate orientation towards next point
            if i < len(path_points) - 1:
                next_x, next_y = path_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = 0.0
            
            pose.pose.orientation = self.yaw_to_quaternion(yaw)
            path.poses.append(pose)
        
        # Publish multiple times for reliability
        for _ in range(3):
            self.path_pub.publish(path)
            time.sleep(0.05)
        
        if self.debug_mode:
            path_length = self.calculate_path_length(np.array([[p[0], p[1]] for p in path_points]))
            obstacle_cost = self.get_obstacle_cost(np.array([[p[0], p[1]] for p in path_points]))
            self.get_logger().info(
                f"📤 Published optimized path: {len(path.poses)} waypoints, "
                f"length={path_length:.2f}m, obstacle_cost={obstacle_cost:.3f}"
            )
    
    def optimization_timer_callback(self):
        """Periodic optimization trigger"""
        current_time = time.time()
        
        # Check if we should trigger new optimization
        if (self.robot_pose is not None and self.goal_pose is not None and 
            not self.optimization_active and
            current_time - self.last_optimization_time > 1.0/self.planning_rate):
            
            # Check if we've moved significantly
            if self.has_robot_moved_significantly():
                self.trigger_optimization()
    
    def has_robot_moved_significantly(self):
        """Check if robot has moved enough to require re-planning"""
        if self.last_robot_position is None and self.robot_pose is not None:
            self.last_robot_position = (
                self.robot_pose.pose.pose.position.x,
                self.robot_pose.pose.pose.position.y
            )
            return True
        
        if self.robot_pose is None:
            return False
        
        current_pos = (self.robot_pose.pose.pose.position.x, 
                      self.robot_pose.pose.pose.position.y)
        
        if self.last_robot_position is None:
            self.last_robot_position = current_pos
            return True
        
        dx = current_pos[0] - self.last_robot_position[0]
        dy = current_pos[1] - self.last_robot_position[1]
        distance = math.hypot(dx, dy)
        
        # Check if moved more than 0.5m
        if distance > 0.5:
            self.last_robot_position = current_pos
            return True
        
        return False
    
    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main():
    rclpy.init()
    node = CompletePathOptimizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down enhanced NSGA-II path optimizer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
