#!/usr/bin/env python3
"""
Fixed Objective Evaluator with PINN Integration - WORKING VERSION
"""
import math
import numpy as np
import rclpy
import time
from rclpy.node import Node
from collections import deque

# Import service message - FIXED PATH
try:
    # Try to import from your package
    from piec_pinn_surrogate_msgs.srv import EvaluateTrajectory
    PINN_AVAILABLE = True
except ImportError:
    # Fallback - create minimal service message
    PINN_AVAILABLE = False
    
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

class ObjectiveEvaluator:
    def __init__(self, node: Node):
        self.node = node
        
        # Get PINN parameters - with defaults
        try:
            self.use_pinn = node.get_parameter('use_pinn_predictions').value
            self.pinn_service_name = node.get_parameter('pinn_service_name').value
        except:
            self.use_pinn = False
            self.pinn_service_name = '/evaluate_trajectory'  # Default
        
        self.pinn_timeout = 2.0
        self.pinn_retry_count = 3
        
        # Log configuration
        self.node.get_logger().info(f"PINN Configuration: use_pinn={self.use_pinn}, service={self.pinn_service_name}")
        
        # Initialize PINN client
        self.service_available = False
        self.client = None
        
        # ADDED: Missing attributes for statistics
        self.pinn_usage = 0  # Total PINN calls made
        self.pinn_success = 0  # Successful PINN calls
        self.pinn_failures = 0  # Failed PINN calls
        
        if self.use_pinn and PINN_AVAILABLE:
            try:
                self.node.get_logger().info(f"Creating PINN client for service: {self.pinn_service_name}")
                self.client = node.create_client(EvaluateTrajectory, self.pinn_service_name)
                
                # Try to connect
                if self.check_pinn_service():
                    self.service_available = True
                    self.node.get_logger().info(f'✅ PINN service connected successfully!')
                else:
                    self.node.get_logger().warn(f'⚠️ PINN service not available at {self.pinn_service_name}')
                    self.use_pinn = False
            except Exception as e:
                self.node.get_logger().error(f'❌ Failed to create PINN client: {str(e)}')
                self.use_pinn = False
        else:
            if not PINN_AVAILABLE:
                self.node.get_logger().warn('PINN messages not available - using simulation mode')
            self.use_pinn = False
        
        # Objective weights
        try:
            self.objective_weights = node.get_parameter('objective_weights').value
        except:
            self.objective_weights = [0.15, 0.1, 0.15, 0.1, 0.1, 0.25, 0.15]
        
        # Performance tracking
        self.performance_history = []
        self.max_history = 100
        self.fallback_count = 0
    
    def check_pinn_service(self):
        """Check if PINN service is available"""
        if self.client is None:
            self.node.get_logger().warn("PINN client not initialized")
            return False
        
        try:
            self.node.get_logger().info(f"Waiting for PINN service: {self.pinn_service_name} (timeout: 3s)")
            
            # Wait for service with timeout
            if self.client.wait_for_service(timeout_sec=3.0):
                self.node.get_logger().info("✅ PINN service is available!")
                return True
            else:
                self.node.get_logger().warn("PINN service not responding")
                return False
        except Exception as e:
            self.node.get_logger().error(f"Error checking PINN service: {str(e)}")
            return False
    
    def call_pinn_service(self, xs, ys, yaws, velocities, max_retries=2):
        """Call PINN service - SIMPLIFIED WORKING VERSION"""
        # Check if we should even try
        if not self.use_pinn or not self.service_available or self.client is None:
            self.node.get_logger().debug("PINN not enabled or client not available")
            return None, None
        
        # Don't call if service not ready
        if not self.client.service_is_ready():
            self.node.get_logger().debug("PINN service not ready")
            return None, None
        
        # Prepare request
        req = EvaluateTrajectory.Request()
        req.xs = [float(x) for x in xs]
        req.ys = [float(y) for y in ys]
        req.yaws = [float(y) for y in yaws]
        req.velocities = [float(v) for v in velocities]
        
        self.node.get_logger().debug(f"Calling PINN with {len(xs)} points")
        
        # Track usage
        self.pinn_usage += 1
        
        # Single attempt with timeout
        try:
            future = self.client.call_async(req)
            
            # Wait with timeout
            start_time = time.time()
            timeout_sec = self.pinn_timeout
            
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    self.node.get_logger().warn(f"PINN call timeout after {timeout_sec}s")
                    future.cancel()
                    self.pinn_failures += 1
                    return None, None
                
                # Process callbacks
                rclpy.spin_once(self.node, timeout_sec=0.01)
            
            # Check result
            if future.done():
                result = future.result()
                if result is not None:
                    self.pinn_success += 1
                    self.node.get_logger().debug(f"PINN success: energy={result.energy:.2f}, stability={result.stability:.2f}")
                    return result.energy, result.stability
                else:
                    self.node.get_logger().warn("PINN call returned None")
                    self.pinn_failures += 1
        
        except Exception as e:
            self.node.get_logger().warn(f"PINN call failed: {str(e)}")
            self.pinn_failures += 1
        
        # Increment fallback counter
        self.fallback_count += 1
        return None, None
    
    def calculate_path_metrics(self, path_array):
        """Calculate geometric metrics of a path"""
        if len(path_array) < 2:
            return 0.0, 0.0, 0.0, []
        
        # Calculate path length
        path_length = 0.0
        segment_lengths = []
        
        for i in range(len(path_array) - 1):
            dx = path_array[i+1, 0] - path_array[i, 0]
            dy = path_array[i+1, 1] - path_array[i, 1]
            seg_len = math.hypot(dx, dy)
            path_length += seg_len
            segment_lengths.append(seg_len)
        
        # Calculate curvature
        total_curvature = 0.0
        if len(path_array) >= 3:
            for i in range(1, len(path_array) - 1):
                p0 = path_array[i-1]
                p1 = path_array[i]
                p2 = path_array[i+1]
                
                # Calculate vectors
                v1 = p1 - p0
                v2 = p2 - p1
                
                norm_v1 = np.linalg.norm(v1)
                norm_v2 = np.linalg.norm(v2)
                
                if norm_v1 > 1e-6 and norm_v2 > 1e-6:
                    cos_angle = np.dot(v1, v2) / (norm_v1 * norm_v2)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = math.acos(cos_angle)
                    total_curvature += abs(angle)
        
        avg_curvature = total_curvature / max(len(path_array) - 2, 1)
        
        # Calculate velocities along path
        velocities = []
        for i in range(len(path_array)):
            if i == 0:
                velocities.append(0.5)  # Start velocity
            elif i == len(path_array) - 1:
                velocities.append(0.0)  # Stop at end
            else:
                # Simple velocity profile
                vel = 0.6
                velocities.append(vel)
        
        return path_length, avg_curvature, segment_lengths, velocities
    
    def calculate_yaws(self, path_array):
        """Calculate yaw angles along path"""
        yaws = []
        
        for i in range(len(path_array)):
            if i == len(path_array) - 1:
                # Last point, use previous yaw
                if len(yaws) > 0:
                    yaws.append(yaws[-1])
                else:
                    yaws.append(0.0)
            else:
                dx = path_array[i+1, 0] - path_array[i, 0]
                dy = path_array[i+1, 1] - path_array[i, 1]
                yaw = math.atan2(dy, dx)
                yaws.append(yaw)
        
        return yaws
    
    def estimate_physics_based_energy(self, path_array, velocities, segment_lengths):
        """Fallback physics-based energy estimation"""
        if len(segment_lengths) == 0:
            return 0.0, 1.0
        
        # Simple physics model
        total_energy = 0.0
        total_stability = 0.0
        
        robot_mass = 50.0  # kg
        gravity = 9.81
        wheel_friction = 0.1
        
        for i in range(len(segment_lengths)):
            # Energy components
            kinetic_energy = 0.5 * robot_mass * velocities[i]**2
            rolling_resistance = robot_mass * gravity * wheel_friction * segment_lengths[i]
            
            total_energy += (kinetic_energy + rolling_resistance)
            total_stability += 1.0  # Default stability
        
        avg_stability = total_stability / len(segment_lengths)
        
        return total_energy, avg_stability
    
    def evaluate_all_objectives(self, path_array, obstacle_cost, uncertainty_cost, 
                                straight_line_length, use_pinn=True):
        """Evaluate all 7 objectives for NSGA-II - WORKING VERSION"""
        if len(path_array) < 2:
            return [float('inf')] * 7
        
        # Calculate basic metrics
        path_length, avg_curvature, segment_lengths, velocities = self.calculate_path_metrics(path_array)
        
        # Calculate deviation from straight line
        deviation_ratio = abs(path_length - straight_line_length) / max(straight_line_length, 0.1)
        
        # Get PINN predictions or use fallback
        pinn_energy = 0.0
        pinn_stability = 1.0
        pinn_used = False
        
        if use_pinn and self.use_pinn and self.service_available:
            # Prepare inputs for PINN
            xs = path_array[:, 0].tolist()
            ys = path_array[:, 1].tolist()
            yaws = self.calculate_yaws(path_array)
            
            # Call PINN service
            energy_pred, stability_pred = self.call_pinn_service(xs, ys, yaws, velocities)
            
            if energy_pred is not None and stability_pred is not None:
                pinn_energy = energy_pred
                pinn_stability = stability_pred
                pinn_used = True
                if self.node.get_parameter('debug_mode').value:
                    self.node.get_logger().info(f"✅ PINN used: E={pinn_energy:.1f}J, S={pinn_stability:.3f}")
        
        # If PINN failed or not used, use physics fallback
        if not pinn_used:
            pinn_energy, pinn_stability = self.estimate_physics_based_energy(path_array, velocities, segment_lengths)
            if self.use_pinn and self.node.get_parameter('debug_mode').value:
                self.node.get_logger().debug(f"PINN fallback: E={pinn_energy:.1f}J")
        
        # Return all 7 objectives
        return [
            path_length,           # 0: Minimize
            avg_curvature,         # 1: Minimize
            obstacle_cost,         # 2: Minimize
            uncertainty_cost,      # 3: Minimize
            deviation_ratio,       # 4: Minimize
            pinn_energy,           # 5: Minimize (YOUR PINN INNOVATION)
            1.0 - pinn_stability   # 6: Minimize (convert stability to cost)
        ]
    def evaluate_all_objectives_with_timeout(self, path_array, obstacle_cost, uncertainty_cost, 
                                            straight_line_length, use_pinn=True, timeout=0.5):
        """Evaluate all 7 objectives with timeout for PINN calls"""
        if len(path_array) < 2:
            return [float('inf')] * 7
        
        # DEBUG: Log PINN usage
        if use_pinn and self.node.get_parameter('debug_mode').value:
            self.node.get_logger().debug(f"PINN evaluation requested for path with {len(path_array)} points")
        
        # Calculate basic metrics
        path_length, avg_curvature, segment_lengths, velocities = self.calculate_path_metrics(path_array)
        
        # Calculate deviation from straight line
        deviation_ratio = abs(path_length - straight_line_length) / max(straight_line_length, 0.1)
        
        pinn_energy = 0.0
        pinn_stability = 1.0
        pinn_used = False
        pinn_success = False
        
        if use_pinn and self.use_pinn and hasattr(self.node, 'call_pinn_service_optimized'):
            try:
                # Call using node's optimized non-blocking method
                result = self.node.call_pinn_service_optimized(path_array)
                
                if result and result.get('success', False):
                    pinn_energy = result.get('energy', 0.0)
                    pinn_stability = result.get('stability', 1.0)
                    pinn_used = True
                    pinn_success = True
                    
                    if self.node.get_parameter('debug_mode').value:
                        response_time = result.get('response_time', 0.0)
                        self.node.get_logger().debug(f"✅ PINN success: E={pinn_energy:.1f}J, S={pinn_stability:.3f} ({response_time:.3f}s)")
                else:
                    # PINN failed or timed out
                    pinn_energy, pinn_stability = self.estimate_physics_based_energy(path_array, velocities, segment_lengths)
                    if result and result.get('timeout', False):
                        if self.node.get_parameter('debug_mode').value:
                            self.node.get_logger().debug(f"PINN timeout, using fallback: E={pinn_energy:.1f}J")
                    
            except Exception as e:
                if self.node.get_parameter('debug_mode').value:
                    self.node.get_logger().debug(f"PINN error: {e}, using fallback")
                pinn_energy, pinn_stability = self.estimate_physics_based_energy(path_array, velocities, segment_lengths)
        else:
            # Use physics fallback
            pinn_energy, pinn_stability = self.estimate_physics_based_energy(path_array, velocities, segment_lengths)
        
        # Update success tracking
        if pinn_used:
            self.pinn_usage += 1
            if pinn_success:
                self.pinn_success += 1
        
        # Return all 7 objectives
        return [
            path_length,           # 0: Minimize
            avg_curvature,         # 1: Minimize
            obstacle_cost,         # 2: Minimize
            uncertainty_cost,      # 3: Minimize
            deviation_ratio,       # 4: Minimize
            pinn_energy,           # 5: Minimize (YOUR PINN INNOVATION)
            1.0 - pinn_stability   # 6: Minimize (convert stability to cost)
        ]
    def calculate_weighted_fitness(self, objectives):
        """Calculate weighted fitness score"""
        if len(objectives) != len(self.objective_weights):
            # Simple sum as fallback
            return sum(objectives)
        
        # Weighted sum
        fitness = sum(w * o for w, o in zip(self.objective_weights, objectives))
        
        # Record performance
        self.record_performance(objectives, fitness)
        
        return fitness
    
    def record_performance(self, objectives, fitness):
        """Record performance for analysis"""
        record = {
            'objectives': objectives.copy(),
            'fitness': fitness,
            'timestamp': time.time()
        }
        self.performance_history.append(record)
        
        if len(self.performance_history) > self.max_history:
            self.performance_history.pop(0)
    
    def get_performance_stats(self):
        """Get performance statistics"""
        if not self.performance_history:
            return None
        
        stats = {
            'total_evaluations': len(self.performance_history),
            'pinn_usage': self.pinn_usage,
            'pinn_success': self.pinn_success,
            'pinn_failures': self.pinn_failures,
            'fallback_usage': self.fallback_count,
            'pinn_success_rate': self.pinn_success / max(self.pinn_usage, 1) * 100
        }
        
        return stats
