#!/usr/bin/env python3
"""
Intelligent Hybrid Planner: MPC-PINN + DWA with Adaptive Mode Selection
"""
import numpy as np
import math
import time
from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Optional
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

# Import both planners
try:
    from .mpc_planner import PINNMPCPlanner
    from .dynamic_dwa import DynamicDWAPlanner
    MPC_AVAILABLE = True
    DWA_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import planners: {e}")
    MPC_AVAILABLE = False
    DWA_AVAILABLE = False


class PlannerMode(Enum):
    """Available planner modes"""
    MPC_PINN = 1      # Predictive, optimal - for complex scenarios
    DWA = 2           # Fast, reactive - for simple navigation
    EMERGENCY = 3     # Emergency stop
    SIMPLE = 4        # Fallback simple control


@dataclass
class EnvironmentAssessment:
    """Assessment of current environment for mode selection"""
    obstacle_density: float = 0.0          # 0-1, density of obstacles
    clearance: float = 10.0               # Average clearance (m)
    path_complexity: float = 0.0          # 0-1, path curvature/variation
    energy_importance: float = 0.5        # 0-1, how important is energy optimization
    stability_importance: float = 0.5      # 0-1, how important is stability
    computation_budget: float = 0.15      # Available computation time (s)
    is_dynamic_environment: bool = False  # Moving obstacles detected


class IntelligentHybridPlanner:
    def __init__(self, node: Node, emergency_distance=0.3):
        self.node = node
        self.emergency_distance = emergency_distance
        
        # Initialize both planners
        self.mpc_planner = None
        self.dwa_planner = None
        
        if MPC_AVAILABLE:
            try:
                self.mpc_planner = PINNMPCPlanner(node, emergency_distance)
                self.node.get_logger().info("✅ MPC-PINN planner initialized")
            except Exception as e:
                self.node.get_logger().warn(f"Failed to init MPC: {e}")
                self.mpc_planner = None
        
        if DWA_AVAILABLE:
            try:
                self.dwa_planner = DynamicDWAPlanner(node, emergency_distance)
                self.node.get_logger().info("✅ DWA planner initialized")
            except Exception as e:
                self.node.get_logger().warn(f"Failed to init DWA: {e}")
                self.dwa_planner = None
        
        # Mode selection parameters
        self.current_mode = PlannerMode.DWA  # Start with DWA (simpler)
        self.mode_switch_time = 0.0
        self.min_mode_duration = 0.5  # Minimum time in a mode (s)
        
        # Performance tracking
        self.mpc_success_count = 0
        self.mpc_failure_count = 0
        self.dwa_success_count = 0
        self.dwa_failure_count = 0
        self.last_planning_times = {
            'mpc': [],
            'dwa': [],
            'simple': []
        }
        
        # Failure tracking
        self.consecutive_dwa_failures = 0
        self.consecutive_mpc_failures = 0
        self.max_consecutive_failures = 3
        
        # Environmental memory
        self.obstacle_history = []
        self.max_history = 20
        
        # Configuration
        self.mpc_timeout = 0.3  # 300ms max for MPC
        self.dwa_timeout = 0.2  # 200ms max for DWA
        
        self.node.get_logger().info("🚀 Intelligent Hybrid Planner initialized")
        self.node.get_logger().info(f"Available planners: MPC={self.mpc_planner is not None}, DWA={self.dwa_planner is not None}")
    
    def assess_environment(self, scan_ranges, scan_angles, path_msg, 
                          current_pose, current_velocity) -> EnvironmentAssessment:
        """Assess current environment to select optimal planner"""
        assessment = EnvironmentAssessment()
        
        # 1. Calculate obstacle density and clearance
        if scan_ranges and scan_angles:
            front_ranges = []
            for r, angle in zip(scan_ranges, scan_angles):
                if abs(angle) < 0.785:  # ±45 degrees front sector
                    if 0.1 < r < 5.0:
                        front_ranges.append(r)
            
            if front_ranges:
                # Obstacle density (proportion of close obstacles)
                close_obstacles = sum(1 for r in front_ranges if r < 1.0)
                assessment.obstacle_density = close_obstacles / max(len(front_ranges), 1)
                
                # Average clearance
                assessment.clearance = np.mean(front_ranges) if front_ranges else 10.0
                
                # Detect dynamic obstacles by checking variance
                if len(self.obstacle_history) >= 5:
                    recent_std = np.std(front_ranges) if len(front_ranges) > 1 else 0.0
                    assessment.is_dynamic_environment = recent_std > 0.2
                
                # Update history
                self.obstacle_history.append(assessment.obstacle_density)
                if len(self.obstacle_history) > self.max_history:
                    self.obstacle_history.pop(0)
        
        # 2. Calculate path complexity
        if path_msg and len(path_msg.poses) >= 3:
            angles = []
            for i in range(1, len(path_msg.poses) - 1):
                p0 = path_msg.poses[i-1].pose.position
                p1 = path_msg.poses[i].pose.position
                p2 = path_msg.poses[i+1].pose.position
                
                v1 = np.array([p1.x - p0.x, p1.y - p0.y])
                v2 = np.array([p2.x - p1.x, p2.y - p1.y])
                
                if np.linalg.norm(v1) > 0.05 and np.linalg.norm(v2) > 0.05:
                    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.arccos(cos_angle)
                    angles.append(angle)
            
            if angles:
                assessment.path_complexity = np.mean(angles) / np.pi
        
        # 3. Determine energy/stability importance
        if assessment.obstacle_density > 0.6:
            assessment.stability_importance = 0.8
            assessment.energy_importance = 0.5
        elif assessment.clearance > 1.5:
            assessment.energy_importance = 0.7
            assessment.stability_importance = 0.4
        
        # 4. Estimate computation budget
        assessment.computation_budget = 0.1
        
        return assessment
    
    def select_planner_mode(self, assessment: EnvironmentAssessment) -> PlannerMode:
        """Intelligently select the best planner mode"""
        current_time = time.time()
        
        # Don't switch too frequently
        if current_time - self.mode_switch_time < self.min_mode_duration:
            return self.current_mode
        
        # Check for emergency conditions FIRST
        if assessment.obstacle_density > 0.7 and assessment.clearance < 0.4:
            return PlannerMode.EMERGENCY
        
        # If DWA has been failing consecutively, avoid using it
        if self.consecutive_dwa_failures >= self.max_consecutive_failures:
            self.node.get_logger().info("Skipping DWA due to consecutive failures")
            if self.mpc_planner is not None:
                return PlannerMode.MPC_PINN
            else:
                return PlannerMode.SIMPLE
        
        # Rule-based mode selection
        should_use_mpc = False
        
        # Condition 1: Complex environment needs MPC
        if (assessment.obstacle_density > 0.3 and 
            assessment.path_complexity > 0.2):
            should_use_mpc = True
        
        # Condition 2: Energy/stability optimization needed
        if (assessment.energy_importance > 0.6 or 
            assessment.stability_importance > 0.6):
            should_use_mpc = True
        
        # Condition 3: Dynamic environment needs predictive planning
        if assessment.is_dynamic_environment:
            should_use_mpc = True
        
        # Condition 4: Simple, open environment - use DWA for speed
        if (assessment.obstacle_density < 0.1 and 
            assessment.clearance > 1.0 and 
            assessment.path_complexity < 0.1 and 
            self.consecutive_dwa_failures == 0):
            should_use_mpc = False
        
        # Condition 5: Computation budget too low for MPC
        if assessment.computation_budget < 0.05:
            should_use_mpc = False
        
        # Select mode
        if should_use_mpc and self.mpc_planner is not None:
            new_mode = PlannerMode.MPC_PINN
        elif self.dwa_planner is not None and self.consecutive_dwa_failures < 2:
            new_mode = PlannerMode.DWA
        else:
            new_mode = PlannerMode.SIMPLE
        
        # Log mode switch if changed
        if new_mode != self.current_mode:
            self.mode_switch_time = current_time
            self.node.get_logger().info(
                f"🔄 Planner mode switch: {self.current_mode.name} -> {new_mode.name}"
            )
            self.log_switch_reason(assessment, new_mode)
        
        return new_mode
    
    def log_switch_reason(self, assessment: EnvironmentAssessment, new_mode: PlannerMode):
        """Log the reason for mode switch"""
        reasons = []
        
        if new_mode == PlannerMode.MPC_PINN:
            if assessment.obstacle_density > 0.3:
                reasons.append(f"obstacle density ({assessment.obstacle_density:.2f})")
            if assessment.path_complexity > 0.2:
                reasons.append(f"path complexity ({assessment.path_complexity:.2f})")
            if self.consecutive_dwa_failures > 0:
                reasons.append("DWA failures")
        
        elif new_mode == PlannerMode.DWA:
            if assessment.obstacle_density < 0.1:
                reasons.append(f"low obstacles ({assessment.obstacle_density:.2f})")
            if assessment.clearance > 1.0:
                reasons.append(f"clearance ({assessment.clearance:.1f}m)")
        
        elif new_mode == PlannerMode.SIMPLE:
            if self.consecutive_dwa_failures >= 2:
                reasons.append("DWA failing repeatedly")
            if self.mpc_planner is None:
                reasons.append("MPC not available")
        
        if reasons:
            self.node.get_logger().info(f"  Reason(s): {', '.join(reasons)}")
    
    def plan_with_mpc(self, current_pose, path_msg, scan_ranges, scan_angles) -> Tuple[float, float]:
        """Execute MPC planning with timeout protection"""
        if self.mpc_planner is None:
            return 0.0, 0.0
        
        start_time = time.time()
        
        try:
            # DISABLE PINN temporarily to avoid timeouts
            if hasattr(self.mpc_planner, 'use_pinn'):
                original_use_pinn = self.mpc_planner.use_pinn
                self.mpc_planner.use_pinn = False  # TEMPORARY FIX
            
            v, w = self.mpc_planner.plan(current_pose, path_msg, scan_ranges, scan_angles)
            
            # Restore PINN setting
            if hasattr(self.mpc_planner, 'use_pinn'):
                self.mpc_planner.use_pinn = original_use_pinn
            
            planning_time = time.time() - start_time
            
            # Record planning time
            self.last_planning_times['mpc'].append(planning_time)
            if len(self.last_planning_times['mpc']) > 10:
                self.last_planning_times['mpc'].pop(0)
            
            # Check for timeout
            if planning_time > self.mpc_timeout:
                self.node.get_logger().warn(f"MPC timeout: {planning_time*1000:.1f}ms")
                self.mpc_failure_count += 1
                self.consecutive_mpc_failures += 1
                return 0.0, 0.0
            
            self.mpc_success_count += 1
            self.consecutive_mpc_failures = 0
            return v, w
            
        except Exception as e:
            self.node.get_logger().warn(f"MPC planning error: {e}")
            self.mpc_failure_count += 1
            self.consecutive_mpc_failures += 1
            return 0.0, 0.0
    
    def plan_with_dwa(self, current_pose, path_msg, scan_ranges, scan_angles) -> Tuple[float, float]:
        """Execute DWA planning with timeout protection"""
        if self.dwa_planner is None:
            return 0.0, 0.0
        
        start_time = time.time()
        
        try:
            # Check if DWA has the correct method name
            if hasattr(self.dwa_planner, 'plan_with_obstacle_awareness'):
                v, w = self.dwa_planner.plan_with_obstacle_awareness(
                    current_pose, path_msg, scan_ranges, scan_angles
                )
            elif hasattr(self.dwa_planner, 'plan'):
                v, w = self.dwa_planner.plan(
                    current_pose, path_msg, scan_ranges, scan_angles
                )
            else:
                self.node.get_logger().error("DWA planner has no plan method!")
                return 0.0, 0.0
                
            planning_time = time.time() - start_time
            
            # Record planning time
            self.last_planning_times['dwa'].append(planning_time)
            if len(self.last_planning_times['dwa']) > 10:
                self.last_planning_times['dwa'].pop(0)
            
            # Check for timeout
            if planning_time > self.dwa_timeout:
                self.node.get_logger().warn(f"DWA timeout: {planning_time*1000:.1f}ms")
                self.dwa_failure_count += 1
                self.consecutive_dwa_failures += 1
                return 0.0, 0.0
            
            self.dwa_success_count += 1
            self.consecutive_dwa_failures = 0
            return v, w
            
        except Exception as e:
            self.node.get_logger().warn(f"DWA planning error: {e}")
            self.dwa_failure_count += 1
            self.consecutive_dwa_failures += 1
            return 0.0, 0.0
    
    def emergency_stop(self, scan_ranges, scan_angles) -> bool:
        """Check for emergency stop conditions"""
        if not scan_ranges or not scan_angles:
            return False
        
        # Check immediate front for obstacles
        for r, angle in zip(scan_ranges, scan_angles):
            if abs(angle) < 0.26:  # ~15 degrees
                if 0.05 < r < self.emergency_distance:
                    return True
        
        return False
    
    def simple_control(self, target_point, current_pose, scan_ranges, scan_angles) -> Tuple[float, float]:
        """Improved simple proportional control with obstacle avoidance"""
        cx, cy, cyaw = current_pose
        tx, ty = target_point
        
        dx = tx - cx
        dy = ty - cy
        distance = math.hypot(dx, dy)
        
        if distance < 0.1:
            return 0.0, 0.0
        
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - cyaw
        
        # Normalize angle
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        # Check for obstacles in front
        obstacle_ahead = False
        min_front_distance = float('inf')
        
        if scan_ranges and scan_angles:
            for r, angle in zip(scan_ranges, scan_angles):
                if abs(angle) < 0.5:  # ~30 degrees front cone
                    if 0.1 < r < 0.8:  # Obstacle within 0.8m
                        obstacle_ahead = True
                        min_front_distance = min(min_front_distance, r)
        
        if obstacle_ahead and min_front_distance < 0.5:
            # Obstacle too close, turn away
            v = 0.0
            # Find clearer direction
            best_clearance = 0.0
            best_direction = 0.0
            
            for i in range(0, len(scan_angles), 5):  # Sample every 5th
                if i < len(scan_ranges) and scan_ranges[i] > best_clearance:
                    best_clearance = scan_ranges[i]
                    best_direction = scan_angles[i]
            
            w = np.clip(best_direction * 0.8, -1.0, 1.0)
            self.node.get_logger().info(f"Obstacle detected at {min_front_distance:.2f}m, turning w={w:.2f}")
        else:
            # Simple P-control with obstacle-aware speed
            speed_factor = 1.0
            if obstacle_ahead:
                speed_factor = min_front_distance / 0.8  # Scale speed with distance
            
            v = min(0.3, distance * 0.5) * speed_factor
            w = yaw_error * 0.8  # Reduced gain
        
        return v, w
    
    def get_target_from_path(self, current_pose, path_msg, lookahead=0.8):
        """Extract target point from path"""
        if not path_msg or len(path_msg.poses) == 0:
            return None
        
        cx, cy, _ = current_pose
        
        # Find closest point
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(path_msg.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - cx, py - cy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Look ahead (adaptive based on speed)
        lookahead_idx = min(closest_idx + int(lookahead * 8), len(path_msg.poses) - 1)
        target_pose = path_msg.poses[lookahead_idx].pose.position
        
        return (target_pose.x, target_pose.y)
    
    def plan(self, current_pose, path_msg, scan_ranges, scan_angles, 
             current_velocity=0.0) -> Tuple[float, float, PlannerMode]:
        """Main planning function with intelligent mode selection"""
        
        # 1. Check for emergency stop
        if self.emergency_stop(scan_ranges, scan_angles):
            self.node.get_logger().warn("🚨 EMERGENCY STOP")
            return 0.0, 0.0, PlannerMode.EMERGENCY
        
        # 2. Assess environment
        assessment = self.assess_environment(
            scan_ranges, scan_angles, path_msg, 
            current_pose, current_velocity
        )
        
        # 3. Select optimal planner mode
        new_mode = self.select_planner_mode(assessment)
        self.current_mode = new_mode
        
        # 4. Execute planning with selected mode
        v, w = 0.0, 0.0
        
        if new_mode == PlannerMode.MPC_PINN:
            v, w = self.plan_with_mpc(current_pose, path_msg, scan_ranges, scan_angles)
            
        elif new_mode == PlannerMode.DWA:
            v, w = self.plan_with_dwa(current_pose, path_msg, scan_ranges, scan_angles)
            
        elif new_mode == PlannerMode.SIMPLE:
            target_point = self.get_target_from_path(current_pose, path_msg)
            if target_point:
                v, w = self.simple_control(target_point, current_pose, scan_ranges, scan_angles)
        
        # 5. If planning failed, try fallback
        if (abs(v) < 0.01 and abs(w) < 0.01) or (v == 0.0 and w == 0.0):
            self.node.get_logger().warn(f"{new_mode.name} failed, trying fallback...")
            
            # Try alternative planner
            if new_mode == PlannerMode.MPC_PINN and self.dwa_planner and self.consecutive_dwa_failures < 2:
                v, w = self.plan_with_dwa(current_pose, path_msg, scan_ranges, scan_angles)
                if abs(v) > 0.01 or abs(w) > 0.01:
                    self.current_mode = PlannerMode.DWA
            
            elif new_mode == PlannerMode.DWA and self.mpc_planner:
                v, w = self.plan_with_mpc(current_pose, path_msg, scan_ranges, scan_angles)
                if abs(v) > 0.01 or abs(w) > 0.01:
                    self.current_mode = PlannerMode.MPC_PINN
            
            # Try simple control if all else fails
            if (abs(v) < 0.01 and abs(w) < 0.01):
                target_point = self.get_target_from_path(current_pose, path_msg)
                if target_point:
                    v, w = self.simple_control(target_point, current_pose, scan_ranges, scan_angles)
                    self.current_mode = PlannerMode.SIMPLE
        
        return float(v), float(w), self.current_mode
    
    def get_performance_stats(self):
        """Get performance statistics"""
        avg_mpc_time = np.mean(self.last_planning_times['mpc']) if self.last_planning_times['mpc'] else 0
        avg_dwa_time = np.mean(self.last_planning_times['dwa']) if self.last_planning_times['dwa'] else 0
        
        mpc_total = self.mpc_success_count + self.mpc_failure_count
        dwa_total = self.dwa_success_count + self.dwa_failure_count
        
        return {
            'current_mode': self.current_mode.name if self.current_mode else 'N/A',
            'mpc_success_rate': self.mpc_success_count / max(mpc_total, 1),
            'dwa_success_rate': self.dwa_success_count / max(dwa_total, 1),
            'avg_mpc_time_ms': avg_mpc_time * 1000,
            'avg_dwa_time_ms': avg_dwa_time * 1000,
            'consecutive_dwa_failures': self.consecutive_dwa_failures,
            'consecutive_mpc_failures': self.consecutive_mpc_failures,
            'mpc_available': self.mpc_planner is not None,
            'dwa_available': self.dwa_planner is not None,
        }
    
    def get_debug_info(self):  # ADD THIS METHOD
        """Get debug information for visualization"""
        return {
            'current_mode': self.current_mode,
            'mode_duration': time.time() - self.mode_switch_time,
            'mpc_available': self.mpc_planner is not None,
            'dwa_available': self.dwa_planner is not None,
            'obstacle_history': self.obstacle_history.copy(),
            'mpc_success_count': self.mpc_success_count,
            'dwa_success_count': self.dwa_success_count,
            'consecutive_failures': {
                'dwa': self.consecutive_dwa_failures,
                'mpc': self.consecutive_mpc_failures
            }
        }
