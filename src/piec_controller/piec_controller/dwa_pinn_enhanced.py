#!/usr/bin/env python3
"""
Enhanced DWA with PINN Multi-Objective Integration
"""

import rclpy
import numpy as np
import math
from rclpy.node import Node

class EnhancedDWAPINN:
    def __init__(self, node: Node):
        self.node = node
        
        # Multi-objective weights
        self.objective_weights = {
            'safety': 0.25,
            'energy': 0.25,
            'stability': 0.20,
            'progress': 0.15,
            'smoothness': 0.10,
            'path_alignment': 0.05
        }
        
        # DWA parameters
        self.max_v = 1.2
        self.min_v = 0.1
        self.max_w = 0.8
        self.robot_radius = 0.3
        
        self.node.get_logger().info("🎯 Enhanced DWA-PINN initialized")
    
    def evaluate_multi_objective(self, trajectory, scan_ranges, scan_angles, 
                                target_pose, global_path):
        """Evaluate trajectory using multiple objectives"""
        objectives = {}
        
        # 1. Safety Objective (Local Obstacle Avoidance)
        objectives['safety'] = self.calculate_safety_score(
            trajectory, scan_ranges, scan_angles
        )
        
        # 2. Energy Objective (Simplified - would call PINN in full version)
        objectives['energy'] = self.calculate_energy_score(trajectory)
        
        # 3. Stability Objective (Simplified)
        objectives['stability'] = self.calculate_stability_score(trajectory)
        
        # 4. Progress Objective (Goal Direction)
        objectives['progress'] = self.calculate_progress_score(
            trajectory, target_pose
        )
        
        # 5. Smoothness Objective
        objectives['smoothness'] = self.calculate_smoothness_score(trajectory)
        
        # 6. Path Alignment (Global Path Following)
        objectives['path_alignment'] = self.calculate_path_alignment_score(
            trajectory, global_path
        )
        
        # Apply weights and combine
        total_score = 0.0
        for objective, weight in self.objective_weights.items():
            if objective in objectives:
                total_score += weight * objectives[objective]
        
        return total_score, objectives
    
    def calculate_energy_score(self, trajectory):
        """Calculate energy efficiency score (simplified)"""
        if not trajectory:
            return 0.5
        
        # Simplified energy model
        total_energy = 0.0
        for i in range(len(trajectory) - 1):
            x1, y1, _ = trajectory[i]
            x2, y2, _ = trajectory[i+1]
            dist = math.hypot(x2 - x1, y2 - y1)
            
            # Energy proportional to distance
            total_energy += dist
        
        # Normalize (shorter path = less energy = better)
        avg_energy = total_energy / max(len(trajectory), 1)
        energy_score = max(0, 1.0 - (avg_energy / 5.0))  # Normalize to 0-1
        
        return energy_score
    
    def calculate_stability_score(self, trajectory):
        """Calculate stability score based on curvature"""
        if len(trajectory) < 3:
            return 0.5
        
        total_curvature = 0.0
        for i in range(1, len(trajectory) - 1):
            x1, y1, _ = trajectory[i-1]
            x2, y2, yaw2 = trajectory[i]
            x3, y3, _ = trajectory[i+1]
            
            # Calculate vectors
            v1 = np.array([x2 - x1, y2 - y1])
            v2 = np.array([x3 - x2, y3 - y2])
            
            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)
            
            if norm_v1 > 0 and norm_v2 > 0:
                cos_angle = np.dot(v1, v2) / (norm_v1 * norm_v2)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle = math.acos(cos_angle)
                total_curvature += abs(angle)
        
        avg_curvature = total_curvature / (len(trajectory) - 2)
        stability = 1.0 / (1.0 + avg_curvature * 2.0)
        
        return stability
    
    def calculate_safety_score(self, trajectory, scan_ranges, scan_angles):
        """Calculate local safety based on obstacles"""
        if not trajectory or not scan_ranges or not scan_angles:
            return 0.0
        
        safe_points = 0
        for x, y, _ in trajectory:
            # Convert to polar coordinates for obstacle check
            is_safe = True
            
            for i, (range_val, angle) in enumerate(zip(scan_ranges, scan_angles)):
                if 0.1 < range_val < 1.0:  # Check obstacles within 1m
                    # Convert scan point to Cartesian
                    obs_x = range_val * math.cos(angle)
                    obs_y = range_val * math.sin(angle)
                    
                    # Check distance
                    dist = math.hypot(obs_x - x, obs_y - y)
                    if dist < self.robot_radius:
                        is_safe = False
                        break
            
            if is_safe:
                safe_points += 1
        
        return safe_points / len(trajectory) if trajectory else 0.0
    
    def calculate_progress_score(self, trajectory, target_pose):
        """Calculate progress toward target"""
        if not trajectory or not target_pose:
            return 0.0
        
        start_x, start_y, _ = trajectory[0]
        end_x, end_y, _ = trajectory[-1]
        tx, ty = target_pose
        
        start_dist = math.hypot(tx - start_x, ty - start_y)
        end_dist = math.hypot(tx - end_x, ty - end_y)
        
        progress = start_dist - end_dist
        normalized_progress = progress / (self.max_v * 0.1 * len(trajectory))
        
        return max(0.0, min(1.0, normalized_progress))
    
    def calculate_smoothness_score(self, trajectory):
        """Calculate trajectory smoothness"""
        if len(trajectory) < 3:
            return 1.0
        
        curvature_sum = 0.0
        for i in range(1, len(trajectory) - 1):
            p0 = trajectory[i-1]
            p1 = trajectory[i]
            p2 = trajectory[i+1]
            
            # Calculate curvature
            dx1 = p1[0] - p0[0]
            dy1 = p1[1] - p0[1]
            dx2 = p2[0] - p1[0]
            dy2 = p2[1] - p1[1]
            
            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            curvature = abs(angle2 - angle1)
            curvature_sum += curvature
        
        avg_curvature = curvature_sum / (len(trajectory) - 2)
        smoothness = 1.0 / (1.0 + avg_curvature * 5.0)
        
        return smoothness
    
    def calculate_path_alignment_score(self, trajectory, global_path):
        """Calculate alignment with global path"""
        if not global_path or len(global_path.poses) < 2:
            return 1.0
        
        alignment_sum = 0.0
        for x, y, _ in trajectory:
            min_dist = float('inf')
            for pose_stamped in global_path.poses:
                px = pose_stamped.pose.position.x
                py = pose_stamped.pose.position.y
                dist = math.hypot(px - x, py - y)
                if dist < min_dist:
                    min_dist = dist
            
            alignment = 1.0 / (1.0 + min_dist * 2.0)
            alignment_sum += alignment
        
        return alignment_sum / len(trajectory) if trajectory else 1.0
