#!/usr/bin/env python3
"""
Simplified MPC Planner - Faster without PINN
"""
import numpy as np
import math
import time
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PINNMPCPlanner:
    def __init__(self, node: Node, emergency_distance=0.3):
        self.node = node
        self.emergency_distance = emergency_distance
        
        # Simplified parameters for faster computation
        self.N = 5  # Reduced horizon
        self.dt = 0.15  # Increased time step
        
        # Control bounds
        self.v_min = 0.0
        self.v_max = 0.6  # Reduced for better control
        self.w_min = -1.0
        self.w_max = 1.0
        
        # Disable PINN to avoid timeouts
        self.use_pinn = False
        
        self.node.get_logger().info("🚀 Simplified MPC Planner initialized")
        self.node.get_logger().info(f"Horizon: N={self.N}, dt={self.dt}s")
        self.node.get_logger().info(f"Using PINN: {self.use_pinn}")
    
    def robot_model(self, x, u):
        """Simple robot kinematic model"""
        dt = self.dt
        x_next = np.zeros_like(x)
        
        # Position update
        x_next[0] = x[0] + x[3] * math.cos(x[2]) * dt
        x_next[1] = x[1] + x[3] * math.sin(x[2]) * dt
        x_next[2] = x[2] + x[4] * dt
        
        # Velocity update
        v_new = x[3] + u[0] * dt
        w_new = x[4] + u[1] * dt
        
        # Apply bounds
        x_next[3] = np.clip(v_new, self.v_min, self.v_max)
        x_next[4] = np.clip(w_new, self.w_min, self.w_max)
        
        return x_next
    
    def get_reference_trajectory(self, current_state, path_msg):
        """Extract reference trajectory"""
        if path_msg is None or len(path_msg.poses) == 0:
            return None
        
        rx, ry, rtheta = current_state[:3]
        
        # Find closest point
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(path_msg.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Extract points ahead
        ref_traj = []
        total_points = min(self.N, len(path_msg.poses) - closest_idx)
        
        for i in range(total_points):
            idx = min(closest_idx + i, len(path_msg.poses) - 1)
            pose = path_msg.poses[idx]
            
            x_ref = pose.pose.position.x
            y_ref = pose.pose.position.y
            
            # Get orientation
            if idx < len(path_msg.poses) - 1:
                next_pose = path_msg.poses[idx + 1]
                next_x = next_pose.pose.position.x
                next_y = next_pose.pose.position.y
                yaw_ref = math.atan2(next_y - y_ref, next_x - x_ref)
            else:
                yaw_ref = rtheta
            
            ref_traj.append([x_ref, y_ref, yaw_ref, 0.3, 0.0])  # Constant velocity
        
        # Pad if needed
        while len(ref_traj) < self.N:
            if ref_traj:
                ref_traj.append(ref_traj[-1].copy())
            else:
                ref_traj.append([rx, ry, rtheta, 0.3, 0.0])
        
        return np.array(ref_traj)
    
    def simple_mpc_solve(self, x0, ref_traj, scan_ranges, scan_angles):
        """Simple MPC solution - faster than optimization"""
        # If path is straight, use simple control
        rx, ry, rtheta = x0[:3]
        
        # Target from reference
        target_x = ref_traj[1, 0] if len(ref_traj) > 1 else ref_traj[0, 0]
        target_y = ref_traj[1, 1] if len(ref_traj) > 1 else ref_traj[0, 1]
        
        # Calculate desired motion
        dx = target_x - rx
        dy = target_y - ry
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        
        # Angle difference
        angle_diff = target_angle - rtheta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Simple control law
        v = min(self.v_max, distance * 0.5)
        w = angle_diff * 1.0
        
        # Check for obstacles
        if scan_ranges and scan_angles:
            # Check front sector
            front_clearance = 2.0
            for i in range(len(scan_ranges)):
                if i < len(scan_angles):
                    angle = scan_angles[i]
                    if abs(angle) < 0.5:  # ~30 degrees
                        if 0.1 < scan_ranges[i] < 0.8:
                            front_clearance = min(front_clearance, scan_ranges[i])
            
            # Reduce speed if obstacle is close
            if front_clearance < 0.8:
                v *= front_clearance / 0.8
                # Turn away from closest obstacle
                if front_clearance < 0.5:
                    # Find direction with most clearance
                    best_clearance = 0.0
                    best_direction = 0.0
                    
                    for i in range(len(scan_ranges)):
                        if i < len(scan_angles):
                            if scan_ranges[i] > best_clearance:
                                best_clearance = scan_ranges[i]
                                best_direction = scan_angles[i]
                    
                    w = best_direction * 0.5
        
        # Apply bounds
        v = np.clip(v, self.v_min, self.v_max)
        w = np.clip(w, self.w_min, self.w_max)
        
        # Calculate acceleration (for logging)
        a = (v - x0[3]) / self.dt
        alpha = (w - x0[4]) / self.dt
        
        return np.array([a, alpha]), v, w
    
    def plan(self, current_pose, path_msg, scan_ranges, scan_angles):
        """Main planning function"""
        start_time = time.time()
        
        # Check for emergency stop
        if self.check_emergency_stop(scan_ranges, scan_angles):
            return 0.0, 0.0
        
        # Extract current state
        x0 = np.array([
            current_pose[0],  # x
            current_pose[1],  # y
            current_pose[2],  # theta
            0.3,  # v (initial guess)
            0.0   # w
        ])
        
        # Get reference trajectory
        ref_traj = self.get_reference_trajectory(x0, path_msg)
        if ref_traj is None:
            return 0.0, 0.0
        
        # Solve MPC (simplified)
        u_opt, v_next, w_next = self.simple_mpc_solve(x0, ref_traj, scan_ranges, scan_angles)
        
        planning_time = time.time() - start_time
        
        # Log if debug mode
        try:
            if self.node.get_parameter('debug_mode').value:
                self.node.get_logger().info(
                    f"MPC: v={v_next:.3f}, w={w_next:.3f}, "
                    f"a={u_opt[0]:.2f}, alpha={u_opt[1]:.2f}"
                )
        except:
            pass
        
        return float(v_next), float(w_next)
    
    def check_emergency_stop(self, scan_ranges, scan_angles):
        """Check for immediate collision danger"""
        if scan_ranges is None or scan_angles is None:
            return False
        
        # Check front sector
        for i in range(min(len(scan_ranges), len(scan_angles))):
            angle = scan_angles[i]
            if abs(angle) < 0.3:  # ~17 degrees
                if 0.05 < scan_ranges[i] < self.emergency_distance:
                    return True
        
        return False
