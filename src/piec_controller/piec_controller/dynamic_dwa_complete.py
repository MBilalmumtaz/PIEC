#!/usr/bin/env python3
"""
Dynamic DWA Complete Planner with Enhanced Obstacle Avoidance and Free Space Awareness
"""

import numpy as np
import math
import time

# Progressive rotation threshold constants (in meters and radians)
# FIX: Reduced thresholds to prevent infinite spinning when goals are to side/back
# Previous values (90/60/30) caused robot to spin in place for lateral goals
ROTATION_THRESHOLD_FAR_DISTANCE = 2.0  # Distance threshold for far range (meters)
ROTATION_THRESHOLD_MID_DISTANCE = 0.5  # Distance threshold for mid range (meters)
ROTATION_THRESHOLD_FAR_ANGLE = 45  # Rotation threshold when far from goal (degrees) - REDUCED from 90
ROTATION_THRESHOLD_MID_ANGLE = 30  # Rotation threshold in mid range (degrees) - REDUCED from 60
ROTATION_THRESHOLD_CLOSE_ANGLE = 15  # Rotation threshold when close to goal (degrees) - REDUCED from 30

# Maximum rotation time before forcing forward motion (seconds)
MAX_ROTATION_TIME = 5.0


class DynamicDWAComplete:
    def __init__(self, node, emergency_distance=0.2):
        self.node = node
        
        # Enhanced DWA parameters
        self.max_v = 1.0
        self.min_v = 0.05
        self.max_w = 0.8  # Increased
        self.sim_time = 1.5  # Increased
        self.dt = 0.05
        self.v_resolution = 0.05
        self.w_resolution = 0.05
        
        # Enhanced weights
        self.w_goal = 1.5
        self.w_speed = 0.7
        self.w_clearance = 3.0  # Increased
        self.w_path = 1.0
        self.w_free_space = 0.8  # NEW: Free space preference
        
        # Robot parameters
        self.robot_radius = 0.35
        self.predict_time = 10  # Increased
        
        # Emergency parameters
        self.emergency_distance = emergency_distance
        
        # Free space parameters
        self.free_space_threshold = 0.8
        
        # Rotation timeout tracking (FIX for infinite spinning)
        self.fallback_rotation_start_time = None
        self.max_rotation_time = MAX_ROTATION_TIME
        
        self.node.get_logger().info("Dynamic DWA Complete initialized with Free Space Awareness")

    def plan(self, current_pose, path, scan_ranges, scan_angles):
        """Main planning function with free space consideration"""
        if path is None or len(path.poses) == 0:
            return 0.0, 0.0
            
        if scan_ranges is None or scan_angles is None or len(scan_ranges) == 0:
            return self.fallback_control(current_pose, path)
        
        try:
            # Get current state
            x, y, theta = current_pose
            
            # Dynamic velocity limits based on obstacles and free space
            dynamic_max_v, dynamic_max_w = self.calculate_dynamic_limits(scan_ranges, scan_angles, theta)
            
            # Generate velocity candidates
            v_samples = np.arange(self.min_v, dynamic_max_v + self.v_resolution, self.v_resolution)
            w_samples = np.arange(-dynamic_max_w, dynamic_max_w + self.w_resolution, self.w_resolution)
            
            if len(v_samples) == 0 or len(w_samples) == 0:
                return self.fallback_control(current_pose, path)
            
            # Evaluate trajectories
            best_trajectory = None
            best_score = -float('inf')
            best_v = 0.0
            best_w = 0.0
            
            # Get goal direction for free space scoring
            goal_x = path.poses[-1].pose.position.x
            goal_y = path.poses[-1].pose.position.y
            goal_dir = math.atan2(goal_y - y, goal_x - x)
            
            for v in v_samples:
                for w in w_samples:
                    # Skip combinations that are too extreme
                    if abs(w) > 1.5 * abs(v) + 0.5:  # Too much rotation for speed
                        continue
                    
                    # Generate trajectory
                    trajectory = self.generate_trajectory(x, y, theta, v, w)
                    
                    # Check if trajectory is valid
                    if not self.check_trajectory(trajectory, scan_ranges, scan_angles, theta):
                        continue
                    
                    # Calculate scores
                    goal_score = self.calc_goal_score(trajectory, path)
                    speed_score = self.calc_speed_score(v, w)
                    clearance_score = self.calc_clearance_score(trajectory, scan_ranges, scan_angles, theta)
                    path_score = self.calc_path_score(trajectory, path)
                    free_space_score = self.calc_free_space_score(trajectory, scan_ranges, scan_angles, theta, goal_dir)
                    
                    # Weighted total score
                    total_score = (
                        self.w_goal * goal_score +
                        self.w_speed * speed_score +
                        self.w_clearance * clearance_score +
                        self.w_path * path_score +
                        self.w_free_space * free_space_score
                    )
                    
                    if total_score > best_score:
                        best_score = total_score
                        best_v = v
                        best_w = w
                        best_trajectory = trajectory
            
            # If no valid trajectory found, use fallback
            if best_trajectory is None:
                return self.fallback_control(current_pose, path)
            
            # Log DWA decision
            if hasattr(self.node, 'debug_mode') and self.node.debug_mode and hasattr(self.node, 'control_counter') and self.node.control_counter % 50 == 0:
                clearance = self.calc_min_clearance(best_trajectory, scan_ranges, scan_angles, theta)
                free_space = self.calc_free_space_score(best_trajectory, scan_ranges, scan_angles, theta, goal_dir)
                self.node.get_logger().debug(f"DWA: v={best_v:.3f}, w={best_w:.3f}, score={best_score:.3f}, clearance={clearance:.3f}, free={free_space:.3f}")
            
            return best_v, best_w
            
        except Exception as e:
            if hasattr(self.node, 'debug_mode') and self.node.debug_mode:
                self.node.get_logger().warn(f"DWA planning error: {e}")
            return self.fallback_control(current_pose, path)

    def calculate_dynamic_limits(self, scan_ranges, scan_angles, robot_theta):
        """Calculate dynamic velocity limits based on obstacles and free space - MAX SPEED in open space"""
        max_v = self.max_v
        max_w = self.max_w
        
        if scan_ranges is None or scan_angles is None:
            return max_v, max_w
        
        # Check if we're in open space
        min_distance = float('inf')
        for range_val in scan_ranges:
            if 0.1 < range_val < min_distance:
                min_distance = range_val
        
        # OPEN SPACE: Maximum speed
        if min_distance > 2.0:  # At least 2m clearance all around
            return self.max_v, self.max_w
        
        # Original logic for obstacles
        min_distances = []
        
        front_min = float('inf')
        for i, (range_val, angle) in enumerate(zip(scan_ranges, scan_angles)):
            if abs(angle) < math.radians(60):
                if 0.1 < range_val < front_min:
                    front_min = range_val
        min_distances.append(front_min)
        
        effective_min = min(min_distances)
        
        if effective_min < self.emergency_distance:
            max_v = 0.0
            max_w = 0.0
        elif effective_min < 0.4:
            max_v = min(max_v, 0.25)
            max_w = min(max_w, 0.3)
        elif effective_min < 0.7:
            max_v = min(max_v, 0.5)
            max_w = min(max_w, 0.5)
        elif effective_min < 1.0:
            max_v = min(max_v, 0.7)
            max_w = min(max_w, 0.7)
        
        return max_v, max_w
    def generate_trajectory(self, x, y, theta, v, w):
        """Generate predicted trajectory"""
        trajectory = []
        time = 0.0
        
        current_x, current_y, current_theta = x, y, theta
        
        while time <= self.sim_time:
            trajectory.append((current_x, current_y, current_theta))
            
            # Apply motion model
            current_x += v * math.cos(current_theta) * self.dt
            current_y += v * math.sin(current_theta) * self.dt
            current_theta += w * self.dt
            
            time += self.dt
            
            # Normalize angle
            current_theta = math.atan2(math.sin(current_theta), math.cos(current_theta))
        
        return trajectory

    def check_trajectory(self, trajectory, scan_ranges, scan_angles, robot_theta):
        """Check if trajectory is collision-free with enhanced checking"""
        if not trajectory:
            return False
        
        # Check multiple points along trajectory
        check_indices = [0, len(trajectory)//3, 2*len(trajectory)//3, -1]
        
        for idx in check_indices:
            if idx >= len(trajectory):
                continue
                
            x, y, theta = trajectory[idx]
            
            # Convert point to robot frame
            dx = x - trajectory[0][0]
            dy = y - trajectory[0][1]
            
            # Rotate to robot frame
            local_x = dx * math.cos(-robot_theta) - dy * math.sin(-robot_theta)
            local_y = dx * math.sin(-robot_theta) + dy * math.cos(-robot_theta)
            
            # Check distance from robot center
            dist = math.hypot(local_x, local_y)
            angle = math.atan2(local_y, local_x)
            
            # Check for collisions using laser data with safety margin
            if not self.check_collision(dist, angle, scan_ranges, scan_angles, safety_margin=0.15):
                return False
        
        return True

    def check_collision(self, dist, angle, scan_ranges, scan_angles, safety_margin=0.1):
        """Check for collision at a specific point with safety margin
        
        Returns:
            True if no collision (safe)
            False if collision detected (unsafe)
        """
        if scan_ranges is None or scan_angles is None:
            return True  # No scan data, assume safe
        
        # Find closest scan angle
        min_angle_diff = float('inf')
        closest_idx = 0
        
        for i, scan_angle in enumerate(scan_angles):
            angle_diff = abs(scan_angle - angle)
            if angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                closest_idx = i
        
        if closest_idx < len(scan_ranges):
            scan_dist = scan_ranges[closest_idx]
            # FIXED: Check if obstacle is CLOSER than where we want to go
            # If obstacle distance is less than our required clearance, it's a collision
            required_clearance = dist + self.robot_radius + safety_margin
            if 0.1 < scan_dist < required_clearance:
                return False  # Collision! Obstacle is too close
        
        return True

    def calc_goal_score(self, trajectory, path):
        """Calculate goal-reaching score"""
        if not trajectory or not path.poses:
            return 0.0
        
        # Use end of trajectory
        end_x, end_y, _ = trajectory[-1]
        
        # Find progress along path
        min_dist = float('inf')
        for pose in path.poses:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - end_x, py - end_y)
            if dist < min_dist:
                min_dist = dist
        
        # Also consider heading toward goal
        start_x, start_y, _ = trajectory[0]
        goal_x = path.poses[-1].pose.position.x
        goal_y = path.poses[-1].pose.position.y
        
        initial_dist = math.hypot(goal_x - start_x, goal_y - start_y)
        progress = max(0, initial_dist - min_dist)
        
        return 0.7 / (1.0 + min_dist) + 0.3 * progress / max(initial_dist, 0.1)

    def calc_speed_score(self, v, w):
        """Calculate speed score with better balancing"""
        # Guard against division by zero
        if self.max_v <= 0 or self.max_w <= 0:
            return 0.0
        
        v_score = v / self.max_v
        w_score = 1.0 - (abs(w) / self.max_w) * 0.5  # Less penalty for turning
        
        # Prefer moving forward over turning in place
        if v < 0.1 and abs(w) > 0.3:
            v_score *= 0.5
        
        return 0.6 * v_score + 0.4 * w_score

    def calc_clearance_score(self, trajectory, scan_ranges, scan_angles, robot_theta):
        """Calculate clearance score with enhanced safety"""
        if not trajectory:
            return 0.0
        
        min_clearance = float('inf')
        clearance_sum = 0.0
        count = 0
        
        # Sample points along trajectory
        step = max(1, len(trajectory) // 5)
        for i in range(0, len(trajectory), step):
            x, y, theta = trajectory[i]
            
            # Convert point to robot frame
            dx = x - trajectory[0][0]
            dy = y - trajectory[0][1]
            
            local_x = dx * math.cos(-robot_theta) - dy * math.sin(-robot_theta)
            local_y = dx * math.sin(-robot_theta) + dy * math.cos(-robot_theta)
            
            dist = math.hypot(local_x, local_y)
            angle = math.atan2(local_y, local_x)
            
            # Get clearance at this point
            clearance = self.get_clearance_at(dist, angle, scan_ranges, scan_angles)
            if clearance < min_clearance:
                min_clearance = clearance
            
            clearance_sum += clearance
            count += 1
        
        if min_clearance == float('inf'):
            return 0.0
        
        # Use both min clearance and average clearance
        avg_clearance = clearance_sum / max(count, 1)
        normalized_min = min(1.0, min_clearance / 1.0)
        normalized_avg = min(1.0, avg_clearance / 2.0)
        
        return 0.7 * normalized_min + 0.3 * normalized_avg

    def calc_free_space_score(self, trajectory, scan_ranges, scan_angles, robot_theta, goal_dir):
        """Calculate free space score - preference for open areas"""
        if not trajectory:
            return 0.0
        
        free_space_sum = 0.0
        count = 0
        
        # Check points along trajectory
        step = max(1, len(trajectory) // 4)
        for i in range(0, len(trajectory), step):
            x, y, theta = trajectory[i]
            
            # Convert point to robot frame
            dx = x - trajectory[0][0]
            dy = y - trajectory[0][1]
            
            local_x = dx * math.cos(-robot_theta) - dy * math.sin(-robot_theta)
            local_y = dx * math.sin(-robot_theta) + dy * math.cos(-robot_theta)
            
            dist = math.hypot(local_x, local_y)
            angle = math.atan2(local_y, local_x)
            
            # Get clearance in multiple directions around this point
            local_free_space = 0.0
            for dir_offset in [-0.5, 0, 0.5]:  # Check left, center, right
                check_angle = angle + dir_offset
                clearance = self.get_clearance_at(dist, check_angle, scan_ranges, scan_angles)
                if clearance > self.free_space_threshold:
                    local_free_space += 1.0
            
            free_space_sum += local_free_space / 3.0  # Average of 3 directions
            count += 1
        
        if count == 0:
            return 0.0
        
        return free_space_sum / count

    def get_clearance_at(self, dist, angle, scan_ranges, scan_angles):
        """Get clearance at a specific point in robot frame"""
        if scan_ranges is None or scan_angles is None:
            return 1.0
        
        # Find closest scan angle
        min_angle_diff = float('inf')
        closest_idx = 0
        
        for i, scan_angle in enumerate(scan_angles):
            angle_diff = abs(scan_angle - angle)
            if angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                closest_idx = i
        
        if closest_idx < len(scan_ranges):
            scan_dist = scan_ranges[closest_idx]
            if scan_dist > 0.1:
                return scan_dist - dist - self.robot_radius
        
        return 2.0  # Default large clearance

    def calc_path_score(self, trajectory, path):
        """Calculate path following score"""
        if not trajectory or not path.poses:
            return 0.0
        
        total_score = 0.0
        count = 0
        
        # Sample trajectory points
        traj_step = max(1, len(trajectory) // 5)
        for i in range(0, len(trajectory), traj_step):
            x, y, _ = trajectory[i]
            
            # Find closest path point
            min_dist = float('inf')
            for pose in path.poses:
                px = pose.pose.position.x
                py = pose.pose.position.y
                dist = math.hypot(px - x, py - y)
                if dist < min_dist:
                    min_dist = dist
            
            total_score += 1.0 / (1.0 + min_dist)
            count += 1
        
        return total_score / max(1, count)

    def calc_min_clearance(self, trajectory, scan_ranges, scan_angles, robot_theta):
        """Calculate minimum clearance along trajectory"""
        min_clearance = float('inf')
        
        if not trajectory:
            return 0.0
        
        # Check key points
        check_points = [0, len(trajectory)//2, -1]
        for idx in check_points:
            if idx >= len(trajectory):
                continue
                
            x, y, theta = trajectory[idx]
            
            dx = x - trajectory[0][0]
            dy = y - trajectory[0][1]
            
            local_x = dx * math.cos(-robot_theta) - dy * math.sin(-robot_theta)
            local_y = dx * math.sin(-robot_theta) + dy * math.cos(-robot_theta)
            
            dist = math.hypot(local_x, local_y)
            angle = math.atan2(local_y, local_x)
            
            clearance = self.get_clearance_at(dist, angle, scan_ranges, scan_angles)
            if clearance < min_clearance:
                min_clearance = clearance
        
        return min_clearance if min_clearance < float('inf') else 0.0

    def fallback_control(self, current_pose, path):
        """Fallback control when DWA fails - IMPROVED for large heading errors with timeout protection"""
        if path is None or len(path.poses) == 0:
            return 0.0, 0.0
        
        x, y, theta = current_pose
        
        # Find target waypoint (look ahead)
        target_idx = min(4, len(path.poses) - 1)
        target_pose = path.poses[target_idx].pose
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        
        # Calculate control
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Distance to final goal for progressive threshold reduction
        goal_x = path.poses[-1].pose.position.x
        goal_y = path.poses[-1].pose.position.y
        goal_distance = math.hypot(goal_x - x, goal_y - y)
        
        # FIX: Progressive rotation threshold - reduced values to prevent infinite spinning
        # Far from goal (>2m): 45°, Mid-range (0.5-2m): 30°, Close (<0.5m): 15°
        if goal_distance > ROTATION_THRESHOLD_FAR_DISTANCE:
            rotation_threshold = math.radians(ROTATION_THRESHOLD_FAR_ANGLE)
        elif goal_distance > ROTATION_THRESHOLD_MID_DISTANCE:
            rotation_threshold = math.radians(ROTATION_THRESHOLD_MID_ANGLE)
        else:
            rotation_threshold = math.radians(ROTATION_THRESHOLD_CLOSE_ANGLE)
        
        if distance < 0.3:
            v = 0.0
            w = 0.0
            # Reset rotation timer when close to waypoint
            self.fallback_rotation_start_time = None
        elif abs(angle_diff) > rotation_threshold:
            # FIX: Add rotation timeout to prevent infinite spinning
            current_time = time.time()
            if self.fallback_rotation_start_time is None:
                self.fallback_rotation_start_time = current_time
            
            rotation_duration = current_time - self.fallback_rotation_start_time
            
            if rotation_duration > self.max_rotation_time:
                # Timeout exceeded - force forward motion with turning to break deadlock
                # This prevents infinite spinning when goals are to the side/back
                # Robot moves forward while turning, which should reduce angle error over time
                v = min(0.2, distance * 0.3)  # Slow forward motion
                w = np.clip(angle_diff * 0.5, -self.max_w * 0.4, self.max_w * 0.4)
                if hasattr(self.node, 'debug_mode') and self.node.debug_mode:
                    self.node.get_logger().warn(
                        f"⚠️ Fallback rotation timeout ({rotation_duration:.1f}s) - forcing forward motion"
                    )
                # Keep timer running (don't reset) so we continue forward motion
                # Timer will reset when angle_diff drops below threshold
            else:
                # Turn in place for large errors (normal behavior)
                v = 0.0
                w = np.clip(angle_diff * 0.7, -self.max_w * 0.6, self.max_w * 0.6)
        else:
            # Reset rotation timer when not rotating in place
            self.fallback_rotation_start_time = None
            
            # Move forward with turning
            v = min(0.4, distance * 0.6)
            w = angle_diff * 0.5
            w = np.clip(w, -self.max_w * 0.5, self.max_w * 0.5)
            
            # Slow down if turning sharply
            if abs(w) > 0.3:
                v *= 0.7
        
        return v, w
