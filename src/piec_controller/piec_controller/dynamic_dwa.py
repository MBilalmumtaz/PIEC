#!/usr/bin/env python3
"""
Enhanced Dynamic DWA Planner with Advanced Obstacle Handling
Includes proper laser scan processing, dynamic obstacle detection, and PINN integration
"""
import math
import numpy as np
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

class DynamicDWAPlanner:
    def __init__(self, node: Node, emergency_distance=0.3):
        self.node = node
        self.emergency_distance = emergency_distance

        # Robot parameters
        self.max_v = 2.8
        self.min_v = 0.3
        self.max_w = 3.0
        self.robot_radius = 0.25
        self.robot_width = 0.35

        # Sampling parameters
        self.v_samples = 12
        self.w_samples = 24

        # Prediction horizon
        self.dt = 0.05
        self.sim_time = 2.0

        # Cost weights
        self.w_clearance = 2.5
        self.w_heading = 1.5
        self.w_progress = 1.8
        self.w_speed = 1.0
        self.w_smoothness = 0.5
        self.w_obstacle_free = 3.0
        self.w_path_alignment = 1.2

        # Obstacle parameters
        self.safe_distance = self.robot_radius + 0.5
        self.critical_distance = self.robot_radius + 0.2
        self.obstacle_influence_radius = 2.0

        # Dynamic obstacle tracking
        self.obstacle_history = []
        self.max_history_size = 10
        self.dynamic_obstacle_threshold = 0.2

        # State tracking
        self.last_best_v = 0.0
        self.last_best_w = 0.0
        self.consecutive_safe_plans = 0
        self.obstacle_map = {}
        self.map_resolution = 0.1

        # PINN integration
        self.pinn_service = None
        self.use_pinn = True

        # Performance monitoring
        self.planning_times = []
        self.max_planning_time = 0.05

        self.node.get_logger().info("🎯 Enhanced Dynamic DWA Planner initialized")
        self.node.get_logger().info(f"Max speed: {self.max_v}m/s, Safety distance: {self.safe_distance}m")

    def plan_with_obstacle_awareness(self, pose, path, scan_ranges, scan_angles):
        """Main planning function with advanced obstacle awareness"""
        start_time = time.time()
        
        # Handle None scan data
        if scan_ranges is None or scan_angles is None:
            self.node.get_logger().warn("⚠️ No scan data, using fallback")
            return (0.0, 0.0)
        
        # Convert to lists if needed
        scan_ranges = list(scan_ranges) if scan_ranges else []
        scan_angles = list(scan_angles) if scan_angles else []

        # Check for immediate danger
        if self.check_emergency_stop(scan_ranges, scan_angles):
            self.node.get_logger().warn("Emergency stop triggered!")
            return (0.0, 0.0)

        # Update obstacle history and detect dynamic obstacles
        self.update_obstacle_history(scan_ranges, scan_angles, pose)
        dynamic_obstacles = self.detect_dynamic_obstacles()

        # Get target from path
        target_pose = self.get_dynamic_lookahead_target(pose, path, scan_ranges)
        tx, ty = target_pose

        # Generate velocity samples with obstacle awareness
        v_samples, w_samples = self.generate_obstacle_aware_samples(
            scan_ranges, scan_angles, pose, dynamic_obstacles
        )

        best_score = -float('inf')
        best_cmd = (0.0, 0.0)
        best_traj = None

        # Evaluate all trajectories
        for v in v_samples:
            for w in w_samples:
                # Skip infeasible commands
                if not self.is_command_feasible(v, w, scan_ranges, scan_angles):
                    continue

                # Simulate trajectory
                traj = self.simulate_trajectory(pose, v, w)

                # Evaluate safety with advanced obstacle detection
                safety_score, closest_obstacle = self.evaluate_trajectory_safety_advanced(
                    traj, scan_ranges, scan_angles, pose, dynamic_obstacles
                )

                # Skip unsafe trajectories
                if safety_score < 0.05:  # Lowered from 0.1 to allow more candidates
                    continue

                # Calculate all cost components
                clearance_score = self.calculate_clearance_score(safety_score, closest_obstacle)
                heading_score = self.calculate_heading_score(traj[-1], tx, ty)
                progress_score = self.calculate_progress_score(traj[0], traj[-1], tx, ty, path)
                speed_score = self.calculate_speed_score(v, safety_score)
                smoothness_score = self.calculate_smoothness_score(v, w)
                obstacle_free_score = self.calculate_obstacle_free_score(traj, scan_ranges, scan_angles, pose)
                path_alignment_score = self.calculate_path_alignment_score(traj, path)

                # Dynamic weights based on environment
                weights = self.calculate_dynamic_weights(
                    safety_score, closest_obstacle, dynamic_obstacles
                )

                # Combined score
                score = (
                    weights['clearance'] * clearance_score +
                    weights['heading'] * heading_score +
                    weights['progress'] * progress_score +
                    weights['speed'] * speed_score +
                    weights['smoothness'] * smoothness_score +
                    weights['obstacle_free'] * obstacle_free_score +
                    weights['path_alignment'] * path_alignment_score
                ) * safety_score

                # Apply PINN prediction if available
                if self.use_pinn and safety_score > 0.5:
                    pinn_score = self.evaluate_pinn_score(traj, v, w)
                    score *= pinn_score

                # FAST LANE: if it's obviously safe, take the fastest sample
                if safety_score > 0.85 and v > best_cmd[0]:
                    best_cmd = (v, w)
                    best_score = 999.0
                elif score > best_score:
                    best_score = score
                    best_cmd = (v, w)
                    best_traj = traj

        planning_time = time.time() - start_time
        self.planning_times.append(planning_time)
        if len(self.planning_times) > 100:
            self.planning_times.pop(0)

        if planning_time > self.max_planning_time:
            self.node.get_logger().warn(f"Planning time high: {planning_time:.3f}s")

        if best_score == -float('inf'):
            self.node.get_logger().debug("No feasible trajectory found, using fallback")
            return self.intelligent_fallback(pose, scan_ranges, scan_angles, path)

        # Update state tracking
        if best_cmd[0] > 0.05:
            self.consecutive_safe_plans += 1
        else:
            self.consecutive_safe_plans = max(0, self.consecutive_safe_plans - 1)

        self.last_best_v = float(best_cmd[0])
        self.last_best_w = float(best_cmd[1])

        if self.node.get_parameter('debug_mode').value:
            self.node.get_logger().debug(
                f"DWA: v={best_cmd[0]:.3f}, w={best_cmd[1]:.3f}, "
                f"score={best_score:.3f}, time={planning_time:.3f}s"
            )

        # Return as Python floats, not numpy types
        return float(best_cmd[0]), float(best_cmd[1])

    # FIXED: Remove plan_with_timeout method as it causes issues on some systems
    # Use simplified version without signal handling
    def plan_with_timeout_simple(self, pose, path, scan_ranges, scan_angles, timeout=0.1):
        """Simplified timeout version without signal handling"""
        try:
            result = self.plan_with_obstacle_awareness(pose, path, scan_ranges, scan_angles)
            return result
        except Exception as e:
            self.node.get_logger().warn(f"DWA planning error: {e}, using fallback")
            return (0.5, 0.0)  # Slow forward, no rotation

    # ... rest of the dynamic_dwa.py file remains the same ...
    # (all other methods unchanged)
    def update_obstacle_history(self, ranges, angles, robot_pose):
        """Update obstacle history for dynamic obstacle detection"""
        if not ranges or not angles:
            return

        rx, ry, ryaw = robot_pose
        current_obstacles = []

        # Convert laser readings to world coordinates
        for i, (range_val, angle) in enumerate(zip(ranges, angles)):
            if 0.1 < range_val < 5.0:  # Reasonable range
                # Convert to robot coordinates
                local_x = range_val * math.cos(angle)
                local_y = range_val * math.sin(angle)

                # Convert to world coordinates
                world_x = rx + local_x * math.cos(ryaw) - local_y * math.sin(ryaw)
                world_y = ry + local_x * math.sin(ryaw) + local_y * math.cos(ryaw)

                current_obstacles.append({
                    'x': world_x,
                    'y': world_y,
                    'range': range_val,
                    'angle': angle,
                    'timestamp': time.time()
                })

        # Update history
        self.obstacle_history.append({
            'timestamp': time.time(),
            'robot_pose': robot_pose,
            'obstacles': current_obstacles
        })

        # Keep only recent history
        if len(self.obstacle_history) > self.max_history_size:
            self.obstacle_history.pop(0)

    def detect_dynamic_obstacles(self):
        """Detect dynamic obstacles by analyzing history"""
        if len(self.obstacle_history) < 2:
            return []

        dynamic_obstacles = []
        current = self.obstacle_history[-1]
        prev = self.obstacle_history[-2]

        # Simple correlation-based detection
        for curr_obs in current['obstacles']:
            min_distance = float('inf')
            closest_prev = None

            for prev_obs in prev['obstacles']:
                dist = math.hypot(
                    curr_obs['x'] - prev_obs['x'],
                    curr_obs['y'] - prev_obs['y']
                )
                if dist < min_distance:
                    min_distance = dist
                    closest_prev = prev_obs

            if closest_prev and min_distance < 0.5:  # Correlation threshold
                # Calculate velocity
                time_diff = current['timestamp'] - prev['timestamp']
                if time_diff > 0:
                    velocity = min_distance / time_diff
                    if velocity > self.dynamic_obstacle_threshold:
                        dynamic_obstacles.append({
                            'x': curr_obs['x'],
                            'y': curr_obs['y'],
                            'velocity': velocity,
                            'direction': math.atan2(
                                curr_obs['y'] - closest_prev['y'],
                                curr_obs['x'] - closest_prev['x']
                            )
                        })

        return dynamic_obstacles

    def generate_obstacle_aware_samples(self, ranges, angles, pose, dynamic_obstacles):
        """Generate velocity samples considering obstacles"""
        # Adaptive sampling based on obstacle density
        obstacle_density = self.calculate_obstacle_density(ranges, angles)

        if obstacle_density > 0.7:
            # High obstacle density - conservative sampling
            v_samples = np.linspace(self.min_v, self.max_v * 0.4, self.v_samples // 2)
            w_samples = np.linspace(-self.max_w * 0.8, self.max_w * 0.8, self.w_samples // 2)
        elif obstacle_density > 0.3:
            # Medium obstacle density - moderate sampling
            v_samples = np.linspace(self.min_v, self.max_v * 0.7, self.v_samples)
            w_samples = np.linspace(-self.max_w * 0.9, self.max_w * 0.9, self.w_samples)
        else:
            # Low obstacle density - aggressive sampling
            v_samples = np.linspace(self.min_v * 0.5, self.max_v, self.v_samples)
            w_samples = np.linspace(-self.max_w, self.max_w, self.w_samples)

        # Remove samples that would immediately collide
        filtered_v = []
        filtered_w = []

        for v in v_samples:
            for w in w_samples:
                if self.is_command_feasible(v, w, ranges, angles):
                    filtered_v.append(v)
                    filtered_w.append(w)

        # Ensure we have enough samples
        if len(filtered_v) < 5:
            filtered_v = v_samples
            filtered_w = w_samples

        return filtered_v, filtered_w

    def calculate_obstacle_density(self, ranges, angles):
        """Calculate obstacle density in front sector"""
        if not ranges or not angles:
            return 0.0

        front_indices = []
        for i, angle in enumerate(angles):
            if abs(angle) < math.radians(60):  # 120-degree front sector
                front_indices.append(i)

        if not front_indices:
            return 0.0

        close_obstacles = 0
        for i in front_indices:
            if 0.1 < ranges[i] < self.safe_distance * 2:
                close_obstacles += 1

        return close_obstacles / len(front_indices)

    def is_command_feasible(self, v, w, ranges, angles):
        """Check if command is immediately feasible"""
        # Basic feasibility
        if abs(v) < 0.01 and abs(w) > 1.0:
            return False

        # Check if immediate motion would collide
        if ranges is not None and angles is not None:
            # Simulate first step
            if v > 0:
                distance = v * self.dt
                # Check if any obstacle in immediate path
                for i, (range_val, angle) in enumerate(zip(ranges, angles)):
                    if abs(angle) < math.radians(30):  # Narrow front cone
                        if 0.1 < range_val < distance + self.robot_radius:
                            return False

        return True

    def get_dynamic_lookahead_target(self, pose, path, scan_ranges):
        """Get dynamic lookahead target based on obstacles"""
        if path is None or len(path.poses) == 0:
            # Default target straight ahead
            rx, ry, ryaw = pose
            return rx + math.cos(ryaw), ry + math.sin(ryaw)

        # Adaptive lookahead based on speed and obstacles
        lookahead_base = 1.0
        if scan_ranges is not None:
            # Reduce lookahead when obstacles are close
            min_range = min([r for r in scan_ranges if r > 0.1] + [10.0])
            lookahead_factor = min(1.0, min_range / self.safe_distance)
            lookahead_base *= lookahead_factor

        # Find lookahead point on path
        rx, ry, _ = pose
        closest_dist = float('inf')
        closest_idx = 0

        for i, path_pose in enumerate(path.poses):
            px = path_pose.pose.position.x
            py = path_pose.pose.position.y
            dist = math.hypot(px - rx, py - ry)

            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        # Look ahead
        lookahead_idx = min(closest_idx + int(lookahead_base * 10), len(path.poses) - 1)
        target = path.poses[lookahead_idx].pose.position

        return target.x, target.y

    def calculate_clearance_score(self, safety_score, closest_obstacle):
        """Calculate clearance score"""
        if closest_obstacle == float('inf'):
            return 1.0

        normalized_distance = min(1.0, closest_obstacle / self.safe_distance)
        return safety_score * normalized_distance

    def calculate_heading_score(self, pose, tx, ty):
        """Calculate heading alignment score"""
        x, y, yaw = pose
        desired_yaw = math.atan2(ty - y, tx - x)
        angle_diff = abs(math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw)))
        return max(0.0, 1.0 - angle_diff / math.pi)

    def calculate_progress_score(self, start_pose, end_pose, tx, ty, path):
        """Calculate progress toward target"""
        sx, sy, _ = start_pose
        ex, ey, _ = end_pose

        start_dist = math.hypot(tx - sx, ty - sy)
        end_dist = math.hypot(tx - ex, ty - ey)

        progress = start_dist - end_dist
        return max(0.0, progress / (self.max_v * self.dt * 5))

    def calculate_speed_score(self, v, safety_score):
        """Calculate speed score with safety consideration"""
        normalized_speed = v / self.max_v
        return normalized_speed * safety_score

    def calculate_smoothness_score(self, v, w):
        """Calculate smoothness score"""
        v_score = 1.0 - abs(v - self.last_best_v) / self.max_v
        w_score = 1.0 - abs(w - self.last_best_w) / self.max_w
        return (v_score + w_score) / 2.0

    def calculate_obstacle_free_score(self, traj, ranges, angles, robot_pose):
        """Calculate how obstacle-free the trajectory is"""
        if not ranges or not angles:
            return 1.0

        rx, ry, ryaw = robot_pose
        free_points = 0
        total_points = len(traj)

        for x, y, _ in traj:
            # Convert to robot frame
            dx = x - rx
            dy = y - ry
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx) - ryaw

            # Find closest laser reading
            min_range = float('inf')
            for range_val, laser_angle in zip(ranges, angles):
                if 0.1 < range_val < self.obstacle_influence_radius:
                    angle_diff = abs(laser_angle - angle)
                    if angle_diff < math.radians(15):
                        min_range = min(min_range, range_val)

            if min_range > dist + self.robot_radius:
                free_points += 1

        return free_points / total_points if total_points > 0 else 0.0

    def calculate_path_alignment_score(self, traj, path):
        """Calculate alignment with global path"""
        if path is None or len(path.poses) < 2:
            return 1.0

        alignment_sum = 0.0
        count = 0

        for x, y, _ in traj:
            # Find closest path segment
            min_distance = float('inf')
            for i in range(len(path.poses) - 1):
                p1 = path.poses[i].pose.position
                p2 = path.poses[i + 1].pose.position

                # Distance from point to line segment
                dist = self.point_to_segment_distance(x, y, p1.x, p1.y, p2.x, p2.y)
                if dist < min_distance:
                    min_distance = dist

            alignment = 1.0 / (1.0 + min_distance)
            alignment_sum += alignment
            count += 1

        return alignment_sum / count if count > 0 else 0.0

    def point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        """Calculate distance from point to line segment"""
        # Vector from p1 to p2
        vx = x2 - x1
        vy = y2 - y1

        # Vector from p1 to point
        wx = px - x1
        wy = py - y1

        # Projection factor
        c1 = wx * vx + wy * vy
        if c1 <= 0:
            return math.hypot(px - x1, py - y1)

        c2 = vx * vx + vy * vy
        if c2 <= c1:
            return math.hypot(px - x2, py - y2)

        # Projection point
        b = c1 / c2
        bx = x1 + b * vx
        by = y1 + b * vy

        return math.hypot(px - bx, py - by)

    def calculate_dynamic_weights(self, safety_score, closest_obstacle, dynamic_obstacles):
        """Calculate dynamic weights based on environment"""
        weights = {
            'clearance': self.w_clearance,
            'heading': self.w_heading,
            'progress': self.w_progress,
            'speed': self.w_speed,
            'smoothness': self.w_smoothness,
            'obstacle_free': self.w_obstacle_free,
            'path_alignment': self.w_path_alignment
        }

        # Adjust based on safety
        if safety_score < 0.3:
            # Dangerous situation - prioritize safety
            weights['clearance'] *= 2.0
            weights['obstacle_free'] *= 2.0
            weights['speed'] *= 0.3
        elif safety_score > 0.8:
            # Safe situation - prioritize progress
            weights['progress'] *= 1.5
            weights['speed'] *= 1.5

        # Adjust for dynamic obstacles
        if dynamic_obstacles:
            weights['obstacle_free'] *= 2.0
            weights['clearance'] *= 1.5

        return weights

    def evaluate_pinn_score(self, traj, v, w):
        """Evaluate trajectory using PINN if available"""
        # This is a placeholder - implement actual PINN integration
        # For now, return a simple heuristic
        if not self.use_pinn:
            return 1.0

        # Simple heuristic: prefer smoother trajectories
        accel = abs(v - self.last_best_v) / self.dt
        jerk = abs(w - self.last_best_w) / self.dt

        smoothness = 1.0 / (1.0 + accel * 0.1 + jerk * 0.05)
        return smoothness

    def intelligent_fallback(self, pose, scan_ranges, scan_angles, path):
        """Intelligent fallback when no feasible trajectory found"""
        rx, ry, ryaw = pose

        # Handle None scan data
        if not scan_ranges or not scan_angles:
            return 0.0, 0.0

        # Try to find safest direction
        best_direction = 0.0
        best_clearance = 0.0

        # Scan all directions for best clearance
        for i, (range_val, angle) in enumerate(zip(scan_ranges, scan_angles)):
            if range_val > best_clearance:
                best_clearance = range_val
                best_direction = angle

        # Determine action based on clearance
        if best_clearance > self.safe_distance * 3:
            # Good clearance - move slowly forward
            v = self.min_v
            w = best_direction * 0.5
        elif best_clearance > self.safe_distance:
            # Moderate clearance - turn toward clear space
            v = self.min_v * 0.5
            w = best_direction * 0.8
        else:
            # Poor clearance - rotate in place
            v = 0.0
            w = 0.3 if best_direction > 0 else -0.3

        self.node.get_logger().info(f"Fallback: v={v:.2f}, w={w:.2f}, clearance={best_clearance:.2f}m")
        return float(v), float(w)
