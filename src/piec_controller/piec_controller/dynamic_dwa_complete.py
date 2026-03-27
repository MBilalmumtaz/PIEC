#!/usr/bin/env python3
"""
Dynamic DWA Complete Planner with Enhanced Obstacle Avoidance and Free Space Awareness
"""

import numpy as np
import math
import time

# Progressive rotation threshold constants (in meters and radians)
# FIX: Increased thresholds to allow turning while moving for lateral/rear goals
# Previous values (45/30/15) were too restrictive - robot would spin for goals >45° away
# New values (120/90/60) match controller_node.py rotate_in_place_angle_deg setting
ROTATION_THRESHOLD_FAR_DISTANCE = 2.0  # Distance threshold for far range (meters)
ROTATION_THRESHOLD_MID_DISTANCE = 0.5  # Distance threshold for mid range (meters)
ROTATION_THRESHOLD_FAR_ANGLE = 90      # Rotation threshold when far from goal (degrees)
ROTATION_THRESHOLD_MID_ANGLE = 60       # Rotation threshold in mid range (degrees)
ROTATION_THRESHOLD_CLOSE_ANGLE = 30     # Rotation threshold when close to goal (degrees)

# Maximum rotation time before forcing forward motion (seconds)
MAX_ROTATION_TIME = 5.0

# Fallback rotation scaling constants
_FALLBACK_W_SCALE = 0.5       # fraction of max_w used in forced/fallback rotations
_FALLBACK_HEADING_GAIN = 0.8  # proportional gain on heading error for rotation magnitude


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

        # ── DWA Scoring weights (GWO-DWA tuned, 2024 approach) ──────────────
        # Grey-Wolf Optimizer surveys (Ref: GWO-DWA, IEEE 2024) show that
        # heading alignment and goal proximity should dominate over path-proximity
        # when the heading error is large, preventing "follows path backwards".
        #
        # heading    – end-orientation vs goal direction (dominant term)
        # goal       – proximity to goal at trajectory end
        # speed      – prefer non-zero velocities
        # clearance  – obstacle safety margin (safety constraint)
        # path       – lateral deviation from planned path (forward WPs only)
        # free_space – open-area preference (tie-breaker)
        self.w_heading = 6.0     # raised from 5.0 for stronger goal alignment
        self.w_goal = 4.5        # raised from 4.0
        self.w_speed = 0.8       # raised from 0.5 to discourage v=0, w=0
        self.w_clearance = 3.0
        self.w_path = 2.0        # reduced: path score can mislead when path is stale
        self.w_free_space = 0.3

        # Heading error thresholds controlling velocity sample generation
        self.ROTATE_ONLY_ANGLE_RAD = math.radians(120)  # rotate-only above this
        self.PREFER_ROTATE_ANGLE_RAD = math.radians(30)  # include v=0 above this
        
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
        
        # Braking model constants (used in check_trajectory safety margin computation)
        self.brake_decel = 0.6   # m/s² conservative deceleration
        self.cmd_latency = 0.2   # s command-to-wheel latency margin

        # Cached nearest-path-waypoint index from the previous planning cycle.
        # We search forward from this cache instead of scanning the whole path
        # from scratch every cycle, reducing the per-cycle cost to O(k) where k
        # is the number of waypoints advanced since the last call.
        self._cached_nearest_path_idx = 0

        # Consecutive low-score fallback: if DWA selects a trajectory with score
        # below this epsilon for N consecutive cycles, fall back to pure rotation
        # toward the goal direction to break local minima.
        self._low_score_threshold = -1.0   # scores below this are considered "no progress"
        self._low_score_cycle_count = 0
        self._low_score_max_cycles = 5     # consecutive low-score cycles before fallback
        self._low_score_fallback_active = False

        self.node.get_logger().info("Dynamic DWA Complete initialized with Free Space Awareness")

    def _get_min_forward_dist(self, scan_ranges, scan_angles, cone_half_angle=math.radians(45)):
        """Return minimum valid scan range within a forward cone in robot frame.

        Args:
            scan_ranges: list of range values (metres)
            scan_angles: list of beam angles in robot frame (radians, already in [-π, π])
            cone_half_angle: half-width of the forward cone (default ±45°)
        Returns:
            Minimum range in the cone, or float('inf') if no valid readings exist.
        """
        if scan_ranges is None or scan_angles is None:
            return float('inf')
        min_dist = float('inf')
        for r, a in zip(scan_ranges, scan_angles):
            if abs(a) < cone_half_angle:
                if math.isfinite(r) and 0.1 < r < min_dist:
                    min_dist = r
        return min_dist

    def plan(self, current_pose, path, scan_ranges, scan_angles):
        """Main planning function with frame-correct heading and progress scoring"""
        if path is None or len(path.poses) == 0:
            return 0.0, 0.0
            
        if scan_ranges is None or scan_angles is None or len(scan_ranges) == 0:
            return self.fallback_control(current_pose, path)
        
        try:
            # Get current state
            x, y, theta = current_pose

            # Goal position (world frame)
            goal_x = path.poses[-1].pose.position.x
            goal_y = path.poses[-1].pose.position.y
            # Goal direction in world frame – used for heading scoring
            goal_dir_world = math.atan2(goal_y - y, goal_x - x)

            # Heading error from current robot orientation to goal direction
            heading_error_to_goal = abs(math.atan2(
                math.sin(goal_dir_world - theta),
                math.cos(goal_dir_world - theta)
            ))

            # Distance to goal at trajectory START (for progress guardrail)
            dist_to_goal_start = math.hypot(goal_x - x, goal_y - y)

            # Minimum forward obstacle distance (for progress penalty gating)
            min_forward_dist = self._get_min_forward_dist(scan_ranges, scan_angles)

            # Find nearest forward path waypoint index so calc_path_score rewards
            # only waypoints ahead of the robot's current position on the path.
            # We search forward from the cached index (not from 0) to keep the
            # per-cycle cost to O(k) rather than O(n).
            n_poses = len(path.poses)
            search_start = min(self._cached_nearest_path_idx, max(0, n_poses - 1))
            nearest_path_idx = search_start
            nearest_path_dist = math.hypot(
                path.poses[search_start].pose.position.x - x,
                path.poses[search_start].pose.position.y - y
            )
            for _pi in range(search_start, n_poses):
                _pd = math.hypot(path.poses[_pi].pose.position.x - x,
                                 path.poses[_pi].pose.position.y - y)
                if _pd < nearest_path_dist:
                    nearest_path_dist = _pd
                    nearest_path_idx = _pi
                elif _pd > nearest_path_dist + 0.5:
                    # Waypoints are getting further away – robot has passed the
                    # nearest one, stop the search early.
                    break
            self._cached_nearest_path_idx = nearest_path_idx

            # Dynamic velocity limits based on obstacles
            dynamic_max_v, dynamic_max_w = self.calculate_dynamic_limits(scan_ranges, scan_angles, theta)

            # ── Velocity sample generation ─────────────────────────────────
            # When heading error is very large (>120°) the robot is essentially
            # facing backwards relative to the goal.  Forcing forward motion in
            # that situation causes circular drift.  Use ONLY pure-rotation
            # candidates so the robot first aligns with the goal, then drives.
            #
            # Between 30° and 120° we include both forward and pure-rotation so
            # DWA can select the best trade-off.
            v_forward = np.arange(self.min_v, dynamic_max_v + self.v_resolution, self.v_resolution)
            if heading_error_to_goal > self.ROTATE_ONLY_ANGLE_RAD:
                # Rotate in place only – no forward samples
                v_samples = np.array([0.0])
            elif heading_error_to_goal > self.PREFER_ROTATE_ANGLE_RAD:
                v_samples = np.concatenate(([0.0], v_forward))
            else:
                v_samples = v_forward

            w_samples = np.arange(-dynamic_max_w, dynamic_max_w + self.w_resolution, self.w_resolution)

            if len(v_samples) == 0 or len(w_samples) == 0:
                return self.fallback_control(current_pose, path)

            # Evaluate trajectories
            best_trajectory = None
            best_score = -float('inf')
            best_v = 0.0
            best_w = 0.0
            best_scores_dbg = None  # for diagnostics
            
            for v in v_samples:
                for w in w_samples:
                    # Feasibility filter: skip dynamically infeasible combinations.
                    # For v=0 (pure rotation) allow full angular range – we do not
                    # apply the speed-ratio filter so the robot can spin freely.
                    if v > 0 and abs(w) > 1.5 * abs(v) + 0.5:
                        continue
                    
                    # Generate trajectory
                    trajectory = self.generate_trajectory(x, y, theta, v, w)
                    
                    # Check if trajectory is collision-free
                    if not self.check_trajectory(trajectory, scan_ranges, scan_angles, theta):
                        continue
                    
                    # Calculate scores
                    heading_score = self.calc_heading_score(trajectory, goal_dir_world)
                    goal_score = self.calc_goal_score(trajectory, path, dist_to_goal_start)
                    speed_score = self.calc_speed_score(v, w)
                    clearance_score = self.calc_clearance_score(trajectory, scan_ranges, scan_angles, theta)
                    path_score = self.calc_path_score(trajectory, path, nearest_path_idx)
                    free_space_score = self.calc_free_space_score(
                        trajectory, scan_ranges, scan_angles, theta, goal_dir_world)

                    # Progress penalty: discourage staying still (v≈0) when the goal
                    # is far and no obstacle blocks the forward path.  Without this,
                    # the robot may rotate in place indefinitely in open space instead
                    # of making forward progress.
                    # Thresholds: v<0.05 m/s ≈ stationary; goal >1 m away; forward
                    # clearance >1 m (no imminent obstacle).
                    # Penalty increased to -3.0 to overcome path-score bias for
                    # trajectories that follow the path without making goal progress.
                    progress_penalty = 0.0
                    if v < 0.05 and dist_to_goal_start > 1.0 and min_forward_dist > 1.0:
                        progress_penalty = -3.0

                    # Weighted total score
                    total_score = (
                        self.w_heading    * heading_score +
                        self.w_goal       * goal_score +
                        self.w_speed      * speed_score +
                        self.w_clearance  * clearance_score +
                        self.w_path       * path_score +
                        self.w_free_space * free_space_score +
                        progress_penalty
                    )
                    
                    if total_score > best_score:
                        best_score = total_score
                        best_v = v
                        best_w = w
                        best_trajectory = trajectory
                        best_scores_dbg = (heading_score, goal_score, speed_score,
                                           clearance_score, path_score, free_space_score)
            
            # If no valid trajectory found, use fallback.
            # For large heading errors with no safe trajectory, issue a
            # forced in-place rotation so the robot can align before moving.
            if best_trajectory is None:
                if heading_error_to_goal > self.PREFER_ROTATE_ANGLE_RAD:
                    # Determine rotation sign from heading error sign
                    heading_diff = math.atan2(
                        math.sin(goal_dir_world - theta),
                        math.cos(goal_dir_world - theta)
                    )
                    forced_w = math.copysign(
                        min(self.max_w * _FALLBACK_W_SCALE, abs(heading_diff) * 0.7),
                        heading_diff
                    )
                    if hasattr(self.node, 'debug_mode') and self.node.debug_mode:
                        self.node.get_logger().warn(
                            f'⚠️ DWA no valid traj, forced rotation: '
                            f'heading_err={math.degrees(heading_error_to_goal):.1f}° '
                            f'→ w={forced_w:.3f}'
                        )
                    return 0.0, forced_w
                return self.fallback_control(current_pose, path)

            # ── Consecutive low-score fallback ──────────────────────────────
            # If the winning trajectory score is persistently near its floor
            # (robot stuck in local minimum), override with pure rotation.
            if best_score < self._low_score_threshold:
                self._low_score_cycle_count += 1
            else:
                self._low_score_cycle_count = 0
                self._low_score_fallback_active = False

            if self._low_score_cycle_count >= self._low_score_max_cycles:
                heading_diff = math.atan2(
                    math.sin(goal_dir_world - theta),
                    math.cos(goal_dir_world - theta)
                )
                forced_w = math.copysign(
                    min(self.max_w * _FALLBACK_W_SCALE, abs(heading_diff) * _FALLBACK_HEADING_GAIN),
                    heading_diff
                )
                if not self._low_score_fallback_active:
                    self._low_score_fallback_active = True
                    if hasattr(self.node, 'debug_mode') and self.node.debug_mode:
                        self.node.get_logger().warn(
                            f'🔄 DWA low-score fallback ({self._low_score_cycle_count} cycles): '
                            f'score={best_score:.3f} → pure rotation w={forced_w:.3f}'
                        )
                return 0.0, forced_w

            # Throttled diagnostics
            dbg = (hasattr(self.node, 'debug_mode') and self.node.debug_mode and
                   hasattr(self.node, 'control_counter') and self.node.control_counter % 50 == 0)
            if dbg and best_scores_dbg is not None:
                h, g, sp, cl, pa, fs = best_scores_dbg
                goal_dir_deg = math.degrees(goal_dir_world)
                heading_err_deg = math.degrees(heading_error_to_goal)
                traj_dir_deg = math.degrees(math.atan2(
                    best_trajectory[-1][1] - best_trajectory[0][1],
                    best_trajectory[-1][0] - best_trajectory[0][0]))
                self.node.get_logger().info(
                    f"🎯 DWA selected v={best_v:.3f} w={best_w:.3f} | "
                    f"goal_dist={dist_to_goal_start:.2f}m heading_err={heading_err_deg:.1f}° "
                    f"goal_dir={goal_dir_deg:.1f}° traj_dir={traj_dir_deg:.1f}°"
                )
                self.node.get_logger().info(
                    f"   scores: heading={h:.3f} goal={g:.3f} speed={sp:.3f} "
                    f"clearance={cl:.3f} path={pa:.3f} free_space={fs:.3f}"
                )
            
            return best_v, best_w
            
        except Exception as e:
            if hasattr(self.node, 'debug_mode') and self.node.debug_mode:
                self.node.get_logger().warn(f"DWA planning error: {e}")
            return self.fallback_control(current_pose, path)

    # --------------------------------------------------------------------------
    # The rest of the methods (calculate_dynamic_limits, generate_trajectory,
    # check_trajectory, calc_goal_score, calc_speed_score, calc_clearance_score,
    # calc_path_score, calc_free_space_score, get_clearance_at, get_clearance_in_direction_scan,
    # calc_min_clearance, fallback_control) remain unchanged.
    # Please keep your existing implementations for these methods.
    # --------------------------------------------------------------------------

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
            # Do NOT zero out max_w: allow rotation so the robot can escape by
            # turning away from the obstacle.  The emergency_stop node already
            # blocks forward motion; keeping max_w non-zero lets DWA select a
            # pure-rotation trajectory that brings the robot to face clear space.
            max_w = min(max_w, 0.3)
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
        """Check if trajectory is collision-free with conservative checking.

        Safety margin = robot_radius + 0.10 m base + braking distance for the
        maximum speed commanded (approximated from trajectory arc length / time).
        """
        if not trajectory:
            return False

        # Estimate speed from arc length of trajectory
        arc_len = sum(
            math.hypot(trajectory[i][0] - trajectory[i-1][0],
                       trajectory[i][1] - trajectory[i-1][1])
            for i in range(1, len(trajectory))
        )
        est_speed = arc_len / max(self.sim_time, 1e-6)

        # Braking distance: v²/(2·a) + v·latency
        braking_dist = (est_speed ** 2) / (2.0 * self.brake_decel) + est_speed * self.cmd_latency
        safety_margin = max(0.15, braking_dist + 0.10)  # at least 0.15 m base

        # Check multiple points along trajectory (skip index 0 – robot centre)
        step = max(1, len(trajectory) // 8)
        for idx in range(1, len(trajectory), step):
            x, y, theta = trajectory[idx]
            
            # Convert point to robot frame
            dx = x - trajectory[0][0]
            dy = y - trajectory[0][1]
            
            local_x = dx * math.cos(-robot_theta) - dy * math.sin(-robot_theta)
            local_y = dx * math.sin(-robot_theta) + dy * math.cos(-robot_theta)
            
            dist = math.hypot(local_x, local_y)
            angle = math.atan2(local_y, local_x)

            if not self.check_collision(dist, angle, scan_ranges, scan_angles,
                                        safety_margin=safety_margin):
                return False
        
        return True

    def check_collision(self, dist, angle, scan_ranges, scan_angles, safety_margin=0.05):
        """
        Check for collision at a specific robot-frame point with safety margin.
        Returns True if no collision (safe), False if collision detected.

        IMPORTANT – scan_angles must be in sensor/robot frame (same as `angle`).
        Uses normalized angle difference to avoid wrap-around artefacts.
        Invalid scan readings (NaN, Inf, or below 0.1 m) in the queried direction
        are treated conservatively: if the queried direction is within a forward ±90°
        cone and the reading is invalid/near-range, we assume something is close.
        """
        if scan_ranges is None or scan_angles is None:
            return True  # No scan data – assume safe

        # Find closest scan angle using normalized difference
        min_angle_diff = float('inf')
        closest_idx = 0
        for i, scan_angle in enumerate(scan_angles):
            # Normalize to [-π, π] before comparing
            a_diff = abs(math.atan2(math.sin(scan_angle - angle),
                                     math.cos(scan_angle - angle)))
            if a_diff < min_angle_diff:
                min_angle_diff = a_diff
                closest_idx = i

        if closest_idx < len(scan_ranges):
            scan_dist = scan_ranges[closest_idx]

            # Conservative handling of invalid readings:
            # If the best-matching scan angle is within ±90° and the reading is
            # suspicious (NaN, Inf, or very small), treat it as occupied.
            if min_angle_diff < math.pi / 2:
                if not math.isfinite(scan_dist) or scan_dist < 0.1:
                    # Unknown / sensor blind-spot → treat as just-at-range_min distance
                    # This prevents driving into sensor dead-zones.
                    return False

            # Normal collision check: obstacle closer than trajectory point + clearance?
            if 0.1 < scan_dist < dist + self.robot_radius + safety_margin:
                return False  # Collision!

        return True

    def calc_heading_score(self, trajectory, goal_dir_world):
        """
        Score how well the trajectory end-direction aligns with the world-frame
        goal direction.  This is the primary fix for the 'robot drifts wrong
        direction' bug: previously DWA had no explicit heading term so it could
        pick trajectories facing away from the goal.

        For v≈0 (pure rotation) trajectories the robot barely moves, so the
        start→end displacement is tiny and directionally unreliable.  In that
        case we score by how well the robot's END ORIENTATION (theta) aligns
        with the goal direction – this correctly rewards rotations that bring
        the robot to face the goal.

        Returns a score in [0, 1] where 1 = perfect alignment.
        """
        if not trajectory or len(trajectory) < 2:
            return 0.0

        start_x, start_y, _ = trajectory[0]
        end_x,   end_y, end_theta = trajectory[-1]

        # Only compute travel direction if the trajectory moved a meaningful distance.
        # Below ~3 cm the start→end vector is directionally unreliable (pure rotation
        # or near-zero v), so we fall back to scoring by end robot orientation.
        MIN_MOVEMENT_FOR_DIR = 0.03  # metres
        move_dist = math.hypot(end_x - start_x, end_y - start_y)
        if move_dist < MIN_MOVEMENT_FOR_DIR:
            # Pure/near-rotation: score by end robot orientation vs goal direction.
            heading_error = abs(math.atan2(
                math.sin(end_theta - goal_dir_world),
                math.cos(end_theta - goal_dir_world)
            ))
            return max(0.0, 1.0 - heading_error / math.pi)

        traj_dir_world = math.atan2(end_y - start_y, end_x - start_x)
        heading_error = abs(math.atan2(math.sin(traj_dir_world - goal_dir_world),
                                        math.cos(traj_dir_world - goal_dir_world)))
        # Score: 1 when aligned (error=0), 0 when opposite (error=π)
        return max(0.0, 1.0 - heading_error / math.pi)

    def calc_goal_score(self, trajectory, path, dist_to_goal_start=None):
        """Calculate goal-reaching score with progress guardrail"""
        if not trajectory or not path.poses:
            return 0.0
        
        end_x, end_y, _ = trajectory[-1]
        start_x, start_y, _ = trajectory[0]

        goal_x = path.poses[-1].pose.position.x
        goal_y = path.poses[-1].pose.position.y

        # Distance from trajectory END to final goal
        end_to_goal = math.hypot(goal_x - end_x, goal_y - end_y)

        # Distance from trajectory START to final goal
        if dist_to_goal_start is None:
            dist_to_goal_start = math.hypot(goal_x - start_x, goal_y - start_y)

        # Progress: positive if trajectory moves TOWARD goal
        progress = dist_to_goal_start - end_to_goal  # positive = good

        # Proximity score (higher when trajectory ends near goal)
        proximity_score = 1.0 / (1.0 + end_to_goal)

        # Progress score (normalised, clipped so only forward progress is rewarded)
        progress_score = max(0.0, progress) / max(dist_to_goal_start, 0.1)

        return 0.5 * proximity_score + 0.5 * progress_score

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
        """Get clearance at a specific point in robot frame using normalized angle diff"""
        if scan_ranges is None or scan_angles is None:
            return 1.0
        
        # Find closest scan angle using normalized difference
        min_angle_diff = float('inf')
        closest_idx = 0
        
        for i, scan_angle in enumerate(scan_angles):
            a_diff = abs(math.atan2(math.sin(scan_angle - angle),
                                     math.cos(scan_angle - angle)))
            if a_diff < min_angle_diff:
                min_angle_diff = a_diff
                closest_idx = i
        
        if closest_idx < len(scan_ranges):
            scan_dist = scan_ranges[closest_idx]
            if math.isfinite(scan_dist) and scan_dist > 0.1:
                return scan_dist - dist - self.robot_radius
        
        return 2.0  # Default large clearance

    def get_clearance_in_direction_scan(self, scan_ranges, scan_angles, robot_theta, world_angle):
        """
        Get clearance in a world-frame direction using scan data.
        Converts world_angle to sensor/robot frame before comparing with scan_angles.
        """
        if scan_ranges is None or scan_angles is None:
            return float('inf')

        # Convert world angle to robot/sensor frame
        local_angle = world_angle - robot_theta
        local_angle = math.atan2(math.sin(local_angle), math.cos(local_angle))

        # Find closest scan angle using normalized difference
        min_diff = float('inf')
        closest_range = float('inf')

        for scan_angle, scan_range in zip(scan_angles, scan_ranges):
            diff = abs(math.atan2(math.sin(scan_angle - local_angle),
                                   math.cos(scan_angle - local_angle)))
            if diff < min_diff and math.isfinite(scan_range) and scan_range > 0.1:
                min_diff = diff
                closest_range = scan_range

        return closest_range

    def calc_path_score(self, trajectory, path, nearest_path_idx=0):
        """Calculate path following score – only forward waypoints are rewarded.

        Scoring only path waypoints at or ahead of ``nearest_path_idx`` prevents
        trajectories that move *backward* along the path (toward already-passed
        waypoints) from receiving an inflated path score.  This is the key fix
        for the "robot follows path in wrong direction" failure mode.
        """
        if not trajectory or not path.poses:
            return 0.0
        
        total_score = 0.0
        count = 0
        
        # Only consider path waypoints from nearest_path_idx onward (forward).
        # If nearest_path_idx >= len(path.poses) the robot has reached the end
        # of the path – return 0.0 so no trajectory is artificially rewarded.
        if nearest_path_idx >= len(path.poses):
            return 0.0
        forward_poses = path.poses[nearest_path_idx:]
        
        # Sample trajectory points
        traj_step = max(1, len(trajectory) // 5)
        for i in range(0, len(trajectory), traj_step):
            x, y, _ = trajectory[i]
            
            # Find closest FORWARD path point
            min_dist = float('inf')
            for pose in forward_poses:
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
        
        # FIX: Progressive rotation threshold - increased values to allow turning while moving
        # Far from goal (>2m): 120°, Mid-range (0.5-2m): 90°, Close (<0.5m): 60°
        # Only rotate in place for extreme angles (>120°), otherwise turn while moving
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
