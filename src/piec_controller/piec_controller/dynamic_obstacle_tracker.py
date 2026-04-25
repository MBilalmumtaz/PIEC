#!/usr/bin/env python3
"""
Dynamic Obstacle Tracker with Candidate Stage and Robust Static Detection.

Improvements:
- New clusters become CANDIDATES first (not dynamic).
- Promotion to DYNAMIC only after sustained displacement (>0.15m over 1s)
  AND change in distance-to-robot (>0.08m) AND speed >0.08m/s.
- When robot moves faster, dynamic threshold increases (0.25m).
- All existing features (threat assessment, evasion, collision prediction, safe distance) preserved.
"""

import math
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
import time

@dataclass
class Obstacle:
    """Represents a detected obstacle with velocity estimation and threat analysis."""
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.15
    confidence: float = 1.0
    last_updated: float = 0.0
    history: deque = field(default_factory=lambda: deque(maxlen=10))
    is_static: bool = False
    static_since: float = 0.0
    low_speed_frames: int = 0
    created_at: float = 0.0
    num_updates: int = 0
    _last_meas_time: float = 0.0

    def __post_init__(self):
        now = time.time()
        self.history.append((self.x, self.y, now))
        self.last_updated = now
        self.created_at = now
        self.num_updates = 0
        self._last_meas_time = now

    def predict_position(self, dt: float) -> Tuple[float, float]:
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update_velocity(self):
        """
        Velocity estimate + static classification.
        Uses robust linear regression on history.
        """
        if len(self.history) < 3:
            return

        times = [t for _, _, t in self.history]
        xs = [x for x, _, _ in self.history]
        ys = [y for y, _, _ in self.history]
        n = len(times)

        t_mean = sum(times) / n
        x_mean = sum(xs) / n
        y_mean = sum(ys) / n

        den = sum((times[i] - t_mean) ** 2 for i in range(n))
        if den > 0:
            num_x = sum((times[i] - t_mean) * (xs[i] - x_mean) for i in range(n))
            num_y = sum((times[i] - t_mean) * (ys[i] - y_mean) for i in range(n))
            self.vx = num_x / den
            self.vy = num_y / den
        else:
            self.vx = 0.0
            self.vy = 0.0

        max_speed = 2.0
        speed = math.hypot(self.vx, self.vy)
        if speed > max_speed:
            self.vx = self.vx / speed * max_speed
            self.vy = self.vy / speed * max_speed
            speed = max_speed

        # Static classification (low speed OR low jitter)
        low_speed = speed < 0.05

        x_std = float(np.std(xs)) if n >= 3 else 999.0
        y_std = float(np.std(ys)) if n >= 3 else 999.0
        jitter = math.hypot(x_std, y_std)
        low_jitter = jitter < 0.03

        if low_speed or low_jitter:
            if not self.is_static:
                if self.static_since == 0.0:
                    self.static_since = time.time()
                elif time.time() - self.static_since > 1.0:
                    self.is_static = True
        else:
            self.is_static = False
            self.static_since = 0.0

    def update(self, x: float, y: float, confidence: float = 1.0, meas_time: float = None):
        now = meas_time if meas_time is not None else time.time()
        self.history.append((x, y, now))
        self.x = x
        self.y = y
        self.confidence = confidence
        self.last_updated = now
        self.num_updates += 1
        self._last_meas_time = now
        self.update_velocity()

    def get_speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def get_direction(self) -> float:
        return math.atan2(self.vy, self.vx)

    def get_age(self) -> float:
        return time.time() - self.created_at

    def get_position_jitter(self) -> float:
        """Std-dev based jitter magnitude over history."""
        if len(self.history) < 3:
            return 999.0
        xs = [x for x, _, _ in self.history]
        ys = [y for _, y, _ in self.history]
        return float(math.hypot(np.std(xs), np.std(ys)))

    def get_displacement_over(self, window_sec: float = 1.0) -> float:
        """
        Displacement between 'now' and ~window_sec ago using history samples.
        More robust than instantaneous vx/vy.
        """
        if len(self.history) < 2:
            return 0.0

        now_t = self.history[-1][2]
        target_t = now_t - window_sec

        idx = 0
        for i in range(len(self.history)):
            if self.history[i][2] >= target_t:
                idx = i
                break

        x0, y0, _ = self.history[idx]
        x1, y1, _ = self.history[-1]
        return math.hypot(x1 - x0, y1 - y0)


class DynamicObstacleTracker:
    def __init__(self, node, max_obstacles: int = 50,
                 tracking_timeout: float = 5.0,
                 min_confidence: float = 0.3,
                 cluster_resolution: float = 0.15,
                 min_cluster_size: int = 3):
        self.node = node
        self.obstacles: Dict[int, Obstacle] = {}      # Dynamic obstacles (active threats)
        self.static_obstacles: Dict[int, Obstacle] = {}  # Permanent static map
        self.candidates: Dict[int, Obstacle] = {}     # Newly seen, not yet classified
        self.max_obstacles = max_obstacles
        self.tracking_timeout = tracking_timeout
        self.min_confidence = min_confidence
        self.cluster_resolution = cluster_resolution
        self.min_cluster_size = min_cluster_size
        self.obstacle_counter = 0
        self.last_update_time = 0.0
        self.logged_obstacle_ids = set()

    def cluster_scan_points(self, scan_ranges, scan_angles,
                           grid_resolution: float = None) -> List[Tuple[float, float, int]]:
        if grid_resolution is None:
            grid_resolution = self.cluster_resolution
        clusters = []
        if not scan_ranges or not scan_angles:
            return clusters
        grid = {}
        for angle, r in zip(scan_angles, scan_ranges):
            if 0.1 < r < 5.0:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                gx = int(x / grid_resolution)
                gy = int(y / grid_resolution)
                grid.setdefault((gx, gy), []).append((x, y))
        for cell_points in grid.values():
            if len(cell_points) >= self.min_cluster_size:
                cx = np.mean([p[0] for p in cell_points])
                cy = np.mean([p[1] for p in cell_points])
                clusters.append((cx, cy, len(cell_points)))
        return clusters

    def update_from_scan(self, scan_ranges, scan_angles,
                         robot_x: float = 0.0, robot_y: float = 0.0,
                         robot_yaw: float = 0.0,
                         robot_vx: float = 0.0, robot_vy: float = 0.0,
                         sensor_x: float = 0.0, sensor_y: float = 0.0,
                         sensor_yaw: float = 0.0,
                         scan_stamp: float = 0.0):
        """
        Update obstacle tracking with candidate stage and robust dynamic promotion.
        Uses ROS time (scan_stamp) for accurate dt.
        """
        clusters = self.cluster_scan_points(scan_ranges, scan_angles)
        # Use scan timestamp if provided, otherwise fallback to system time
        current_time = scan_stamp if scan_stamp > 0.0 else time.time()

        # Transform clusters from sensor frame to world frame
        cos_r = math.cos(robot_yaw)
        sin_r = math.sin(robot_yaw)
        cos_s = math.cos(sensor_yaw)
        sin_s = math.sin(sensor_yaw)

        world_clusters = []
        for cx_s, cy_s, count in clusters:
            bx = sensor_x + cos_s * cx_s - sin_s * cy_s
            by = sensor_y + sin_s * cx_s + cos_s * cy_s
            wx = robot_x + cos_r * bx - sin_r * by
            wy = robot_y + sin_r * bx + cos_r * by
            world_clusters.append((wx, wy, count))

        robot_speed = math.hypot(robot_vx, robot_vy)
        assoc_dist = 1.0 if robot_speed > 0.2 else 0.6

        def near_static(wx: float, wy: float, dist_th: float = 0.12) -> bool:
            for s in self.static_obstacles.values():
                if math.hypot(wx - s.x, wy - s.y) < dist_th:
                    return True
            return False

        def associate_and_update(container: Dict[int, Obstacle], container_name: str):
            associated = set()
            for obs_id, obstacle in list(container.items()):
                best_idx = -1
                best_dist = assoc_dist
                for i, (wx, wy, _) in enumerate(world_clusters):
                    if i in associated:
                        continue
                    dist = math.hypot(wx - obstacle.x, wy - obstacle.y)
                    if dist < best_dist:
                        best_dist = dist
                        best_idx = i

                if best_idx >= 0:
                    wx, wy, count = world_clusters[best_idx]

                    last_t = getattr(obstacle, "_last_meas_time", obstacle.last_updated if obstacle.last_updated > 0 else current_time)
                    dt = max(1e-3, current_time - last_t)

                    implied_speed = math.hypot(wx - obstacle.x, wy - obstacle.y) / dt
                    max_assoc_speed = 2.5   # increased tolerance
                    if implied_speed > max_assoc_speed and best_dist > 0.15:
                        # Reject only if large jump AND far away
                        obstacle.confidence *= 0.90
                        if obstacle.confidence < self.min_confidence:
                            self.node.get_logger().info(
                                f"🗑️ {container_name} {obs_id}: removed (low confidence {obstacle.confidence:.2f})"
                            )
                            del container[obs_id]
                        continue

                    confidence = min(1.0, obstacle.confidence + 0.1)
                    old_x, old_y = obstacle.x, obstacle.y
                    old_speed = obstacle.get_speed()

                    obstacle.update(wx, wy, confidence, meas_time=current_time)
                    new_speed = obstacle.get_speed()

                    if math.hypot(obstacle.x - old_x, obstacle.y - old_y) > 0.1 or abs(new_speed - old_speed) > 0.2:
                        self.node.get_logger().info(
                            f"📡 {container_name} {obs_id}: pos=({obstacle.x:.2f},{obstacle.y:.2f}) "
                            f"vel=({obstacle.vx:.2f},{obstacle.vy:.2f}) speed={new_speed:.2f}m/s conf={confidence:.2f}"
                        )
                    associated.add(best_idx)
                else:
                    obstacle.confidence *= 0.85
                    if obstacle.confidence < self.min_confidence:
                        self.node.get_logger().info(
                            f"🗑️ {container_name} {obs_id}: removed (low confidence {obstacle.confidence:.2f})"
                        )
                        del container[obs_id]
            return associated

        dyn_associated = associate_and_update(self.obstacles, "DynObs")
        cand_associated = associate_and_update(self.candidates, "CandObs")

        # Create new candidates
        for i, (wx, wy, count) in enumerate(world_clusters):
            if i in dyn_associated or i in cand_associated:
                continue
            if near_static(wx, wy):
                continue

            self.obstacle_counter += 1
            new_id = self.obstacle_counter
            self.candidates[new_id] = Obstacle(x=wx, y=wy, confidence=0.5)
            self.node.get_logger().debug(
                f"🆕 NEW CandObs {new_id}: at ({wx:.2f},{wy:.2f}) cluster size={count}"
            )

        # Promote candidates
        promote_to_static = []
        promote_to_dynamic = []

        min_age_sec = 1.0
        min_updates = 4
        static_disp_th = 0.05
        dynamic_disp_th = 0.15
        if robot_speed > 0.2:
            dynamic_disp_th = 0.25

        for obs_id, obs in self.candidates.items():
            age = current_time - obs.created_at
            if age < min_age_sec or obs.num_updates < min_updates:
                continue

            disp = obs.get_displacement_over(1.0)
            jitter = obs.get_position_jitter()
            speed = obs.get_speed()

            # Change in distance to robot
            delta_r = 0.0
            # Change in bearing to robot
            delta_bearing = 0.0
            if len(obs.history) >= 2:
                now_t = obs.history[-1][2]
                target_t = now_t - 1.0
                idx = 0
                for k in range(len(obs.history)):
                    if obs.history[k][2] >= target_t:
                        idx = k
                        break
                x0, y0, _ = obs.history[idx]
                r_old = math.hypot(x0 - robot_x, y0 - robot_y)
                r_now = math.hypot(obs.x - robot_x, obs.y - robot_y)
                delta_r = abs(r_now - r_old)

                bearing_old = math.atan2(y0 - robot_y, x0 - robot_x)
                bearing_now = math.atan2(obs.y - robot_y, obs.x - robot_x)
                delta_bearing = abs(math.atan2(math.sin(bearing_now - bearing_old),
                                               math.cos(bearing_now - bearing_old)))

            # STATIC
            if disp < static_disp_th or jitter < 0.02:
                self.static_obstacles[obs_id] = obs
                promote_to_static.append(obs_id)
                self.node.get_logger().debug(
                    f"📌 CandObs {obs_id}: STATIC (age={age:.1f}s updates={obs.num_updates} disp={disp:.2f} "
                    f"jitter={jitter:.3f} speed={speed:.2f} Δr={delta_r:.2f} Δθ={math.degrees(delta_bearing):.1f}°)"
                )
                continue

            # DYNAMIC: displacement + range change + speed + small bearing change
            if disp > dynamic_disp_th and delta_r > 0.08 and speed > 0.08 and delta_bearing < math.radians(15):
                self.obstacles[obs_id] = obs
                promote_to_dynamic.append(obs_id)
                self.node.get_logger().info(
                    f"✅ CandObs {obs_id}: promoted DYNAMIC (age={age:.1f}s updates={obs.num_updates} "
                    f"disp={disp:.2f}m speed={speed:.2f} Δr={delta_r:.2f} Δθ={math.degrees(delta_bearing):.1f}°)"
                )

        for obs_id in promote_to_static:
            if obs_id in self.candidates:
                del self.candidates[obs_id]
        for obs_id in promote_to_dynamic:
            if obs_id in self.candidates:
                del self.candidates[obs_id]

        # Timeout cleanup
        to_remove = []
        for obs_id, obs in self.obstacles.items():
            if current_time - obs.last_updated > self.tracking_timeout:
                to_remove.append(obs_id)
        for obs_id in to_remove:
            self.node.get_logger().info(f"⏰ DynObs {obs_id}: removed (timeout {self.tracking_timeout}s)")
            del self.obstacles[obs_id]

        to_remove = []
        for obs_id, obs in self.candidates.items():
            if current_time - obs.last_updated > self.tracking_timeout:
                to_remove.append(obs_id)
        for obs_id in to_remove:
            self.node.get_logger().debug(f"⏰ CandObs {obs_id}: removed (timeout {self.tracking_timeout}s)")
            del self.candidates[obs_id]

        self.last_update_time = current_time

    # ----- Threat assessment (only on dynamic obstacles) -----
    def get_threat_level(self, robot_x: float, robot_y: float,
                         goal_x: float, goal_y: float,
                         robot_speed: float = 0.0) -> float:
        if not self.obstacles:
            return 0.0

        max_threat = 0.0
        threat_obs_id = None
        for obs_id, obs in self.obstacles.items():
            dist_to_robot = math.hypot(obs.x - robot_x, obs.y - robot_y)
            if dist_to_robot > 3.0:
                continue
            dist_threat = max(0.0, min(1.0, 1.0 - (dist_to_robot - 0.5) / 2.5))
            obs_speed = obs.get_speed()
            speed_threat = min(1.0, obs_speed / 1.5)
            to_robot_x = robot_x - obs.x
            to_robot_y = robot_y - obs.y
            closing = to_robot_x * obs.vx + to_robot_y * obs.vy
            if closing > 0:
                closing_threat = min(1.0, closing / (dist_to_robot * max(0.5, obs_speed) + 0.1))
            else:
                closing_threat = 0.0
            to_goal_x = goal_x - obs.x
            to_goal_y = goal_y - obs.y
            goal_dist = math.hypot(to_goal_x, to_goal_y)
            if goal_dist > 0.1:
                dot_goal = to_goal_x * obs.vx + to_goal_y * obs.vy
                if dot_goal > 0:
                    goal_alignment = min(1.0, dot_goal / (goal_dist * max(0.5, obs_speed) + 0.1))
                else:
                    goal_alignment = 0.0
            else:
                goal_alignment = 0.0
            threat = 0.4 * dist_threat + 0.3 * speed_threat + 0.2 * closing_threat + 0.1 * goal_alignment
            if threat > max_threat:
                max_threat = threat
                threat_obs_id = obs_id

        if max_threat > 0.2 and threat_obs_id is not None:
            obs = self.obstacles[threat_obs_id]
            self.node.get_logger().info(
                f"⚠️ Threat level {max_threat:.2f} from DynObs {threat_obs_id} "
                f"(dist={math.hypot(obs.x - robot_x, obs.y - robot_y):.2f}m, "
                f"speed={obs.get_speed():.2f}m/s)"
            )
        return max_threat

    def get_suggested_evasion_direction(self, robot_x: float, robot_y: float,
                                        robot_yaw: float,
                                        goal_x: float, goal_y: float) -> Optional[float]:
        if not self.obstacles:
            return None

        best_threat = 0.0
        threat_obs = None
        for obs in self.obstacles.values():
            dist_to_robot = math.hypot(obs.x - robot_x, obs.y - robot_y)
            if dist_to_robot > 3.0:
                continue
            dist_threat = max(0.0, min(1.0, 1.0 - (dist_to_robot - 0.5) / 2.5))
            obs_speed = obs.get_speed()
            speed_threat = min(1.0, obs_speed / 1.5)
            to_robot_x = robot_x - obs.x
            to_robot_y = robot_y - obs.y
            closing = to_robot_x * obs.vx + to_robot_y * obs.vy
            if closing > 0:
                closing_threat = min(1.0, closing / (dist_to_robot * max(0.5, obs_speed) + 0.1))
            else:
                closing_threat = 0.0
            to_goal_x = goal_x - obs.x
            to_goal_y = goal_y - obs.y
            goal_dist = math.hypot(to_goal_x, to_goal_y)
            if goal_dist > 0.1:
                dot_goal = to_goal_x * obs.vx + to_goal_y * obs.vy
                if dot_goal > 0:
                    goal_alignment = min(1.0, dot_goal / (goal_dist * max(0.5, obs_speed) + 0.1))
                else:
                    goal_alignment = 0.0
            else:
                goal_alignment = 0.0
            threat = 0.4 * dist_threat + 0.3 * speed_threat + 0.2 * closing_threat + 0.1 * goal_alignment
            if threat > best_threat:
                best_threat = threat
                threat_obs = obs

        if threat_obs is None or best_threat < 0.2:
            return None

        obs_angle = math.atan2(threat_obs.y - robot_y, threat_obs.x - robot_x)
        goal_angle = math.atan2(goal_y - robot_y, goal_x - robot_x)
        obs_move_angle = math.atan2(threat_obs.vy, threat_obs.vx)
        to_robot_angle = math.atan2(robot_y - threat_obs.y, robot_x - threat_obs.x)
        angle_diff = abs(math.atan2(math.sin(obs_move_angle - to_robot_angle),
                                    math.cos(obs_move_angle - to_robot_angle)))
        if angle_diff < math.radians(30):
            evade_angle = obs_move_angle + math.pi
            self.node.get_logger().info(
                f"🔄 Evasion: obstacle moving toward robot → turn opposite (angle={math.degrees(evade_angle):.0f}°)")
        else:
            left_angle = goal_angle + math.pi/2
            right_angle = goal_angle - math.pi/2
            if abs(obs_angle - left_angle) < abs(obs_angle - right_angle):
                evade_angle = right_angle
                self.node.get_logger().info(f"🔄 Evasion: turn RIGHT (away from obstacle)")
            else:
                evade_angle = left_angle
                self.node.get_logger().info(f"🔄 Evasion: turn LEFT (away from obstacle)")

        return math.atan2(math.sin(evade_angle), math.cos(evade_angle))

    # ----- Existing public methods (unchanged) -----
    def get_obstacles(self) -> Dict[int, Obstacle]:
        return self.obstacles

    def get_high_confidence_obstacles(self) -> List[Obstacle]:
        return [obs for obs in self.obstacles.values() if obs.confidence > 0.7]

    def predict_collision(self, path_points: List[Tuple[float, float]],
                         robot_radius: float = 0.35,
                         prediction_horizon: float = 2.0) -> Optional[Tuple[float, int, int]]:
        for waypoint_idx, (px, py) in enumerate(path_points):
            arrival_time = waypoint_idx * 0.1
            for obs_id, obs in self.obstacles.items():
                if obs.confidence < self.min_confidence:
                    continue
                pred_x, pred_y = obs.predict_position(arrival_time)
                dist = math.hypot(px - pred_x, py - pred_y)
                if dist < robot_radius + obs.radius:
                    self.node.get_logger().warn(
                        f"💥 PREDICTED COLLISION: waypoint {waypoint_idx}, obs {obs_id}, "
                        f"dist={dist:.2f}m, arrival={arrival_time:.1f}s"
                    )
                    return (arrival_time, waypoint_idx, obs_id)
        return None

    def get_safe_distance(self, robot_speed: float = 0.5) -> float:
        base_safety = 0.6
        max_approach_speed = 0.0
        for obs in self.obstacles.values():
            speed = obs.get_speed()
            if speed > max_approach_speed:
                max_approach_speed = speed
        speed_buffer = min(1.5, max_approach_speed * 1.2)
        if max_approach_speed > 0.5:
            self.node.get_logger().debug(f"Dynamic safe distance: {base_safety + speed_buffer:.2f}m (approach speed {max_approach_speed:.2f}m/s)")
        return base_safety + speed_buffer
