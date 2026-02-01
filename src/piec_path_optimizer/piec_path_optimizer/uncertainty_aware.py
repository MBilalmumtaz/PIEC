#!/usr/bin/env python3
import numpy as np
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry

class UncertaintyAwarePlanner:
    def __init__(self, node: Node, grid_size=100, resolution=0.1):
        self.node = node
        self.grid_size = grid_size
        self.resolution = resolution
        
        # Uncertainty map
        self.uncertainty_map = np.zeros((grid_size, grid_size))
        self.decay_factor = 0.95
        self.max_uncertainty = 5.0
        
        # Historical uncertainty for trend analysis
        self.uncertainty_history = []
        self.history_size = 50
        
    def update_uncertainty_map(self, ukf_odom: Odometry):
        """Update uncertainty heatmap from UKF covariance"""
        # Extract covariance matrix
        cov = np.array(ukf_odom.pose.covariance).reshape(6, 6)
        
        # Position uncertainty (x, y)
        position_cov = cov[:2, :2]
        pos_uncertainty = np.trace(position_cov)  # Sum of variances
        
        # Orientation uncertainty (yaw)
        yaw_uncertainty = cov[2, 2]
        
        # Total uncertainty metric
        total_uncertainty = pos_uncertainty + 0.5 * yaw_uncertainty
        
        # Get robot position
        x = ukf_odom.pose.pose.position.x
        y = ukf_odom.pose.pose.position.y
        
        # Convert to grid coordinates
        grid_x = int(x / self.resolution) + self.grid_size // 2
        grid_y = int(y / self.resolution) + self.grid_size // 2
        
        # Update uncertainty map with Gaussian distribution
        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            # Add uncertainty at current position
            self.uncertainty_map[grid_x, grid_y] += total_uncertainty
            
            # Also affect neighboring cells (uncertainty spreads)
            spread_radius = 2
            for dx in range(-spread_radius, spread_radius + 1):
                for dy in range(-spread_radius, spread_radius + 1):
                    nx, ny = grid_x + dx, grid_y + dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        distance = math.sqrt(dx**2 + dy**2)
                        if distance <= spread_radius:
                            weight = math.exp(-distance**2 / 2.0)
                            self.uncertainty_map[nx, ny] += total_uncertainty * weight * 0.3
            
            # Apply decay to entire map
            self.uncertainty_map *= self.decay_factor
            
            # Clip to maximum
            self.uncertainty_map = np.clip(self.uncertainty_map, 0, self.max_uncertainty)
    
    def get_uncertainty_cost(self, path_points):
        """Calculate uncertainty cost for a given path"""
        if len(path_points) == 0:
            return 0.0
        
        total_cost = 0.0
        for point in path_points:
            x, y = point
            
            # Convert to grid coordinates
            grid_x = int(x / self.resolution) + self.grid_size // 2
            grid_y = int(y / self.resolution) + self.grid_size // 2
            
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                total_cost += self.uncertainty_map[grid_x, grid_y]
            else:
                # Penalize paths outside map bounds
                total_cost += self.max_uncertainty
        
        return total_cost / len(path_points)
    
    def suggest_low_uncertainty_path(self, start, goal, current_path):
        """Suggest path modifications to avoid high uncertainty regions"""
        if len(current_path) < 3:
            return current_path
        
        modified_path = np.array(current_path, copy=True)
        
        # Check each waypoint for high uncertainty
        for i in range(1, len(modified_path) - 1):
            x, y = modified_path[i]
            grid_x = int(x / self.resolution) + self.grid_size // 2
            grid_y = int(y / self.resolution) + self.grid_size // 2
            
            if (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size and
                self.uncertainty_map[grid_x, grid_y] > self.max_uncertainty * 0.7):
                
                # High uncertainty at this point, try to shift it
                # Find direction with lower uncertainty
                best_dx, best_dy = 0, 0
                best_uncertainty = self.uncertainty_map[grid_x, grid_y]
                
                # Check 8 directions
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        
                        nx, ny = grid_x + dx, grid_y + dy
                        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                            uncertainty = self.uncertainty_map[nx, ny]
                            if uncertainty < best_uncertainty:
                                best_uncertainty = uncertainty
                                best_dx, best_dy = dx, dy
                
                # Shift the waypoint
                if best_dx != 0 or best_dy != 0:
                    modified_path[i, 0] += best_dx * self.resolution
                    modified_path[i, 1] += best_dy * self.resolution
        
        return modified_path
    
    def get_uncertainty_gradient(self, x, y):
        """Get uncertainty gradient at a point (for optimization)"""
        grid_x = int(x / self.resolution) + self.grid_size // 2
        grid_y = int(y / self.resolution) + self.grid_size // 2
        
        if not (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size):
            return np.array([0.0, 0.0])
        
        # Calculate gradient using finite differences
        grad_x, grad_y = 0.0, 0.0
        
        if grid_x > 0 and grid_x < self.grid_size - 1:
            grad_x = (self.uncertainty_map[grid_x + 1, grid_y] - 
                     self.uncertainty_map[grid_x - 1, grid_y]) / (2 * self.resolution)
        
        if grid_y > 0 and grid_y < self.grid_size - 1:
            grad_y = (self.uncertainty_map[grid_x, grid_y + 1] - 
                     self.uncertainty_map[grid_x, grid_y - 1]) / (2 * self.resolution)
        
        return np.array([grad_x, grad_y])
