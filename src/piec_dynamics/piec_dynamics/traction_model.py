#!/usr/bin/env python3
"""
Traction and Stability Model
"""
import numpy as np

class TractionModel:
    def __init__(self, mass=50.0):
        """
        Initialize traction model
        
        Args:
            mass: Robot mass in kg (default: 50.0 for Scout Mini)
        """
        self.mass = mass  # Robot mass in kg
        
        # Terrain coefficients (friction multipliers)
        self.terrain_coeffs = {
            'asphalt': 1.0,
            'grass': 0.7,
            'gravel': 0.6,
            'sand': 0.4,
            'mud': 0.3,
            'snow': 0.2
        }
        
        # Load transfer parameters
        self.cg_height = 0.3  # Center of gravity height (m)
        self.wheelbase = 0.5  # m
        self.track_width = 0.4  # Track width in m (Scout Mini wheelbase)
        
    def calculate_traction_limit(self, velocity, slope_angle, terrain_type='asphalt'):
        """
        Calculate maximum traction force before slip
        
        Returns: (longitudinal_traction, lateral_traction) in N
        """
        # Normal force on wheels
        normal_force = 9.81  # Simplified for now
        
        # Terrain friction coefficient
        base_friction = self.terrain_coeffs.get(terrain_type, 0.5)
        
        # Velocity-dependent friction (decreases with speed)
        velocity_factor = 1.0 / (1.0 + 0.1 * abs(velocity))
        
        # Slope effect (traction decreases on slopes)
        slope_factor = np.cos(slope_angle)
        
        # Effective friction coefficient
        mu_effective = base_friction * velocity_factor * slope_factor
        
        # Traction limits
        longitudinal_traction = mu_effective * normal_force
        lateral_traction = 0.8 * mu_effective * normal_force  # Lateral is typically lower
        
        return longitudinal_traction, lateral_traction
    
    def calculate_load_transfer(self, acceleration, deceleration, lateral_acc):
        """
        Calculate wheel load transfer during acceleration/braking/turning
        """
        # Longitudinal load transfer
        long_transfer = (self.mass * self.cg_height * acceleration) / self.wheelbase
        
        # Lateral load transfer
        lat_transfer = (self.mass * self.cg_height * lateral_acc) / self.track_width
        
        return long_transfer, lat_transfer
    
    def stability_metric(self, velocity, curvature, slope, terrain_type='asphalt'):
        """
        Calculate comprehensive stability metric (0-1)
        
        0 = unstable (tip/slip imminent)
        1 = completely stable
        """
        stability = 1.0
        
        # 1. Lateral stability (cornering)
        lateral_acc = velocity**2 * abs(curvature) if abs(curvature) > 1e-6 else 0
        _, lateral_traction = self.calculate_traction_limit(velocity, slope, terrain_type)
        max_lateral_acc = lateral_traction / self.mass
        
        if max_lateral_acc > 0:
            lateral_stability = 1.0 - min(1.0, lateral_acc / max_lateral_acc)
            stability *= lateral_stability
        
        # 2. Longitudinal stability (braking/acceleration)
        if slope > 0:  # Uphill
            grade_factor = 1.0 - 0.5 * slope
            stability *= grade_factor
        
        # 3. Rollover stability
        track_width = self.track_width
        rollover_threshold = (track_width / 2) / self.cg_height
        
        if lateral_acc > 0:
            roll_stability = 1.0 - min(1.0, lateral_acc / (rollover_threshold * 9.81))
            stability *= roll_stability
        
        return max(0.0, min(1.0, stability))
