#!/usr/bin/env python3
"""
Vehicle Dynamics Models for Scout UGV
"""
import numpy as np
import math

class VehicleDynamics:
    def __init__(self, robot_mass=25.0, wheel_radius=0.08, friction_coeff=0.8):
        """
        Initialize vehicle dynamics model
        
        Parameters:
        - robot_mass: Mass of robot in kg
        - wheel_radius: Wheel radius in meters
        - friction_coeff: Tire-ground friction coefficient
        """
        self.mass = robot_mass
        self.wheel_radius = wheel_radius
        self.friction_coeff = friction_coeff
        
        # Moments of inertia (approximate for Scout)
        self.I_z = 1.5  # Yaw inertia
        self.g = 9.81  # Gravity
        
        # Vehicle dimensions
        self.wheelbase = 0.35  # meters
        self.track_width = 0.40  # meters
        
    def calculate_forces(self, velocity, acceleration, steering_angle, slope_angle):
        """
        Calculate forces acting on vehicle
        
        Returns: (tractive_force, rolling_resistance, grade_resistance, total_force)
        """
        # Rolling resistance
        rolling_resistance = 0.01 * self.mass * self.g * np.cos(slope_angle)
        
        # Grade resistance (gravity component)
        grade_resistance = self.mass * self.g * np.sin(slope_angle)
        
        # Acceleration force
        acceleration_force = self.mass * acceleration
        
        # Aerodynamic drag (simplified)
        air_density = 1.225  # kg/m³
        frontal_area = 0.5  # m²
        drag_coeff = 0.8
        drag_force = 0.5 * air_density * frontal_area * drag_coeff * velocity**2
        
        # Total force required
        total_force = rolling_resistance + grade_resistance + acceleration_force + drag_force
        
        return {
            'rolling_resistance': rolling_resistance,
            'grade_resistance': grade_resistance,
            'acceleration_force': acceleration_force,
            'drag_force': drag_force,
            'total_force': total_force
        }
    
    def calculate_power(self, velocity, acceleration, slope_angle):
        """Calculate power consumption"""
        forces = self.calculate_forces(velocity, acceleration, 0, slope_angle)
        power = forces['total_force'] * velocity
        
        # Motor efficiency (typically 70-90%)
        motor_efficiency = 0.8
        electrical_power = power / motor_efficiency
        
        return electrical_power  # Watts
    
    def calculate_slip_angle(self, velocity, steering_angle):
        """
        Calculate tire slip angles
        
        Returns: (front_slip_angle, rear_slip_angle) in radians
        """
        if abs(velocity) < 0.1:
            return 0.0, 0.0
        
        # Bicycle model approximation
        L = self.wheelbase
        beta = steering_angle - (L / 2) * steering_angle / velocity
        
        front_slip = beta + (L/2) * steering_angle / velocity - steering_angle
        rear_slip = beta - (L/2) * steering_angle / velocity
        
        return front_slip, rear_slip
    
    def lateral_stability(self, velocity, curvature, friction_coeff=None):
        """
        Calculate lateral stability metric (0-1, where 1 is most stable)
        """
        if friction_coeff is None:
            friction_coeff = self.friction_coeff
        
        # Centripetal acceleration
        if abs(curvature) < 1e-6:
            lateral_acc = 0.0
        else:
            radius = 1.0 / curvature
            lateral_acc = velocity**2 / radius
        
        # Maximum lateral acceleration before slip
        max_lateral_acc = friction_coeff * self.g
        
        # Stability factor (closer to 1 is better)
        if max_lateral_acc > 0:
            stability = 1.0 - min(1.0, lateral_acc / max_lateral_acc)
        else:
            stability = 0.0
        
        return stability
