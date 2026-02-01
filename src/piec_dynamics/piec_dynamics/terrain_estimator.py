#!/usr/bin/env python3
"""
Estimate terrain slope and roughness from IMU and wheel encoders
"""
import numpy as np
from scipy import signal

class TerrainEstimator:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.imu_buffer = []
        self.odom_buffer = []
        
    def estimate_slope(self, imu_data):
        """
        Estimate terrain slope from IMU accelerometer
        
        imu_data: Dictionary with 'accel_x', 'accel_y', 'accel_z', 'orientation'
        """
        # Extract acceleration in world frame
        accel = np.array([imu_data['accel_x'], 
                          imu_data['accel_y'], 
                          imu_data['accel_z']])
        
        # Remove gravity component to get terrain-induced acceleration
        # This is simplified - in practice you'd use orientation to rotate to world frame
        gravity = 9.81
        vertical_accel = accel[2] - gravity
        
        # Slope angle from vertical acceleration
        slope_angle = np.arcsin(vertical_accel / gravity)
        
        return float(slope_angle)
    
    def estimate_roughness(self, imu_data_history, velocity_history):
        """
        Estimate terrain roughness from IMU vibration
        
        Returns: Roughness index (0-1, where 1 is roughest)
        """
        if len(imu_data_history) < self.window_size:
            return 0.0
        
        # Extract vertical accelerations
        vertical_accels = [data['accel_z'] for data in imu_data_history[-self.window_size:]]
        
        # Calculate RMS of high-frequency components
        accel_array = np.array(vertical_accels)
        
        # Remove DC component
        accel_filtered = accel_array - np.mean(accel_array)
        
        # Calculate RMS as roughness measure
        rms = np.sqrt(np.mean(accel_filtered**2))
        
        # Normalize to 0-1 range (assuming 0-2 m/s² is typical range)
        roughness = min(1.0, rms / 2.0)
        
        return roughness
    
    def detect_terrain_type(self, imu_data, wheel_slip=None):
        """
        Classify terrain type based on vibration patterns
        
        Returns: terrain_type string
        """
        # Simple heuristic-based classification
        vertical_variance = np.var([imu_data['accel_x'], 
                                    imu_data['accel_y'], 
                                    imu_data['accel_z']])
        
        if vertical_variance < 0.1:
            return 'asphalt'
        elif vertical_variance < 0.5:
            return 'grass'
        elif vertical_variance < 1.0:
            return 'gravel'
        elif vertical_variance < 2.0:
            return 'sand'
        else:
            return 'rough_terrain'
    
    def update(self, imu_data, odom_data):
        """
        Update terrain estimates with new sensor data
        
        Returns: (slope_angle, roughness, terrain_type)
        """
        # Update buffers
        self.imu_buffer.append(imu_data)
        if len(self.imu_buffer) > self.window_size * 2:
            self.imu_buffer.pop(0)
        
        # Estimate terrain properties
        slope = self.estimate_slope(imu_data)
        roughness = self.estimate_roughness(self.imu_buffer, [])
        terrain_type = self.detect_terrain_type(imu_data)
        
        return slope, roughness, terrain_type
