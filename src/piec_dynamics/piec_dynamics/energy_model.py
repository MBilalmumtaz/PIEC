#!/usr/bin/env python3
"""
Physics-based Energy Consumption Model
"""
import numpy as np

class PhysicsEnergyModel:
    def __init__(self, vehicle_dynamics):
        self.dynamics = vehicle_dynamics
        
        # Battery parameters (typical Li-ion)
        self.battery_voltage = 24.0  # V
        self.battery_capacity = 20.0  # Ah
        
        # Motor parameters
        self.motor_efficiency = 0.85
        self.gear_ratio = 20.0
        self.motor_resistance = 0.1  # Ohms
        
        # Constants
        self.rho_air = 1.225  # kg/m³
        self.Cd = 0.8  # Drag coefficient
        self.A_front = 0.5  # m²
        
    def calculate_energy_consumption(self, trajectory):
        """
        Calculate energy consumption for a trajectory
        
        trajectory: List of (x, y, velocity, acceleration, slope) tuples
        
        Returns: Total energy in Joules
        """
        total_energy = 0.0
        
        for i in range(len(trajectory) - 1):
            x1, y1, v1, a1, slope1 = trajectory[i]
            x2, y2, v2, a2, slope2 = trajectory[i+1]
            
            # Average values for segment
            v_avg = (v1 + v2) / 2
            a_avg = (a1 + a2) / 2
            slope_avg = (slope1 + slope2) / 2
            
            # Distance
            dx = x2 - x1
            dy = y2 - y1
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < 1e-6:
                continue
            
            # Time for segment
            dt = distance / max(abs(v_avg), 0.1)
            
            # Calculate power
            power = self.calculate_instantaneous_power(v_avg, a_avg, slope_avg)
            
            # Energy for segment
            segment_energy = power * dt
            total_energy += segment_energy
        
        return total_energy
    
    def calculate_instantaneous_power(self, velocity, acceleration, slope_angle):
        """Calculate instantaneous power requirement"""
        # Forces from vehicle dynamics
        forces = self.dynamics.calculate_forces(velocity, acceleration, 0, slope_angle)
        total_force = forces['total_force']
        
        # Mechanical power
        mech_power = total_force * velocity
        
        # Electrical power (accounting for losses)
        if mech_power >= 0:
            # Motoring - power from battery to wheels
            elec_power = mech_power / self.motor_efficiency
        else:
            # Regenerative braking - power from wheels to battery
            elec_power = mech_power * self.motor_efficiency
        
        # Add constant power for electronics
        electronics_power = 50.0  # Watts for computer, sensors, etc.
        
        return max(elec_power, 0) + electronics_power
    
    def battery_state_of_charge(self, initial_soc, energy_consumed):
        """
        Update battery state of charge
        
        Returns: New SOC (0-1)
        """
        total_energy_wh = self.battery_capacity * self.battery_voltage
        consumed_wh = energy_consumed / 3600.0  # Convert Joules to Wh
        
        new_soc = initial_soc - (consumed_wh / total_energy_wh)
        return max(0.0, min(1.0, new_soc))
