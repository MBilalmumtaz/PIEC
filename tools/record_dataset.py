#!/usr/bin/env python3
"""
Enhanced PINN Data Recorder with laser scan features
Records robot state, commands, IMU, and laser scan for PINN training
"""
import rclpy
import csv
import math
import os
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan

class EnhancedPINNDataRecorder(Node):
    def __init__(self, filename='pinn_dataset.csv'):
        super().__init__('enhanced_piec_recorder')
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Initialize variables
        self.filename = filename
        os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Extended header with terrain and laser features
        self.csv_writer.writerow([
            't', 'x', 'y', 'yaw', 'v_cmd', 'omega_cmd', 'imu_wz',
            'slope', 'roughness', 'obstacle_density', 'clearance',
            'terrain_type', 'sim_energy', 'sim_stability'
        ])
        
        self.last_cmd = (0.0, 0.0)
        self.last_imu_wz = 0.0
        self.last_scan = None
        self.scan_angles = None
        self.record_counter = 0
        
        self.get_logger().info(f'Enhanced PINN Recorder started -> {filename}')
        self.get_logger().info('Recording: pose, commands, IMU, laser scan features')
    
    def scan_callback(self, msg):
        """Process laser scan for terrain/obstacle features"""
        self.last_scan = msg.ranges
        if self.scan_angles is None:
            self.scan_angles = [
                msg.angle_min + i * msg.angle_increment 
                for i in range(len(msg.ranges))
            ]
    
    def cmd_callback(self, msg):
        """Record command velocities"""
        self.last_cmd = (msg.linear.x, msg.angular.z)
    
    def imu_callback(self, msg):
        """Record IMU angular velocity"""
        self.last_imu_wz = msg.angular_velocity.z
    
    def extract_laser_features(self):
        """Extract obstacle features from laser scan"""
        if self.last_scan is None or self.scan_angles is None:
            return 0.0, 0.0, 0.0  # obstacle_density, clearance, terrain_type
        
        ranges = np.array(self.last_scan)
        
        # Filter valid ranges
        valid_mask = (ranges > 0.1) & (ranges < 10.0)
        if not np.any(valid_mask):
            return 0.0, 10.0, 0  # No obstacles detected
        
        valid_ranges = ranges[valid_mask]
        
        # Calculate obstacle density in front sector (-45 to +45 degrees)
        front_indices = []
        for i, angle in enumerate(self.scan_angles):
            if valid_mask[i] and abs(angle) < 0.785:  # ±45 degrees
                front_indices.append(i)
        
        if front_indices:
            front_ranges = ranges[front_indices]
            
            # Obstacle density: proportion of close obstacles
            close_obstacles = len([r for r in front_ranges if r < 1.5])
            obstacle_density = close_obstacles / len(front_ranges)
            
            # Average clearance
            clearance = np.mean(front_ranges)
            
            # Terrain type classification based on scan pattern
            range_std = np.std(valid_ranges)
            if range_std < 0.2:
                terrain_type = 0  # Flat/open
            elif range_std < 0.5:
                terrain_type = 1  # Moderately rough
            else:
                terrain_type = 2  # Cluttered/rough
            
            return obstacle_density, clearance, terrain_type
        
        return 0.0, 10.0, 0
    
    def calculate_energy_stability(self, v_cmd, omega_cmd, slope, roughness, obstacle_density):
        """Calculate energy consumption and stability (enhanced physics model)"""
        # Robot parameters
        mass = 50.0  # kg
        g = 9.81  # m/s²
        
        # Energy components
        kinetic_energy = 0.5 * mass * v_cmd**2
        turning_energy = 0.1 * mass * abs(omega_cmd)  # Simplified turning model
        slope_energy = mass * g * abs(slope) * 0.1  # Height change over distance
        friction_energy = 0.05 * mass * g * roughness * v_cmd
        obstacle_energy = 0.3 * mass * obstacle_density * v_cmd**2  # Avoidance energy
        
        total_energy = kinetic_energy + turning_energy + slope_energy + friction_energy + obstacle_energy
        
        # Stability metric (0-1, higher is more stable)
        base_stability = 1.0
        
        # Stability penalties
        speed_penalty = min(0.3, v_cmd * 0.2)  # Higher speed reduces stability
        turn_penalty = min(0.3, abs(omega_cmd) * 0.5)  # Sharp turns reduce stability
        roughness_penalty = min(0.4, roughness * 0.8)  # Rough terrain reduces stability
        obstacle_penalty = min(0.5, obstacle_density * 1.0)  # Obstacles reduce stability
        
        stability = base_stability - speed_penalty - turn_penalty - roughness_penalty - obstacle_penalty
        stability = max(0.05, min(1.0, stability))  # Clamp to valid range
        
        return total_energy, stability
    
    def odom_callback(self, msg):
        """Main recording callback - triggered by odometry updates"""
        # Record at controlled rate (approx 10Hz)
        self.record_counter += 1
        if self.record_counter % 5 != 0:  # Downsample from ~50Hz to ~10Hz
            return
        
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        
        # Extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y**2 + q.z**2))
        
        v_cmd, omega_cmd = self.last_cmd
        imu_wz = self.last_imu_wz
        
        # Extract laser features
        obstacle_density, clearance, terrain_type = self.extract_laser_features()
        
        # Placeholder values for slope and roughness
        # In a real system, these would come from terrain estimation
        slope = 0.0  # Get from IMU/terrain estimator
        roughness = 0.0  # Get from vibration analysis
        
        # Calculate energy and stability using enhanced model
        sim_energy, sim_stability = self.calculate_energy_stability(
            v_cmd, omega_cmd, slope, roughness, obstacle_density
        )
        
        # Record all features
        self.csv_writer.writerow([
            timestamp, x, y, yaw, v_cmd, omega_cmd, imu_wz,
            slope, roughness, obstacle_density, clearance,
            terrain_type, sim_energy, sim_stability
        ])
        
        self.csv_file.flush()
        
        # Log progress occasionally
        if self.record_counter % 100 == 0:
            self.get_logger().info(
                f"Recorded {self.record_counter//5} samples | "
                f"Pos: ({x:.2f}, {y:.2f}) | "
                f"Cmd: ({v_cmd:.2f}, {omega_cmd:.2f}) | "
                f"Obstacles: {obstacle_density:.2f}"
            )
    
    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f"Data recording complete. Saved to {self.filename}")

def main():
    rclpy.init()
    node = EnhancedPINNDataRecorder(filename='pinn_dataset.csv')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Recording interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Recording error: {str(e)}")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
