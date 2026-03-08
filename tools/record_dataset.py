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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
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
        
        # CRITICAL FIX: Match the publisher's QoS exactly
        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Changed from BEST_EFFORT
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, laser_qos
        )
        
        # Initialize variables
        self.filename = filename
        os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Extended header with terrain and laser features
        self.csv_writer.writerow([
            't', 'x', 'y', 'yaw', 'v_cmd', 'omega_cmd', 'imu_wz',
            'slope', 'roughness', 'obstacle_density', 'clearance',
            'terrain_type', 'sim_energy', 'sim_stability','path_length'
        ])
        
        self.last_cmd = (0.0, 0.0)
        self.last_imu_wz = 0.0
        self.last_scan = None
        self.scan_angles = None
        self.record_counter = 0
        
        # For path length estimation
        self.last_position = None
        self.path_length_accumulator = 0.0
        
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
        
        # DEBUG: Print min range in front sector
        if self.record_counter % 50 == 0:
            front_ranges = []
            for i, angle in enumerate(self.scan_angles):
                if abs(angle) < 0.785:  # ±45 degrees
                    if 0.1 < msg.ranges[i] < 10.0:
                        front_ranges.append(msg.ranges[i])
            if front_ranges:
                min_front = min(front_ranges)
                avg_front = sum(front_ranges)/len(front_ranges)
                self.get_logger().info(f"🔍 Front sector - min: {min_front:.2f}m, avg: {avg_front:.2f}m, count: {len(front_ranges)}")
            else:
                self.get_logger().info("🔍 Front sector - NO OBSTACLES DETECTED")
    
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
    
    def calculate_energy_stability(self, v_cmd, omega_cmd, slope, roughness, obstacle_density, path_length):
        """
        Hybrid energy model for Scout Mini with Jetson Orin and Realsense Helios.
        Combines physics-based terms with insights from the research papers.
        
        E_total = rolling + turning + obstacle + roughness + slope + kinetic + base + drag
        where:
          rolling = C_rr * mass * g * path_length
          turning = C_t * total_turn_angle
          obstacle = k_obs * obstacle_density * path_length
          roughness = k_rough * roughness * path_length
          slope = mass * g * abs(slope) * path_length * slope_factor
          kinetic = 0.5 * mass * v_cmd^2 * (path_length / 2.0)
          base    = P_base * time
          drag    = k_drag * v_avg * path_length
          time    = path_length / v_avg   (if v_avg > 0 else fallback)

        Constants are derived from:
          - mass = 26 kg (Scout Mini)
          - g = 9.81 m/s²
          - C_rr = 0.03 (indoor hard floor, Paper 2)
          - C_t  = 15 J/rad (skid‑steer turning, your tuning)
          - k_obs = 80 (obstacle negotiation, your earlier value)
          - k_rough = 40 (roughness penalty, your earlier value)
          - slope_factor = 0.5 (your earlier value)
          - P_base = 25 W (Jetson Orin 20W + Realsense Helios 5W + IMU <1W)
          - k_drag = 1.0 J·s/m² (small speed‑dependent losses, can be tuned)
        """
        # Robot & environment constants
        mass = 26.0                # kg
        g = 9.81                   # m/s²
        C_rr = 0.03                 # rolling resistance (indoor floor)
        C_t = 15.0                  # turning cost (J/rad)
        k_obs = 80.0                # obstacle negotiation factor (J/m per density)
        k_rough = 40.0              # roughness factor (J/m per roughness unit)
        slope_factor = 0.5           # slope energy multiplier
        P_base = 25.0                # base electronics power (Jetson + sensors + controller) in watts
        k_drag = 1.0                 # speed‑dependent drag coefficient (J·s/m²)

        # Average speed for time calculation
        if abs(v_cmd) > 0.1:
            v_avg = abs(v_cmd)
        else:
            v_avg = 0.5              # fallback speed (avoid division by zero)

        time = path_length / v_avg    # seconds

        # Total turning angle from angular velocity
        total_turn_angle = abs(omega_cmd) * time   # radians

        # Energy components
        rolling_energy = C_rr * mass * g * path_length
        turning_energy = C_t * total_turn_angle
        obstacle_energy = k_obs * obstacle_density * path_length
        roughness_energy = k_rough * roughness * path_length
        slope_energy = mass * g * abs(slope) * path_length * slope_factor
        kinetic_energy = 0.5 * mass * v_cmd**2 * (path_length / 2.0)   # acceleration events
        base_energy = P_base * time
        drag_energy = k_drag * v_avg * path_length

        total_energy = (rolling_energy + turning_energy + obstacle_energy +
                        roughness_energy + slope_energy + kinetic_energy +
                        base_energy + drag_energy)

        # Clamp to realistic bounds (can be adjusted)
        total_energy = max(30.0, min(2000.0, total_energy))

        # ---------- Stability calculation (unchanged) ----------
        base_stability = 0.95
        obstacle_penalty = min(0.5, obstacle_density * 0.7)
        roughness_penalty = min(0.4, roughness * 0.5)
        turn_penalty = min(0.4, abs(omega_cmd) * 0.6)
        speed_penalty = min(0.3, abs(v_cmd) * 0.15)
        slope_penalty = min(0.3, abs(slope) * 0.8)

        stability = (base_stability - obstacle_penalty - roughness_penalty -
                     turn_penalty - speed_penalty - slope_penalty)
        stability = max(0.1, min(1.0, stability))

        return total_energy, stability
    
    def odom_callback(self, msg):
        """Main recording callback - triggered by odometry updates"""
        self.record_counter += 1
        if self.record_counter % 5 != 0:
            return
        
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y**2 + q.z**2))
        
        current_pos = (x, y)
        if self.last_position is not None:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            segment_length = math.hypot(dx, dy)
            self.path_length_accumulator += segment_length
        self.last_position = current_pos
        
        v_cmd, omega_cmd = self.last_cmd
        imu_wz = self.last_imu_wz
        
        obstacle_density, clearance, terrain_type = self.extract_laser_features()
        
        slope = 0.0
        roughness = 0.0
        
        path_length = max(1.0, self.path_length_accumulator)
        
        sim_energy, sim_stability = self.calculate_energy_stability(
            v_cmd, omega_cmd, slope, roughness, obstacle_density, path_length
        )
        
        self.csv_writer.writerow([
            timestamp, x, y, yaw, v_cmd, omega_cmd, imu_wz,
            slope, roughness, obstacle_density, clearance,
            terrain_type, sim_energy, sim_stability, path_length
        ])
        
        self.csv_file.flush()
        
        if self.record_counter % 100 == 0:
            self.get_logger().info(
                f"Recorded {self.record_counter//5} samples | "
                f"Pos: ({x:.2f}, {y:.2f}) | "
                f"Cmd: ({v_cmd:.2f}, {omega_cmd:.2f}) | "
                f"Obstacles: {obstacle_density:.2f} | "
                f"Energy: {sim_energy:.1f}J | "
                f"Path length: {path_length:.1f}m"
            )
    
    def cleanup(self):
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
