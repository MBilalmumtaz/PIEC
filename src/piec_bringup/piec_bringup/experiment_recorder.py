#!/usr/bin/env python3
"""
ROS 2 node for automated experimental data recording.

Records robot state, localization, and mission metrics during PIEC experiments
in a format compatible with the piec_analysis package.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool
import pandas as pd
import numpy as np
import os
from datetime import datetime
import threading
import math


class ExperimentRecorder(Node):
    """Records experimental data for PIEC navigation experiments."""
    
    def __init__(self):
        super().__init__('experiment_recorder')
        
        # Declare parameters
        self.declare_parameter('method', 'PIEC')
        self.declare_parameter('environment', 'corridor')
        self.declare_parameter('trial_name', 'trial_001')
        self.declare_parameter('output_dir', os.path.expanduser('~/piec_data'))
        self.declare_parameter('record_rate', 20.0)
        
        # Get parameters
        self.method = self.get_parameter('method').value
        self.environment = self.get_parameter('environment').value
        self.trial_name = self.get_parameter('trial_name').value
        self.output_dir = self.get_parameter('output_dir').value
        self.record_rate = self.get_parameter('record_rate').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize data storage
        self.trajectory_data = []
        self.localization_data = []
        self.lock = threading.Lock()
        
        # State variables
        self.current_odom = None
        self.current_scan = None
        self.current_ukf = None
        self.current_ground_truth = None
        self.current_energy = 0.0
        self.start_time = None
        self.mission_active = False
        self.path_length = 0.0
        self.last_position = None
        self.num_replans = 0
        self.min_clearance = float('inf')
        self.clearance_values = []
        
        # Create subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.ukf_sub = self.create_subscription(
            Odometry, '/ukf/odom', self.ukf_callback, 10)
        
        self.ground_truth_sub = self.create_subscription(
            Odometry, '/ground_truth/pose', self.ground_truth_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/piec/path', self.path_callback, 10)
        
        self.goal_status_sub = self.create_subscription(
            Bool, '/piec/goal_reached', self.goal_status_callback, 10)
        
        # Timer for periodic recording
        self.timer = self.create_timer(1.0/self.record_rate, self.record_callback)
        
        self.get_logger().info(f'Experiment Recorder initialized:')
        self.get_logger().info(f'  Method: {self.method}')
        self.get_logger().info(f'  Environment: {self.environment}')
        self.get_logger().info(f'  Trial: {self.trial_name}')
        self.get_logger().info(f'  Output: {self.output_dir}')
    
    def odom_callback(self, msg):
        """Update odometry state."""
        self.current_odom = msg
        
        # Start mission timing
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.mission_active = True
        
        # Calculate path length
        if self.last_position is not None:
            dx = msg.pose.pose.position.x - self.last_position[0]
            dy = msg.pose.pose.position.y - self.last_position[1]
            self.path_length += math.sqrt(dx**2 + dy**2)
        
        self.last_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # Simple energy estimation (can be replaced with actual power monitor)
        if hasattr(msg.twist.twist, 'linear'):
            v = msg.twist.twist.linear.x
            omega = msg.twist.twist.angular.z
            # Simple energy model: E = k1*v^2 + k2*omega^2
            dt = 1.0 / self.record_rate
            self.current_energy += (0.5 * v**2 + 0.3 * omega**2) * dt
    
    def scan_callback(self, msg):
        """Update laser scan data."""
        self.current_scan = msg
        
        # Calculate minimum clearance
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_range = min(valid_ranges)
            self.min_clearance = min(self.min_clearance, min_range)
            self.clearance_values.append(min_range)
    
    def ukf_callback(self, msg):
        """Update UKF localization estimate."""
        self.current_ukf = msg
    
    def ground_truth_callback(self, msg):
        """Update ground truth pose."""
        self.current_ground_truth = msg
    
    def path_callback(self, msg):
        """Track path replanning events."""
        if len(msg.poses) > 0:
            self.num_replans += 1
    
    def goal_status_callback(self, msg):
        """Handle mission completion."""
        if msg.data and self.mission_active:
            self.get_logger().info('Goal reached - saving trial data')
            self.mission_active = False
            self.save_trial_summary()
    
    def record_callback(self):
        """Periodic data recording callback."""
        if not self.mission_active or self.current_odom is None:
            return
        
        with self.lock:
            # Record trajectory data
            self.record_trajectory_point()
            
            # Record localization data if available
            if self.current_ukf is not None and self.current_ground_truth is not None:
                self.record_localization_point()
    
    def record_trajectory_point(self):
        """Record current robot state to trajectory timeseries."""
        timestamp = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Extract robot state
        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y
        
        # Extract orientation (convert quaternion to yaw)
        q = self.current_odom.pose.pose.orientation
        theta = math.atan2(2.0*(q.w*q.z + q.x*q.y), 
                          1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        v = self.current_odom.twist.twist.linear.x
        omega = self.current_odom.twist.twist.angular.z
        
        # Get clearance from current scan
        clearance = 0.0
        obstacle_density = 0.0
        if self.current_scan is not None:
            valid_ranges = [r for r in self.current_scan.ranges 
                          if self.current_scan.range_min < r < self.current_scan.range_max]
            if valid_ranges:
                clearance = min(valid_ranges)
                # Obstacle density: fraction of short-range returns
                close_obstacles = sum(1 for r in valid_ranges if r < 2.0)
                obstacle_density = close_obstacles / len(valid_ranges)
        
        # Create trajectory record
        record = {
            'trial_id': self.trial_name,
            'timestamp': timestamp,
            'robot_x': x,
            'robot_y': y,
            'robot_theta': theta,
            'robot_v': v,
            'robot_omega': omega,
            'energy_cumulative': self.current_energy,
            'clearance': clearance,
            'obstacle_density': obstacle_density,
            'method': self.method,
            'environment': self.environment
        }
        
        self.trajectory_data.append(record)
    
    def record_localization_point(self):
        """Record localization estimate vs ground truth."""
        timestamp = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Extract UKF estimate
        ukf_x = self.current_ukf.pose.pose.position.x
        ukf_y = self.current_ukf.pose.pose.position.y
        ukf_q = self.current_ukf.pose.pose.orientation
        ukf_theta = math.atan2(2.0*(ukf_q.w*ukf_q.z + ukf_q.x*ukf_q.y),
                               1.0 - 2.0*(ukf_q.y*ukf_q.y + ukf_q.z*ukf_q.z))
        
        # Extract ground truth
        gt_x = self.current_ground_truth.pose.pose.position.x
        gt_y = self.current_ground_truth.pose.pose.position.y
        gt_q = self.current_ground_truth.pose.pose.orientation
        gt_theta = math.atan2(2.0*(gt_q.w*gt_q.z + gt_q.x*gt_q.y),
                             1.0 - 2.0*(gt_q.y*gt_q.y + gt_q.z*gt_q.z))
        
        # Extract covariance
        cov = self.current_ukf.pose.covariance
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_tt = cov[35]
        
        # Create localization record
        record = {
            'trial_id': self.trial_name,
            'timestamp': timestamp,
            'method': 'UKF',
            'estimated_x': ukf_x,
            'estimated_y': ukf_y,
            'estimated_theta': ukf_theta,
            'ground_truth_x': gt_x,
            'ground_truth_y': gt_y,
            'ground_truth_theta': gt_theta,
            'covariance_xx': cov_xx,
            'covariance_yy': cov_yy,
            'covariance_tt': cov_tt
        }
        
        self.localization_data.append(record)
    
    def save_trial_summary(self):
        """Save trial summary to navigation_trials.csv."""
        if self.start_time is None:
            return
        
        mission_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Calculate average clearance
        avg_clearance = np.mean(self.clearance_values) if self.clearance_values else 0.0
        
        # Create trial summary
        summary = {
            'trial_id': self.trial_name,
            'method': self.method,
            'environment': self.environment,
            'energy_consumed': self.current_energy,
            'path_length': self.path_length,
            'mission_time': mission_time,
            'success': True,
            'failure_mode': '',
            'clearance_min': self.min_clearance if self.min_clearance != float('inf') else 0.0,
            'clearance_avg': avg_clearance,
            'num_replans': self.num_replans,
            'num_near_misses': 0,  # TODO: Implement near-miss detection
            'timestamp': datetime.now().isoformat()
        }
        
        # Append to navigation_trials.csv
        trials_file = os.path.join(self.output_dir, 'navigation_trials.csv')
        df = pd.DataFrame([summary])
        
        if os.path.exists(trials_file):
            df.to_csv(trials_file, mode='a', header=False, index=False)
        else:
            df.to_csv(trials_file, mode='w', header=True, index=False)
        
        self.get_logger().info(f'Saved trial summary: {trials_file}')
        
        # Save trajectory timeseries
        self.save_trajectory_data()
        
        # Save localization timeseries
        if self.localization_data:
            self.save_localization_data()
    
    def save_trajectory_data(self):
        """Save trajectory timeseries to CSV."""
        if not self.trajectory_data:
            return
        
        trajectory_file = os.path.join(self.output_dir, 
                                      f'{self.trial_name}_trajectory.csv')
        df = pd.DataFrame(self.trajectory_data)
        df.to_csv(trajectory_file, index=False)
        
        self.get_logger().info(f'Saved trajectory data: {trajectory_file}')
    
    def save_localization_data(self):
        """Save localization timeseries to CSV."""
        if not self.localization_data:
            return
        
        localization_file = os.path.join(self.output_dir,
                                        f'{self.trial_name}_localization.csv')
        df = pd.DataFrame(self.localization_data)
        df.to_csv(localization_file, index=False)
        
        self.get_logger().info(f'Saved localization data: {localization_file}')
    
    def shutdown(self):
        """Clean shutdown - save any remaining data."""
        if self.mission_active:
            self.get_logger().info('Shutdown - saving incomplete trial data')
            self.save_trial_summary()


def main(args=None):
    rclpy.init(args=args)
    
    recorder = ExperimentRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.shutdown()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
