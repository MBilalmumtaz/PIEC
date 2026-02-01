#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import json
import csv
import os
from datetime import datetime
from collections import deque
from threading import Lock

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import BatteryState

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Parameters
        self.declare_parameter('output_dir', '~/ukf_piec_metrics')
        self.declare_parameter('sampling_rate', 1.0)  # Hz
        
        output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.sampling_rate = self.get_parameter('sampling_rate').value
        
        # Create output directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(output_dir, f'run_{timestamp}')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Metrics storage
        self.metrics = {
            'timestamps': [],
            'position': {'x': [], 'y': [], 'z': []},
            'velocity': {'linear': [], 'angular': []},
            'energy': {'consumption': [], 'efficiency': []},
            'localization': {'error': [], 'covariance': []},
            'path': {'length': [], 'smoothness': []},
            'performance': {'replanning_time': [], 'computation_time': []}
        }
        
        # Performance tracking
        self.replanning_times = deque(maxlen=100)
        self.computation_times = deque(maxlen=100)
        self.last_goal_time = None
        self.path_history = []
        self.ground_truth_history = []
        
        # Lock for thread safety
        self.lock = Lock()
        
        # Subscribers
        self.create_subscription(Odometry, '/ukf/odom', self.ukf_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Path, '/piec/path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        
        # Timer for periodic collection
        self.create_timer(1.0 / self.sampling_rate, self.collect_metrics)
        
        self.get_logger().info(f"Metrics collector started. Output: {self.output_dir}")
    
    def ukf_callback(self, msg):
        with self.lock:
            # Extract covariance
            cov = np.array(msg.pose.covariance).reshape(6, 6)
            pos_cov = np.trace(cov[:2, :2])
            
            # Store for RMSE calculation
            self.current_ukf_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'timestamp': self.get_clock().now().nanoseconds
            }
            
            self.metrics['localization']['covariance'].append(pos_cov)
    
    def gt_callback(self, msg):
        with self.lock:
            # Store ground truth for RMSE
            self.current_gt_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'timestamp': self.get_clock().now().nanoseconds
            }
            
            if hasattr(self, 'current_ukf_pose'):
                # Calculate instantaneous error
                error = np.sqrt(
                    (self.current_ukf_pose['x'] - self.current_gt_pose['x'])**2 +
                    (self.current_ukf_pose['y'] - self.current_gt_pose['y'])**2
                )
                self.metrics['localization']['error'].append(error)
    
    def cmd_callback(self, msg):
        with self.lock:
            self.current_cmd = {
                'linear': msg.linear.x,
                'angular': msg.angular.z
            }
    
    def path_callback(self, msg):
        with self.lock:
            # Calculate path metrics
            path_length = 0.0
            path_smoothness = 0.0
            
            if len(msg.poses) >= 2:
                # Calculate total path length
                for i in range(len(msg.poses) - 1):
                    p1 = msg.poses[i].pose.position
                    p2 = msg.poses[i+1].pose.position
                    path_length += np.sqrt(
                        (p2.x - p1.x)**2 + (p2.y - p1.y)**2
                    )
                
                # Calculate smoothness (average curvature)
                angles = []
                for i in range(1, len(msg.poses) - 1):
                    p0 = msg.poses[i-1].pose.position
                    p1 = msg.poses[i].pose.position
                    p2 = msg.poses[i+1].pose.position
                    
                    v1 = np.array([p1.x - p0.x, p1.y - p0.y])
                    v2 = np.array([p2.x - p1.x, p2.y - p1.y])
                    
                    if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                        cos_angle = np.clip(cos_angle, -1.0, 1.0)
                        angle = np.arccos(cos_angle)
                        angles.append(angle)
                
                if angles:
                    path_smoothness = np.mean(angles)
            
            self.path_history.append({
                'length': path_length,
                'smoothness': path_smoothness,
                'timestamp': self.get_clock().now().nanoseconds
            })
    
    def goal_callback(self, msg):
        with self.lock:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            
            if self.last_goal_time is not None:
                replanning_time = current_time - self.last_goal_time
                self.replanning_times.append(replanning_time)
                self.metrics['performance']['replanning_time'].append(replanning_time)
            
            self.last_goal_time = current_time
    
    def battery_callback(self, msg):
        with self.lock:
            # Estimate energy consumption from battery
            if hasattr(self, 'last_battery'):
                energy_used = self.last_battery - msg.percentage
                if energy_used > 0:
                    self.metrics['energy']['consumption'].append(energy_used)
            
            self.last_battery = msg.percentage
    
    def collect_metrics(self):
        """Collect and store metrics periodically"""
        with self.lock:
            timestamp = self.get_clock().now().nanoseconds * 1e-9
            self.metrics['timestamps'].append(timestamp)
            
            # Collect current state
            if hasattr(self, 'current_ukf_pose'):
                self.metrics['position']['x'].append(self.current_ukf_pose['x'])
                self.metrics['position']['y'].append(self.current_ukf_pose['y'])
            
            if hasattr(self, 'current_cmd'):
                self.metrics['velocity']['linear'].append(self.current_cmd['linear'])
                self.metrics['velocity']['angular'].append(self.current_cmd['angular'])
    
    def calculate_statistics(self):
        """Calculate final statistics"""
        stats = {}
        
        # Localization RMSE
        if self.metrics['localization']['error']:
            errors = np.array(self.metrics['localization']['error'])
            stats['localization_rmse'] = np.sqrt(np.mean(errors**2))
            stats['localization_std'] = np.std(errors)
            stats['localization_max'] = np.max(errors)
        
        # Energy efficiency
        if (self.metrics['energy']['consumption'] and 
            len(self.path_history) > 0):
            
            total_energy = np.sum(self.metrics['energy']['consumption'])
            total_distance = np.sum([p['length'] for p in self.path_history])
            
            if total_distance > 0:
                stats['energy_per_meter'] = total_energy / total_distance
                stats['total_energy'] = total_energy
                stats['total_distance'] = total_distance
        
        # Path optimization
        if self.path_history:
            path_lengths = [p['length'] for p in self.path_history]
            stats['avg_path_length'] = np.mean(path_lengths)
            stats['path_length_std'] = np.std(path_lengths)
            
            path_smoothness = [p['smoothness'] for p in self.path_history]
            stats['avg_smoothness'] = np.mean(path_smoothness)
        
        # Performance
        if self.metrics['performance']['replanning_time']:
            replan_times = np.array(self.metrics['performance']['replanning_time'])
            stats['avg_replanning_time'] = np.mean(replan_times)
            stats['replanning_std'] = np.std(replan_times)
        
        # Compare with targets from proposal
        stats['targets'] = {
            'energy_savings_target': 0.20,  # 20%
            'replanning_faster_target': 0.35,  # 35%
            'localization_rmse_target': 0.2  # meters
        }
        
        return stats
    
    def save_metrics(self):
        """Save all metrics to files"""
        with self.lock:
            # Save raw metrics
            metrics_file = os.path.join(self.output_dir, 'raw_metrics.json')
            with open(metrics_file, 'w') as f:
                json.dump(self.metrics, f, indent=2, default=lambda x: float(x))
            
            # Save path history
            path_file = os.path.join(self.output_dir, 'path_history.json')
            with open(path_file, 'w') as f:
                json.dump(self.path_history, f, indent=2, default=lambda x: float(x))
            
            # Calculate and save statistics
            stats = self.calculate_statistics()
            stats_file = os.path.join(self.output_dir, 'statistics.json')
            with open(stats_file, 'w') as f:
                json.dump(stats, f, indent=2)
            
            # Save CSV for easy analysis
            csv_file = os.path.join(self.output_dir, 'metrics.csv')
            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                
                # Write header
                writer.writerow([
                    'timestamp', 'pos_x', 'pos_y', 'vel_linear', 'vel_angular',
                    'loc_error', 'loc_covariance', 'energy_consumption'
                ])
                
                # Write data
                n_samples = len(self.metrics['timestamps'])
                for i in range(n_samples):
                    row = [
                        self.metrics['timestamps'][i] if i < n_samples else '',
                        self.metrics['position']['x'][i] if i < len(self.metrics['position']['x']) else '',
                        self.metrics['position']['y'][i] if i < len(self.metrics['position']['y']) else '',
                        self.metrics['velocity']['linear'][i] if i < len(self.metrics['velocity']['linear']) else '',
                        self.metrics['velocity']['angular'][i] if i < len(self.metrics['velocity']['angular']) else '',
                        self.metrics['localization']['error'][i] if i < len(self.metrics['localization']['error']) else '',
                        self.metrics['localization']['covariance'][i] if i < len(self.metrics['localization']['covariance']) else '',
                        self.metrics['energy']['consumption'][i] if i < len(self.metrics['energy']['consumption']) else ''
                    ]
                    writer.writerow(row)
            
            # Generate summary report
            report_file = os.path.join(self.output_dir, 'validation_report.txt')
            with open(report_file, 'w') as f:
                f.write("=" * 60 + "\n")
                f.write("UKF-PIEC Framework Validation Report\n")
                f.write("=" * 60 + "\n\n")
                
                f.write("1. LOCALIZATION PERFORMANCE\n")
                f.write("-" * 40 + "\n")
                if 'localization_rmse' in stats:
                    f.write(f"RMSE: {stats['localization_rmse']:.4f} m\n")
                    f.write(f"Target: {stats['targets']['localization_rmse_target']} m\n")
                    
                    if stats['localization_rmse'] <= stats['targets']['localization_rmse_target']:
                        f.write("✅ PASS: RMSE meets target\n")
                    else:
                        f.write("❌ FAIL: RMSE exceeds target\n")
                
                f.write("\n2. ENERGY EFFICIENCY\n")
                f.write("-" * 40 + "\n")
                if 'energy_per_meter' in stats:
                    f.write(f"Energy per meter: {stats['energy_per_meter']:.6f}\n")
                    f.write(f"Target savings: {stats['targets']['energy_savings_target']*100:.1f}%\n")
                
                f.write("\n3. PATH OPTIMIZATION\n")
                f.write("-" * 40 + "\n")
                if 'avg_path_length' in stats:
                    f.write(f"Average path length: {stats['avg_path_length']:.2f} m\n")
                    f.write(f"Path smoothness: {stats['avg_smoothness']:.4f} rad\n")
                
                f.write("\n4. PERFORMANCE\n")
                f.write("-" * 40 + "\n")
                if 'avg_replanning_time' in stats:
                    f.write(f"Average replanning time: {stats['avg_replanning_time']:.4f} s\n")
                    f.write(f"Target improvement: {stats['targets']['replanning_faster_target']*100:.1f}%\n")
            
            self.get_logger().info(f"✅ Metrics saved to: {self.output_dir}")
            return stats

def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down metrics collector...")
        
        # Save metrics before shutdown
        stats = node.save_metrics()
        
        # Print summary to console
        if stats:
            print("\n" + "=" * 60)
            print("VALIDATION SUMMARY")
            print("=" * 60)
            
            if 'localization_rmse' in stats:
                print(f"Localization RMSE: {stats['localization_rmse']:.4f} m")
                print(f"  Target: {stats['targets']['localization_rmse_target']} m")
                if stats['localization_rmse'] <= stats['targets']['localization_rmse_target']:
                    print("  ✅ PASS")
                else:
                    print("  ❌ FAIL")
            
            print(f"\nMetrics saved to: {node.output_dir}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
