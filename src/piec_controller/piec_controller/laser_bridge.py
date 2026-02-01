#!/usr/bin/env python3
"""
Enhanced Laser Bridge with Data Processing
Filters and enhances laser scan data before passing to controller
"""
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class LaserBridge(Node):
    def __init__(self):
        super().__init__('enhanced_laser_bridge')
        
        # Parameters
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('range_filter_window', 3)
        self.declare_parameter('median_filter', True)
        self.declare_parameter('outlier_threshold', 0.5)
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('input_topic', '/laser_scan')
        self.declare_parameter('output_fixed_topic', '/scan_fixed')
        self.declare_parameter('output_scan_topic', '/scan')
        
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.filter_window = self.get_parameter('range_filter_window').value
        self.use_median = self.get_parameter('median_filter').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_fixed_topic = self.get_parameter('output_fixed_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        
        # QoS profiles - Optimized for sensor data
        
        # For SUBSCRIBING to raw laser data (simulation/scanner typically uses BEST_EFFORT)
        # Try multiple QoS profiles to match publisher
        subscribe_qos_profiles = [
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,  # Most laser scanners
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            ),
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,     # Some simulators
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE
            )
        ]
        
        # For PUBLISHING processed data (use RELIABLE for consistent delivery)
        publish_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,        # Reliable delivery for planners
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriber for raw laser data - try different QoS profiles
        self.scan_subscription = None
        self.subscription_qos = subscribe_qos_profiles[0]  # Default to BEST_EFFORT
        
        # Try to create subscription with adaptive QoS
        try:
            self.scan_subscription = self.create_subscription(
                LaserScan,
                self.input_topic,
                self.scan_callback,
                self.subscription_qos
            )
            self.get_logger().info(f"Subscribed to {self.input_topic} with QoS: BEST_EFFORT")
        except Exception as e:
            self.get_logger().warn(f"Failed with BEST_EFFORT: {e}. Trying RELIABLE...")
            try:
                self.subscription_qos = subscribe_qos_profiles[1]
                self.scan_subscription = self.create_subscription(
                    LaserScan,
                    self.input_topic,
                    self.scan_callback,
                    self.subscription_qos
                )
                self.get_logger().info(f"Subscribed to {self.input_topic} with QoS: RELIABLE")
            except Exception as e2:
                self.get_logger().error(f"Failed to subscribe: {e2}")
        
        # Publishers for processed data (always use RELIABLE for consistency)
        self.publisher_fixed = self.create_publisher(
            LaserScan,
            self.output_fixed_topic,
            publish_qos
        )
        
        self.publisher_scan = self.create_publisher(
            LaserScan,
            self.output_scan_topic,
            publish_qos
        )
        
        # Optional: Publisher for filtered scan (debug)
        if self.get_parameter('publish_raw').value:
            self.publisher_raw = self.create_publisher(
                LaserScan,
                '/scan_raw',
                publish_qos
            )
        
        # Scan history for filtering
        self.scan_history = []
        self.max_history = 5
        
        # Statistics
        self.processed_count = 0
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info("Enhanced Laser Bridge started")
        self.get_logger().info(f"Input: {self.input_topic}, Outputs: {self.output_fixed_topic}, {self.output_scan_topic}")
        self.get_logger().info(f"Filter: window={self.filter_window}, median={self.use_median}")
        self.get_logger().info(f"Subscriber QoS: {self.subscription_qos.reliability}")
        self.get_logger().info(f"Publisher QoS: RELIABLE")
    
    def apply_median_filter(self, ranges, window_size):
        """Apply median filter to reduce noise"""
        if window_size < 2 or len(ranges) < window_size:
            return ranges
            
        filtered = []
        half_window = window_size // 2
        
        for i in range(len(ranges)):
            # Get window indices
            start = max(0, i - half_window)
            end = min(len(ranges), i + half_window + 1)
            
            # Get values in window
            window_values = []
            for j in range(start, end):
                if self.min_range < ranges[j] < self.max_range:
                    window_values.append(ranges[j])
            
            # Use median if we have enough values, otherwise use original
            if len(window_values) >= max(3, window_size // 2):
                filtered.append(np.median(window_values))
            else:
                filtered.append(ranges[i])
        
        return filtered
    
    def remove_outliers(self, ranges, angles, threshold):
        """Remove outlier readings"""
        if len(ranges) < 3:
            return ranges
            
        filtered = list(ranges)
        
        for i in range(1, len(ranges) - 1):
            if self.min_range < ranges[i] < self.max_range:
                # Compare with neighbors
                prev_valid = self.min_range < ranges[i-1] < self.max_range
                next_valid = self.min_range < ranges[i+1] < self.max_range
                
                if prev_valid and next_valid:
                    # Check if this reading is an outlier
                    neighbor_avg = (ranges[i-1] + ranges[i+1]) / 2
                    if abs(ranges[i] - neighbor_avg) > threshold:
                        # Replace with average of neighbors
                        filtered[i] = neighbor_avg
        
        return filtered
    
    def apply_temporal_filter(self, new_ranges):
        """Apply temporal filter using history"""
        if len(new_ranges) == 0:
            return new_ranges
            
        if not self.scan_history:
            self.scan_history.append(new_ranges)
            return new_ranges
        
        # Keep history
        self.scan_history.append(new_ranges)
        if len(self.scan_history) > self.max_history:
            self.scan_history.pop(0)
        
        # Simple moving average
        filtered = []
        for i in range(len(new_ranges)):
            valid_values = []
            for history_ranges in self.scan_history:
                if i < len(history_ranges) and self.min_range < history_ranges[i] < self.max_range:
                    valid_values.append(history_ranges[i])
            
            if valid_values:
                filtered.append(np.mean(valid_values))
            else:
                filtered.append(new_ranges[i])
        
        return filtered
    
    def scan_callback(self, msg: LaserScan):
        """Process and republish laser scan data"""
        self.processed_count += 1
        
        # Create a copy of the message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'laser'
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment if msg.time_increment > 0 else 0.0
        filtered_msg.scan_time = msg.scan_time if msg.scan_time > 0 else 0.1
        filtered_msg.range_min = max(self.min_range, msg.range_min)
        filtered_msg.range_max = min(self.max_range, msg.range_max)
        
        # Process ranges
        ranges = list(msg.ranges)
        
        # Step 1: Clip to valid range and handle inf/nan
        for i in range(len(ranges)):
            r = ranges[i]
            if (np.isnan(r) or np.isinf(r) or 
                r < filtered_msg.range_min or r > filtered_msg.range_max):
                ranges[i] = float('inf')
        
        # Step 2: Apply median filter if enabled
        if self.use_median and self.filter_window > 1 and len(ranges) > self.filter_window:
            ranges = self.apply_median_filter(ranges, self.filter_window)
        
        # Step 3: Remove outliers
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(ranges))]
        ranges = self.remove_outliers(ranges, angles, self.outlier_threshold)
        
        # Step 4: Apply temporal filter
        ranges = self.apply_temporal_filter(ranges)
        
        # Final safety check
        for i in range(len(ranges)):
            if np.isnan(ranges[i]) or np.isinf(ranges[i]):
                ranges[i] = filtered_msg.range_max
        
        filtered_msg.ranges = ranges
        filtered_msg.intensities = list(msg.intensities) if msg.intensities else []
        
        # Publish to multiple topics
        try:
            self.publisher_fixed.publish(filtered_msg)
            self.publisher_scan.publish(filtered_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish: {e}")
            return
        
        # Optional: Publish raw data for comparison
        if hasattr(self, 'publisher_raw') and self.publisher_raw:
            try:
                self.publisher_raw.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish raw: {e}")
        
        # Log processing stats occasionally (every 2 seconds)
        current_time = self.get_clock().now()
        if (current_time.nanoseconds - self.last_log_time.nanoseconds) > 2e9:
            valid_ranges = [r for r in ranges if self.min_range < r < self.max_range]
            if valid_ranges:
                avg_range = np.mean(valid_ranges)
                min_range = np.min(valid_ranges)
                self.get_logger().debug(
                    f"Processed {self.processed_count} scans: "
                    f"{len(valid_ranges)}/{len(ranges)} valid, "
                    f"min={min_range:.2f}m, avg={avg_range:.2f}m, "
                    f"rate={self.processed_count/2:.1f} Hz"
                )
            self.processed_count = 0
            self.last_log_time = current_time
    
    def get_qos_description(self, qos_profile):
        """Get human-readable QoS description"""
        reliability_map = {
            0: "SYSTEM_DEFAULT",
            1: "RELIABLE",
            2: "BEST_EFFORT"
        }
        return f"Reliability: {reliability_map.get(qos_profile.reliability, 'UNKNOWN')}"

def main():
    rclpy.init()
    node = LaserBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Laser bridge shutting down...")
    except Exception as e:
        node.get_logger().error(f"Laser bridge error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
