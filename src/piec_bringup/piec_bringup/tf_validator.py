#!/usr/bin/env python3
"""
TF Validator - Checks transform tree consistency at launch time
Validates that required transforms exist and are reasonable
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


class TFValidator(Node):
    def __init__(self):
        super().__init__('tf_validator')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Define required transforms to validate
        # Can be made configurable via ROS parameters in future versions
        self.required_transforms = [
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'imu_link'),
            ('base_link', 'rslidar'),
        ]
        
        # Define expected transform properties (parent, child, expected_translation, expected_rotation_yaw)
        # None means any value is acceptable
        self.expected_transforms = {
            ('base_footprint', 'base_link'): {
                'z_min': 0.20, 'z_max': 0.25,  # Height should be ~0.237m
                'xy_max': 0.05,  # X and Y should be near zero
                'description': 'Base height offset'
            },
            ('base_link', 'imu_link'): {
                'z_min': 0.15, 'z_max': 0.25,  # IMU at ~0.2m height
                'xy_max': 0.05,
                'yaw_expected': -0.94,  # -54 degrees
                'yaw_tolerance': 0.2,   # ±11 degrees tolerance
                'description': 'IMU mount with yaw offset'
            },
            ('base_link', 'rslidar'): {
                'x_min': 0.1, 'x_max': 0.2,   # Forward of base
                'z_min': 0.25, 'z_max': 0.35,  # Above base
                'description': 'LiDAR mount position'
            },
        }
        
        self.get_logger().info("TF Validator initialized - waiting for TF tree to populate...")
        
        # Wait for TF tree to be available using a timer-based approach
        # This is more ROS2-friendly than blocking sleep
        self.validation_timer = self.create_timer(3.0, self.validate_all_callback)
        self.validation_done = False
    
    def validate_all_callback(self):
        """Timer callback to validate TF tree after delay"""
        if not self.validation_done:
            self.validate_all()
            self.validation_done = True
            self.validation_timer.cancel()
    
    def validate_all(self):
        """Validate all required transforms"""
        all_valid = True
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("TF TREE VALIDATION")
        self.get_logger().info("=" * 60)
        
        for parent, child in self.required_transforms:
            valid = self.validate_transform(parent, child)
            if not valid:
                all_valid = False
        
        self.get_logger().info("=" * 60)
        if all_valid:
            self.get_logger().info("✅ TF TREE VALIDATION PASSED - All transforms OK")
        else:
            self.get_logger().error("❌ TF TREE VALIDATION FAILED - Check errors above")
        self.get_logger().info("=" * 60)
        
        return all_valid
    
    def validate_transform(self, parent_frame, child_frame):
        """Validate a single transform"""
        try:
            # Try to get transform
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Extract translation and rotation
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # Convert quaternion to yaw
            yaw = math.atan2(
                2.0 * (rot.w * rot.z + rot.x * rot.y),
                1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
            )
            
            # Check if we have expectations for this transform
            key = (parent_frame, child_frame)
            if key in self.expected_transforms:
                expected = self.expected_transforms[key]
                issues = []
                
                # Check X coordinate
                if 'x_min' in expected and trans.x < expected['x_min']:
                    issues.append(f"X too small: {trans.x:.3f} < {expected['x_min']}")
                if 'x_max' in expected and trans.x > expected['x_max']:
                    issues.append(f"X too large: {trans.x:.3f} > {expected['x_max']}")
                
                # Check Y coordinate
                if 'y_min' in expected and trans.y < expected['y_min']:
                    issues.append(f"Y too small: {trans.y:.3f} < {expected['y_min']}")
                if 'y_max' in expected and trans.y > expected['y_max']:
                    issues.append(f"Y too large: {trans.y:.3f} > {expected['y_max']}")
                
                # Check Z coordinate
                if 'z_min' in expected and trans.z < expected['z_min']:
                    issues.append(f"Z too small: {trans.z:.3f} < {expected['z_min']}")
                if 'z_max' in expected and trans.z > expected['z_max']:
                    issues.append(f"Z too large: {trans.z:.3f} > {expected['z_max']}")
                
                # Check XY distance from origin
                if 'xy_max' in expected:
                    xy_dist = math.sqrt(trans.x**2 + trans.y**2)
                    if xy_dist > expected['xy_max']:
                        issues.append(f"XY offset too large: {xy_dist:.3f} > {expected['xy_max']}")
                
                # Check yaw
                if 'yaw_expected' in expected:
                    yaw_diff = abs(yaw - expected['yaw_expected'])
                    # Handle wraparound
                    if yaw_diff > math.pi:
                        yaw_diff = 2 * math.pi - yaw_diff
                    if yaw_diff > expected['yaw_tolerance']:
                        issues.append(
                            f"Yaw incorrect: {yaw:.3f} rad ({math.degrees(yaw):.1f}°), "
                            f"expected {expected['yaw_expected']:.3f} rad "
                            f"({math.degrees(expected['yaw_expected']):.1f}°)"
                        )
                
                if issues:
                    self.get_logger().warn(
                        f"⚠️  {parent_frame} -> {child_frame}: {expected['description']}"
                    )
                    self.get_logger().warn(f"    Translation: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]")
                    self.get_logger().warn(f"    Yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
                    for issue in issues:
                        self.get_logger().warn(f"    ISSUE: {issue}")
                    return False
                else:
                    self.get_logger().info(
                        f"✅ {parent_frame} -> {child_frame}: {expected['description']} - OK"
                    )
                    self.get_logger().debug(f"    Translation: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]")
                    self.get_logger().debug(f"    Yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
                    return True
            else:
                # No specific expectations, just check it exists
                self.get_logger().info(f"✅ {parent_frame} -> {child_frame}: EXISTS")
                self.get_logger().debug(f"    Translation: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]")
                self.get_logger().debug(f"    Yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
                return True
                
        except TransformException as ex:
            self.get_logger().error(f"❌ {parent_frame} -> {child_frame}: MISSING")
            self.get_logger().error(f"    Error: {ex}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    validator = TFValidator()
    
    # Spin until validation completes (timer-based, non-blocking)
    while rclpy.ok() and not validator.validation_done:
        rclpy.spin_once(validator, timeout_sec=0.5)
    
    validator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
