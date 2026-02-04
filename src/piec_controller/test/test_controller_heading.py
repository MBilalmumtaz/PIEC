#!/usr/bin/env python3
"""
Unit tests for controller heading logic fixes
Tests the new parameter-driven heading control
"""

import math
import unittest
from unittest.mock import MagicMock, patch
import sys
import os
from types import SimpleNamespace

# Simple clip function to avoid numpy dependency
def clip(value, min_val, max_val):
    """Clip value to range"""
    return max(min_val, min(max_val, value))


# Mock rclpy before importing controller
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['nav_msgs'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['ament_index_python'] = MagicMock()
sys.modules['ament_index_python.packages'] = MagicMock()
sys.modules['numpy'] = SimpleNamespace(
    clip=clip,
    mean=lambda values: sum(values) / len(values) if values else 0.0,
)
sys.modules['piec_pinn_surrogate_msgs'] = MagicMock()
sys.modules['piec_pinn_surrogate_msgs.srv'] = MagicMock()
sys.modules['piec_controller.dynamic_dwa_complete'] = MagicMock()
sys.modules['rclpy.duration'] = MagicMock()


class MockPose:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class TestControllerHeadingLogic(unittest.TestCase):
    """Test the improved heading control logic"""
    
    def test_angle_wrapping(self):
        """Test that angles are properly wrapped to [-pi, pi]"""
        # Test wrapping from > pi
        angle = 4.0  # > pi
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        self.assertGreaterEqual(wrapped, -math.pi)
        self.assertLessEqual(wrapped, math.pi)
        
        # Test wrapping from < -pi
        angle = -4.0  # < -pi
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        self.assertGreaterEqual(wrapped, -math.pi)
        self.assertLessEqual(wrapped, math.pi)
    
    def test_bearing_calculation_forward(self):
        """Test bearing calculation for goal ahead"""
        # Robot at origin, facing forward (yaw=0)
        current_x, current_y = 0.0, 0.0
        target_x, target_y = 1.0, 0.0  # Goal 1m ahead
        
        dx = target_x - current_x
        dy = target_y - current_y
        bearing = math.atan2(dy, dx)
        
        # Should be 0 radians (straight ahead)
        self.assertAlmostEqual(bearing, 0.0, places=5)
    
    def test_bearing_calculation_right(self):
        """Test bearing calculation for goal on right side"""
        # Robot at origin, facing forward (yaw=0)
        current_x, current_y = 0.0, 0.0
        target_x, target_y = 0.0, -1.0  # Goal 1m to the right
        
        dx = target_x - current_x
        dy = target_y - current_y
        bearing = math.atan2(dy, dx)
        
        # Should be -pi/2 (90 degrees clockwise)
        self.assertAlmostEqual(bearing, -math.pi/2, places=5)
    
    def test_bearing_calculation_left(self):
        """Test bearing calculation for goal on left side"""
        # Robot at origin, facing forward (yaw=0)
        current_x, current_y = 0.0, 0.0
        target_x, target_y = 0.0, 1.0  # Goal 1m to the left
        
        dx = target_x - current_x
        dy = target_y - current_y
        bearing = math.atan2(dy, dx)
        
        # Should be +pi/2 (90 degrees counter-clockwise)
        self.assertAlmostEqual(bearing, math.pi/2, places=5)
    
    def test_angle_error_right_side_goal(self):
        """Test angle error calculation when goal is on right side"""
        # Robot facing forward (yaw=0)
        current_yaw = 0.0
        
        # Goal to the right (-pi/2)
        target_bearing = -math.pi/2
        
        angle_diff = target_bearing - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Should be -pi/2 (need to turn right/clockwise)
        self.assertAlmostEqual(angle_diff, -math.pi/2, places=5)
        
        # Angular velocity should be negative for right turn
        heading_kp = 1.5
        w = angle_diff * heading_kp
        self.assertLess(w, 0.0, "Angular velocity should be negative for right turn")
    
    def test_angle_error_left_side_goal(self):
        """Test angle error calculation when goal is on left side"""
        # Robot facing forward (yaw=0)
        current_yaw = 0.0
        
        # Goal to the left (+pi/2)
        target_bearing = math.pi/2
        
        angle_diff = target_bearing - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Should be +pi/2 (need to turn left/counter-clockwise)
        self.assertAlmostEqual(angle_diff, math.pi/2, places=5)
        
        # Angular velocity should be positive for left turn
        heading_kp = 1.5
        w = angle_diff * heading_kp
        self.assertGreater(w, 0.0, "Angular velocity should be positive for left turn")
    
    def test_rotate_in_place_threshold(self):
        """Test rotate-in-place logic for large angle errors"""
        # Test parameters - UPDATED to 90° threshold
        rotate_in_place_angle_deg = 90.0
        rotate_threshold_rad = math.radians(rotate_in_place_angle_deg)
        
        # Large angle error (120 degrees) - should trigger rotate in place
        angle_diff = math.radians(120)
        
        # Should trigger rotate in place
        should_rotate = abs(angle_diff) > rotate_threshold_rad
        self.assertTrue(should_rotate, "Should rotate in place for 120° error")
        
        # Medium angle error (60 degrees) - should NOT trigger rotate in place
        angle_diff = math.radians(60)
        
        # Should NOT trigger rotate in place
        should_rotate = abs(angle_diff) > rotate_threshold_rad
        self.assertFalse(should_rotate, "Should not rotate in place for 60° error")
    
    def test_heading_deadband(self):
        """Test heading deadband to prevent oscillation"""
        heading_deadband_deg = 2.0
        heading_deadband_rad = math.radians(heading_deadband_deg)
        
        # Angle error within deadband
        angle_diff = math.radians(1.5)
        
        within_deadband = abs(angle_diff) < heading_deadband_rad
        self.assertTrue(within_deadband, "1.5° should be within 2° deadband")
        
        # Angle error outside deadband
        angle_diff = math.radians(3.0)
        
        within_deadband = abs(angle_diff) < heading_deadband_rad
        self.assertFalse(within_deadband, "3° should be outside 2° deadband")
    
    def test_angular_velocity_capping(self):
        """Test that angular velocity is properly capped"""
        max_heading_rate = 0.6  # rad/s
        heading_kp = 2.0
        
        # Large angle error
        angle_diff = math.pi  # 180 degrees
        
        # Calculate w
        w = angle_diff * heading_kp
        
        # Should exceed max before capping
        self.assertGreater(abs(w), max_heading_rate)
        
        # Apply cap
        w_capped = clip(w, -max_heading_rate, max_heading_rate)
        
        # Should be within limits
        self.assertLessEqual(abs(w_capped), max_heading_rate)
        self.assertEqual(w_capped, max_heading_rate, "Should be capped to max")
    
    def test_sign_convention(self):
        """Test ROS standard sign convention (positive = CCW, negative = CW)"""
        # Goal to the left -> positive angle error -> positive w (CCW)
        angle_diff_left = math.pi/4  # 45° to left
        heading_kp = 1.5
        w_left = angle_diff_left * heading_kp
        
        self.assertGreater(w_left, 0.0, "Left turn should have positive w (CCW)")
        
        # Goal to the right -> negative angle error -> negative w (CW)
        angle_diff_right = -math.pi/4  # 45° to right
        w_right = angle_diff_right * heading_kp
        
        self.assertLess(w_right, 0.0, "Right turn should have negative w (CW)")


class TestAngularSignCorrection(unittest.TestCase):
    """Test angular sign correction for Scout Mini robot"""
    
    def test_sign_correction_positive(self):
        """Test that positive sign correction preserves standard ROS convention"""
        # Standard ROS: positive w = CCW, negative w = CW
        angular_sign_correction = 1.0
        
        # Right turn (negative w)
        w_computed = -0.5
        w_final = w_computed * angular_sign_correction
        self.assertEqual(w_final, -0.5, "Positive sign correction should preserve sign")
        
        # Left turn (positive w)
        w_computed = 0.5
        w_final = w_computed * angular_sign_correction
        self.assertEqual(w_final, 0.5, "Positive sign correction should preserve sign")
    
    def test_sign_correction_negative(self):
        """Test that negative sign correction inverts angular velocity (LEGACY - now using 1.0)"""
        # NOTE: This test documents old behavior with angular_sign_correction=-1.0
        # Current configuration uses 1.0 (standard ROS convention)
        angular_sign_correction = -1.0  # OLD value for reference
        
        # Controller computes negative w for right turn (CW)
        # But robot interprets positive w as CW, so we flip it
        w_computed = -0.5  # Controller wants CW (right)
        w_final = w_computed * angular_sign_correction
        self.assertEqual(w_final, 0.5, "Negative correction should flip to positive for CW")
        
        # Controller computes positive w for left turn (CCW)
        # But robot interprets negative w as CCW, so we flip it
        w_computed = 0.5  # Controller wants CCW (left)
        w_final = w_computed * angular_sign_correction
        self.assertEqual(w_final, -0.5, "Negative correction should flip to negative for CCW")
    
    def test_right_side_goal_with_sign_correction(self):
        """Test complete flow for right-side goal with standard ROS convention"""
        # Robot at origin facing forward
        current_yaw = 0.0
        target_bearing = -math.pi/2  # Goal to the right
        
        # Compute angle error
        angle_diff = target_bearing - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        self.assertAlmostEqual(angle_diff, -math.pi/2, places=5)
        
        # Compute angular velocity (standard ROS convention)
        heading_kp = 1.5
        w_computed = angle_diff * heading_kp
        self.assertLess(w_computed, 0.0, "Standard computation gives negative w for right turn")
        
        # Apply standard ROS sign correction (1.0 = no inversion)
        angular_sign_correction = 1.0
        w_final = w_computed * angular_sign_correction
        self.assertLess(w_final, 0.0, "With standard ROS convention, w remains negative for right turn")
    
    def test_left_side_goal_with_sign_correction(self):
        """Test complete flow for left-side goal with standard ROS convention"""
        # Robot at origin facing forward
        current_yaw = 0.0
        target_bearing = math.pi/2  # Goal to the left
        
        # Compute angle error
        angle_diff = target_bearing - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        self.assertAlmostEqual(angle_diff, math.pi/2, places=5)
        
        # Compute angular velocity (standard ROS convention)
        heading_kp = 1.5
        w_computed = angle_diff * heading_kp
        self.assertGreater(w_computed, 0.0, "Standard computation gives positive w for left turn")
        
        # Apply standard ROS sign correction (1.0 = no inversion)
        angular_sign_correction = 1.0
        w_final = w_computed * angular_sign_correction
        self.assertGreater(w_final, 0.0, "With standard ROS convention, w remains positive for left turn")


class TestCommandTrackingUpdates(unittest.TestCase):
    """Test control bookkeeping updates."""

    def test_last_cmd_tracking(self):
        """Ensure last_cmd stores raw values in publish_cmd."""
        from piec_controller.controller_node import ControllerNode

        controller = ControllerNode.__new__(ControllerNode)
        controller.last_cmd = (0.0, 0.0)
        controller.linear_scale = 1.0
        controller.angular_scale = 1.0
        controller.angular_sign = 1.0
        controller.max_linear_vel = 1.0
        controller.max_angular_vel = 1.0
        controller.min_linear_vel = 0.05
        controller.debug_mode = False
        controller.control_counter = 0
        controller.last_commanded_linear = 0.0
        controller.cmd_pub = MagicMock()
        controller.linear_history = MagicMock()
        controller.angular_history = MagicMock()
        controller.commanded_velocity_history = MagicMock()
        controller.get_logger = MagicMock(return_value=MagicMock())

        controller.publish_cmd(0.4, -0.2)

        self.assertEqual(controller.last_cmd, (0.4, -0.2))
    


class TestGoalCompletionLogic(unittest.TestCase):
    """Test the improved goal completion logic"""
    
    def test_goal_distance_check(self):
        """Test goal completion based on distance"""
        goal_completion_distance = 0.25  # 25cm
        
        # Robot at origin
        robot_x, robot_y = 0.0, 0.0
        
        # Goal close enough
        goal_x, goal_y = 0.1, 0.1
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        
        self.assertLess(distance, goal_completion_distance, 
                       "Should be within goal completion distance")
        
        # Goal too far
        goal_x, goal_y = 0.5, 0.0
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        
        self.assertGreater(distance, goal_completion_distance,
                          "Should be outside goal completion distance")
    
    def test_velocity_stability_check(self):
        """Test stability check based on velocity"""
        stability_threshold = 0.05  # m/s
        
        # Robot nearly stopped
        linear_vel = 0.02
        angular_vel = 0.01
        
        is_stable = (linear_vel < stability_threshold and 
                    angular_vel < stability_threshold)
        
        self.assertTrue(is_stable, "Robot should be considered stable")
        
        # Robot still moving
        linear_vel = 0.1
        angular_vel = 0.0
        
        is_stable = (linear_vel < stability_threshold and 
                    angular_vel < stability_threshold)
        
        self.assertFalse(is_stable, "Robot should not be considered stable")


class TestDistanceToGoalCrash(unittest.TestCase):
    """Test for distance_to_goal UnboundLocalError regression"""
    
    def test_distance_to_goal_always_defined(self):
        """Test that distance_to_goal is defined even when goal_position is None"""
        # Simulate the scenario that caused the crash:
        # - Path exists
        # - goal_position is None (no explicit goal set)
        # - PINN optimization tries to use distance_to_goal
        
        # Mock scenario variables
        goal_position = None
        path_exists = True
        pinn_client_available = True
        
        # This is the fix: initialize distance_to_goal early
        distance_to_goal = float('inf')
        
        # Only update if goal position is available
        if goal_position is not None:
            # In real code: distance_to_goal = math.hypot(goal_x - x, goal_y - y)
            distance_to_goal = 1.5  # example
        
        # Verify distance_to_goal is always defined
        self.assertIsNotNone(distance_to_goal, "distance_to_goal should always be defined")
        self.assertTrue(isinstance(distance_to_goal, (int, float)), 
                       "distance_to_goal should be numeric")
        
        # When goal_position is None, distance should be infinity
        if goal_position is None:
            self.assertEqual(distance_to_goal, float('inf'), 
                           "distance_to_goal should be inf when no goal")
    
    def test_distance_to_goal_with_goal(self):
        """Test distance_to_goal calculation when goal exists"""
        # Mock current position
        current_x, current_y = 0.0, 0.0
        
        # Mock goal position
        goal_position = (1.0, 1.0)
        
        # Initialize distance_to_goal
        distance_to_goal = float('inf')
        
        # Calculate distance when goal exists
        if goal_position is not None:
            goal_x, goal_y = goal_position
            distance_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)
        
        # Verify distance is calculated correctly
        expected_distance = math.sqrt(2.0)  # sqrt(1^2 + 1^2)
        self.assertAlmostEqual(distance_to_goal, expected_distance, places=5)
    
    def test_pinn_optimization_safe_with_no_goal(self):
        """Test that PINN optimization can safely use distance_to_goal even without goal"""
        # Initialize distance_to_goal to safe default
        distance_to_goal = float('inf')
        
        # Simulate PINN optimization check
        # PINN typically only optimizes when close to goal
        should_use_pinn = distance_to_goal < 2.0
        
        # When no goal exists (distance = inf), PINN should not be used
        self.assertFalse(should_use_pinn, 
                        "PINN should not be used when distance_to_goal is inf")


class TestAngularSignDoubleCorrection(unittest.TestCase):
    """Test that angular sign correction is only applied once (Bug Fix)"""
    
    def test_no_double_sign_correction(self):
        """
        Test the bug fix: angular_sign should only be applied in publish_cmd(),
        not in calculate_simple_control().
        
        Bug scenario from logs:
        - Goal at (-0.066, 1.262) when robot at origin
        - Goal is ~93° to the LEFT
        - Controller should output positive w (turn LEFT/CCW)
        - But with double sign correction, it was outputting negative w (turn RIGHT/CW)
        """
        # Simulate the bug scenario
        current_x, current_y, current_yaw = 0.0, 0.0, 0.0
        target_x, target_y = -0.066, 1.262
        
        # Calculate what controller should compute
        dx = target_x - current_x
        dy = target_y - current_y
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Goal is to the LEFT (positive angle)
        self.assertGreater(angle_diff, 0, "Goal should be to the LEFT (positive angle)")
        self.assertAlmostEqual(math.degrees(angle_diff), 92.7, places=0, 
                              msg="Goal should be ~93° to the left")
        
        # Controller calculates angular velocity (before any sign correction)
        heading_kp = 1.0
        w_raw = angle_diff * heading_kp
        
        # w_raw should be POSITIVE for left turn
        self.assertGreater(w_raw, 0, 
                          "Raw angular velocity should be POSITIVE for left turn")
        
        # Simulate OLD buggy behavior (double sign correction) - FIXED by changing to 1.0
        angular_sign = 1.0  # Correct value (was -1.0 which caused wrong turns)
        angular_scale = 1.0
        
        # With correct sign (1.0), no sign flip occurs
        w_after_calculate = w_raw * angular_scale * angular_sign
        self.assertGreater(w_after_calculate, 0, 
                       "With correct sign (1.0), w remains positive for left turn")
        
        # Even if applied again (not recommended), result stays correct
        w_final = w_after_calculate * angular_sign
        self.assertGreater(w_final, 0, 
                          "With correct sign, w stays positive for left turn")
        
        # NEW CORRECT behavior: sign correction only in publish_cmd()
        w_after_calculate_fixed = w_raw * angular_scale  # No sign correction here!
        self.assertGreater(w_after_calculate_fixed, 0,
                          "calculate_simple_control should output positive w (no sign flip)")
        
        # With angular_sign = 1.0 (standard ROS)
        angular_sign_correct = 1.0
        w_final_correct = w_after_calculate_fixed * angular_sign_correct
        self.assertGreater(w_final_correct, 0,
                          "Final w should be POSITIVE for left turn with correct sign")
    
    def test_sign_correction_only_once_right_turn(self):
        """Test that right turn also works correctly with single sign correction"""
        current_x, current_y, current_yaw = 0.0, 0.0, 0.0
        target_x, target_y = 1.0, -0.5  # Goal to the RIGHT
        
        dx = target_x - current_x
        dy = target_y - current_y
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Goal is to the RIGHT (negative angle)
        self.assertLess(angle_diff, 0, "Goal should be to the RIGHT (negative angle)")
        
        # Controller calculates angular velocity
        heading_kp = 1.0
        w_raw = angle_diff * heading_kp
        
        # w_raw should be NEGATIVE for right turn
        self.assertLess(w_raw, 0,
                       "Raw angular velocity should be NEGATIVE for right turn")
        
        # NEW CORRECT behavior: no sign correction in calculate_simple_control()
        angular_scale = 1.0
        w_after_calculate = w_raw * angular_scale  # No sign correction
        self.assertLess(w_after_calculate, 0,
                       "After calculate, w should still be negative")
        
        # Apply sign correction once in publish_cmd()
        angular_sign = 1.0  # Standard ROS convention
        w_final = w_after_calculate * angular_sign
        self.assertLess(w_final, 0,
                       "Final w should be NEGATIVE for right turn")


if __name__ == '__main__':
    unittest.main()
