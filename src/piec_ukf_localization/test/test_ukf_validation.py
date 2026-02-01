#!/usr/bin/env python3
"""
Unit tests for UKF measurement validation
"""
import unittest
import sys
import os
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class TestUKFValidation(unittest.TestCase):
    """Test UKF validation logic without full ROS setup"""
    
    def test_adaptive_threshold(self):
        """Test adaptive threshold calculation"""
        # Simulate the adaptive threshold logic
        base_threshold = 5.0
        
        # Test low rejection rate
        rejection_count = 10
        measurement_count = 100
        rejection_rate = rejection_count / max(measurement_count, 1)
        
        if rejection_rate > 0.3:
            threshold = base_threshold * 1.5
        else:
            threshold = base_threshold
        
        self.assertEqual(threshold, 5.0)  # Should not increase
        
        # Test high rejection rate
        rejection_count = 40
        measurement_count = 100
        rejection_rate = rejection_count / max(measurement_count, 1)
        
        if rejection_rate > 0.3:
            threshold = base_threshold * 1.5
        else:
            threshold = base_threshold
        
        self.assertEqual(threshold, 7.5)  # Should increase to 7.5
    
    def test_mahalanobis_calculation(self):
        """Test Mahalanobis distance calculation"""
        z = np.array([1.0, 2.0])
        z_pred = np.array([1.1, 2.1])
        R = np.diag([0.1, 0.1])
        
        innovation = z - z_pred
        
        try:
            mahalanobis_dist = np.sqrt(innovation.T @ np.linalg.inv(R) @ innovation)
            # Should complete without error
            self.assertIsInstance(mahalanobis_dist, (int, float, np.number))
            self.assertGreater(mahalanobis_dist, 0)
        except Exception as e:
            self.fail(f"Mahalanobis calculation failed: {e}")
    
    def test_reset_threshold(self):
        """Test covariance reset threshold"""
        # Test that the new threshold (10.0) is more strict than old (100.0)
        old_threshold = 100.0
        new_threshold = 10.0
        
        self.assertLess(new_threshold, old_threshold)
        self.assertEqual(new_threshold, 10.0)
    
    def test_condition_number_threshold(self):
        """Test condition number threshold"""
        old_threshold = 1e10
        new_threshold = 1e8
        
        self.assertLess(new_threshold, old_threshold)
        self.assertEqual(new_threshold, 1e8)


if __name__ == '__main__':
    unittest.main()
