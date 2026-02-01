#!/usr/bin/env python3
"""
Unit tests for TractionModel
"""
import unittest
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from piec_dynamics.traction_model import TractionModel


class TestTractionModel(unittest.TestCase):
    """Test TractionModel initialization and basic functionality"""
    
    def test_init_default_mass(self):
        """Test that TractionModel initializes with default mass"""
        model = TractionModel()
        self.assertEqual(model.mass, 50.0)
        self.assertEqual(model.track_width, 0.4)
        self.assertEqual(model.wheelbase, 0.5)
        self.assertEqual(model.cg_height, 0.3)
    
    def test_init_custom_mass(self):
        """Test that TractionModel initializes with custom mass"""
        model = TractionModel(mass=75.0)
        self.assertEqual(model.mass, 75.0)
    
    def test_calculate_load_transfer(self):
        """Test load transfer calculation doesn't crash"""
        model = TractionModel(mass=50.0)
        long_transfer, lat_transfer = model.calculate_load_transfer(
            acceleration=1.0,
            deceleration=0.0,
            lateral_acc=0.5
        )
        # Just verify it returns numeric values
        self.assertIsInstance(long_transfer, (int, float))
        self.assertIsInstance(lat_transfer, (int, float))
    
    def test_stability_metric(self):
        """Test stability metric calculation"""
        model = TractionModel(mass=50.0)
        stability = model.stability_metric(
            velocity=1.0,
            curvature=0.1,
            slope=0.0,
            terrain_type='asphalt'
        )
        # Stability should be between 0 and 1
        self.assertGreaterEqual(stability, 0.0)
        self.assertLessEqual(stability, 1.0)


if __name__ == '__main__':
    unittest.main()
