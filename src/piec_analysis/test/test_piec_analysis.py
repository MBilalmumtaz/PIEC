"""
Test suite for piec_analysis package.
"""

import unittest
import os
import sys
import pandas as pd
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from piec_analysis import DataLoader, PINNAnalyzer, NavigationAnalyzer
from piec_analysis import StatisticalTests, Visualizer, TableGenerator


class TestDataLoader(unittest.TestCase):
    """Test data loading functionality."""
    
    def setUp(self):
        """Set up test data directory."""
        self.test_data_dir = os.path.join(
            os.path.dirname(__file__), 
            '../data/sample_data'
        )
        self.loader = DataLoader(self.test_data_dir)
    
    def test_load_pinn_training_history(self):
        """Test loading PINN training history."""
        df = self.loader.load_pinn_training_history()
        self.assertIsInstance(df, pd.DataFrame)
        self.assertIn('epoch', df.columns)
        self.assertIn('train_loss', df.columns)
        self.assertIn('val_loss', df.columns)
        self.assertGreater(len(df), 0)
    
    def test_load_pinn_test_predictions(self):
        """Test loading PINN test predictions."""
        df = self.loader.load_pinn_test_predictions()
        self.assertIsInstance(df, pd.DataFrame)
        self.assertIn('actual_energy', df.columns)
        self.assertIn('predicted_energy', df.columns)
        self.assertGreater(len(df), 0)
    
    def test_load_navigation_trials(self):
        """Test loading navigation trials."""
        df = self.loader.load_navigation_trials()
        self.assertIsInstance(df, pd.DataFrame)
        self.assertIn('method', df.columns)
        self.assertIn('energy_consumed', df.columns)
        self.assertGreater(len(df), 0)


class TestStatisticalTests(unittest.TestCase):
    """Test statistical test functionality."""
    
    def setUp(self):
        """Set up test data."""
        self.stats = StatisticalTests()
        # Create sample data
        np.random.seed(42)
        self.energy_dict = {
            'PIEC': np.random.normal(150, 10, 20),
            'ASTAR': np.random.normal(200, 15, 20),
            'RRT': np.random.normal(180, 12, 20)
        }
    
    def test_energy_anova(self):
        """Test ANOVA calculation."""
        results = self.stats.energy_anova(self.energy_dict)
        self.assertIn('F_statistic', results)
        self.assertIn('p_value', results)
        self.assertIn('significant', results)
        self.assertIsInstance(results['F_statistic'], float)
        self.assertIsInstance(results['p_value'], float)
    
    def test_cohens_d(self):
        """Test Cohen's d calculation."""
        d = self.stats.cohens_d(
            self.energy_dict['PIEC'],
            self.energy_dict['ASTAR']
        )
        self.assertIsInstance(d, float)
        # PIEC should have lower energy, so d should be negative
        self.assertLess(d, 0)
    
    def test_cohens_d_analysis(self):
        """Test Cohen's d analysis for all methods."""
        df = self.stats.cohens_d_analysis(self.energy_dict, baseline='PIEC')
        self.assertIsInstance(df, pd.DataFrame)
        self.assertEqual(len(df), 2)  # 2 comparisons (ASTAR and RRT vs PIEC)
        self.assertIn('cohens_d', df.columns)
        self.assertIn('interpretation', df.columns)


class TestPINNAnalyzer(unittest.TestCase):
    """Test PINN analysis functionality."""
    
    def setUp(self):
        """Set up analyzer."""
        self.test_data_dir = os.path.join(
            os.path.dirname(__file__),
            '../data/sample_data'
        )
        self.analyzer = PINNAnalyzer(self.test_data_dir)
    
    def test_get_accuracy_metrics(self):
        """Test accuracy metrics calculation."""
        metrics = self.analyzer.get_accuracy_metrics()
        self.assertIn('mape', metrics)
        self.assertIn('r2', metrics)
        self.assertIn('rmse_energy', metrics)
        self.assertGreater(metrics['r2'], 0.9)  # Should have high R²
        self.assertLess(metrics['mape'], 10)  # MAPE should be low


class TestNavigationAnalyzer(unittest.TestCase):
    """Test navigation analysis functionality."""
    
    def setUp(self):
        """Set up analyzer."""
        self.test_data_dir = os.path.join(
            os.path.dirname(__file__),
            '../data/sample_data'
        )
        self.analyzer = NavigationAnalyzer(self.test_data_dir)
    
    def test_get_energy_data(self):
        """Test energy data extraction."""
        energy_dict = self.analyzer.get_energy_data()
        self.assertIsInstance(energy_dict, dict)
        self.assertGreater(len(energy_dict), 0)
        for method, energies in energy_dict.items():
            self.assertIsInstance(energies, np.ndarray)
            self.assertGreater(len(energies), 0)
    
    def test_get_summary_statistics(self):
        """Test summary statistics."""
        df = self.analyzer.get_summary_statistics()
        self.assertIsInstance(df, pd.DataFrame)
        self.assertIn('method', df.columns)
        self.assertIn('energy_mean', df.columns)
        self.assertIn('success_rate', df.columns)


if __name__ == '__main__':
    unittest.main()
