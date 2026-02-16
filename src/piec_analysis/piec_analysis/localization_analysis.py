"""
Localization analysis module for UKF vs EKF vs odometry comparison.
"""

import pandas as pd
import numpy as np
from typing import Dict


class LocalizationAnalyzer:
    """Analyze localization performance."""
    
    def __init__(self, data_dir: str):
        """
        Initialize localization analyzer.
        
        Args:
            data_dir: Directory containing localization data files
        """
        self.data_dir = data_dir
        self.timeseries_data = None
        
    def load_timeseries_data(self, filename: str = 'localization_timeseries.csv'):
        """Load localization time series data."""
        from .data_loader import DataLoader
        loader = DataLoader(self.data_dir)
        self.timeseries_data = loader.load_localization_timeseries(filename)
        return self.timeseries_data
    
    def generate_accuracy_comparison(self, visualizer):
        """Generate localization accuracy comparison plots."""
        if self.timeseries_data is None:
            self.load_timeseries_data()
        
        # Placeholder - would need visualizer method
        print("Localization accuracy comparison not fully implemented")
    
    def generate_error_cdf(self, visualizer):
        """Generate error CDF plot."""
        if self.timeseries_data is None:
            self.load_timeseries_data()
        
        # Placeholder - would need visualizer method
        print("Error CDF plot not fully implemented")
    
    def generate_sensor_fusion_analysis(self, visualizer):
        """Generate sensor fusion analysis plots."""
        # Placeholder - would need additional data
        print("Sensor fusion analysis not fully implemented (requires UKF internal states)")
    
    def get_accuracy_metrics(self) -> Dict[str, Dict[str, float]]:
        """
        Calculate accuracy metrics for each localization method.
        
        Returns:
            Dictionary mapping methods to their metrics
        """
        if self.timeseries_data is None:
            self.load_timeseries_data()
        
        metrics = {}
        
        for method in self.timeseries_data['method'].unique():
            method_data = self.timeseries_data[self.timeseries_data['method'] == method]
            
            # Calculate position errors
            pos_error_x = method_data['estimated_x'] - method_data['ground_truth_x']
            pos_error_y = method_data['estimated_y'] - method_data['ground_truth_y']
            pos_error = np.sqrt(pos_error_x**2 + pos_error_y**2)
            
            # Calculate orientation errors
            if 'estimated_theta' in method_data.columns:
                ori_error = method_data['estimated_theta'] - method_data['ground_truth_theta']
                # Normalize to [-pi, pi]
                ori_error = np.arctan2(np.sin(ori_error), np.cos(ori_error))
                ori_rmse = np.sqrt(np.mean(ori_error**2))
            else:
                ori_rmse = 0.0
            
            # Calculate metrics
            pos_rmse = np.sqrt(np.mean(pos_error**2))
            pos_95th = np.percentile(pos_error, 95)
            pos_max = np.max(pos_error)
            
            metrics[method] = {
                'position_rmse': pos_rmse,
                'orientation_rmse': ori_rmse,
                'position_95th': pos_95th,
                'position_max': pos_max
            }
        
        return metrics
