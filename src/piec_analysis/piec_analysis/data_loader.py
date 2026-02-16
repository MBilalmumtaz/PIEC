"""
Data loader module for loading experimental data from logs and CSV files.
"""

import pandas as pd
import numpy as np
import os
from typing import Dict, List, Tuple, Optional


class DataLoader:
    """Load and preprocess experimental data for analysis."""
    
    def __init__(self, data_dir: str):
        """
        Initialize data loader.
        
        Args:
            data_dir: Directory containing experimental data files
        """
        self.data_dir = data_dir
        
    def load_pinn_training_history(self, filename: str = 'pinn_training_history.csv') -> pd.DataFrame:
        """
        Load PINN training history data.
        
        Expected columns: epoch, train_loss, val_loss, data_loss, physics_loss, learning_rate
        
        Args:
            filename: Name of the CSV file
            
        Returns:
            DataFrame with training history
        """
        filepath = os.path.join(self.data_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
        
        df = pd.read_csv(filepath)
        required_cols = ['epoch', 'train_loss', 'val_loss']
        for col in required_cols:
            if col not in df.columns:
                raise ValueError(f"Missing required column: {col}")
        
        return df
    
    def load_pinn_test_predictions(self, filename: str = 'pinn_test_predictions.csv') -> pd.DataFrame:
        """
        Load PINN test predictions.
        
        Expected columns: actual_energy, predicted_energy, terrain_type, velocity, slope, roughness
        
        Args:
            filename: Name of the CSV file
            
        Returns:
            DataFrame with test predictions
        """
        filepath = os.path.join(self.data_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
        
        df = pd.read_csv(filepath)
        required_cols = ['actual_energy', 'predicted_energy']
        for col in required_cols:
            if col not in df.columns:
                raise ValueError(f"Missing required column: {col}")
        
        return df
    
    def load_navigation_trials(self, filename: str = 'navigation_trials.csv') -> pd.DataFrame:
        """
        Load navigation experiment trial data.
        
        Expected columns: trial_id, method, environment, energy_consumed, 
                         path_length, mission_time, success, failure_mode, clearance_min
        
        Args:
            filename: Name of the CSV file
            
        Returns:
            DataFrame with navigation trial data
        """
        filepath = os.path.join(self.data_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
        
        df = pd.read_csv(filepath)
        required_cols = ['method', 'environment', 'energy_consumed', 'success']
        for col in required_cols:
            if col not in df.columns:
                raise ValueError(f"Missing required column: {col}")
        
        return df
    
    def load_localization_timeseries(self, filename: str = 'localization_timeseries.csv') -> pd.DataFrame:
        """
        Load localization time series data.
        
        Expected columns: timestamp, method, estimated_x, estimated_y, estimated_theta,
                         ground_truth_x, ground_truth_y, ground_truth_theta
        
        Args:
            filename: Name of the CSV file
            
        Returns:
            DataFrame with localization time series
        """
        filepath = os.path.join(self.data_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
        
        df = pd.read_csv(filepath)
        required_cols = ['timestamp', 'method', 'estimated_x', 'ground_truth_x']
        for col in required_cols:
            if col not in df.columns:
                raise ValueError(f"Missing required column: {col}")
        
        return df
    
    def load_nsga_evolution(self, filename: str = 'nsga_evolution.csv') -> pd.DataFrame:
        """
        Load NSGA-II population evolution data.
        
        Expected columns: generation, individual_id, front_rank, crowding_distance,
                         f1_length, f2_curvature, f3_obstacle, f4_localization,
                         f5_deviation, f6_energy, f7_stability
        
        Args:
            filename: Name of the CSV file
            
        Returns:
            DataFrame with NSGA-II evolution data
        """
        filepath = os.path.join(self.data_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
        
        df = pd.read_csv(filepath)
        required_cols = ['generation', 'front_rank', 'f6_energy']
        for col in required_cols:
            if col not in df.columns:
                raise ValueError(f"Missing required column: {col}")
        
        return df
    
    def load_ablation_results(self, filename: str = 'ablation_results.csv') -> pd.DataFrame:
        """
        Load ablation study results.
        
        Expected columns: configuration, energy, success_rate, localization_rmse, planning_time
        
        Args:
            filename: Name of the CSV file
            
        Returns:
            DataFrame with ablation results
        """
        filepath = os.path.join(self.data_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
        
        df = pd.read_csv(filepath)
        return df
    
    def get_energy_by_method(self, df: pd.DataFrame) -> Dict[str, np.ndarray]:
        """
        Extract energy consumption data grouped by method.
        
        Args:
            df: Navigation trials DataFrame
            
        Returns:
            Dictionary mapping method names to energy arrays
        """
        energy_dict = {}
        for method in df['method'].unique():
            method_data = df[df['method'] == method]
            energy_dict[method] = method_data['energy_consumed'].values
        
        return energy_dict
    
    def get_energy_by_environment(self, df: pd.DataFrame) -> Dict[str, Dict[str, float]]:
        """
        Extract energy consumption statistics by method and environment.
        
        Args:
            df: Navigation trials DataFrame
            
        Returns:
            Nested dictionary: {method: {environment: {'mean': x, 'std': y}}}
        """
        result = {}
        
        for method in df['method'].unique():
            result[method] = {}
            method_data = df[df['method'] == method]
            
            for env in method_data['environment'].unique():
                env_data = method_data[method_data['environment'] == env]
                result[method][env] = {
                    'mean': env_data['energy_consumed'].mean(),
                    'std': env_data['energy_consumed'].std(),
                    'count': len(env_data)
                }
        
        return result
