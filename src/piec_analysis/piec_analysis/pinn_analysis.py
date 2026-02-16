"""
PINN analysis module for training curves and accuracy metrics.
"""

import pandas as pd
import numpy as np
from typing import Dict, Optional
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error


class PINNAnalyzer:
    """Analyze PINN training and performance."""
    
    def __init__(self, data_dir: str):
        """
        Initialize PINN analyzer.
        
        Args:
            data_dir: Directory containing PINN data files
        """
        self.data_dir = data_dir
        self.training_history = None
        self.test_predictions = None
        
    def load_training_history(self, filename: str = 'pinn_training_history.csv'):
        """Load PINN training history."""
        from .data_loader import DataLoader
        loader = DataLoader(self.data_dir)
        self.training_history = loader.load_pinn_training_history(filename)
        return self.training_history
    
    def load_test_predictions(self, filename: str = 'pinn_test_predictions.csv'):
        """Load PINN test predictions."""
        from .data_loader import DataLoader
        loader = DataLoader(self.data_dir)
        self.test_predictions = loader.load_pinn_test_predictions(filename)
        return self.test_predictions
    
    def generate_training_curves(self, visualizer):
        """Generate training curves figure."""
        if self.training_history is None:
            self.load_training_history()
        
        visualizer.plot_training_curves(self.training_history)
    
    def generate_accuracy_plots(self, visualizer):
        """Generate accuracy analysis plots."""
        if self.test_predictions is None:
            self.load_test_predictions()
        
        visualizer.plot_prediction_accuracy(self.test_predictions)
    
    def generate_terrain_analysis(self, visualizer):
        """Generate terrain-specific accuracy analysis."""
        if self.test_predictions is None:
            self.load_test_predictions()
        
        if 'terrain_type' not in self.test_predictions.columns:
            print("Warning: No terrain_type column found, skipping terrain analysis")
            return
        
        # Calculate metrics by terrain
        terrain_metrics = {}
        for terrain in self.test_predictions['terrain_type'].unique():
            terrain_data = self.test_predictions[
                self.test_predictions['terrain_type'] == terrain
            ]
            
            actual = terrain_data['actual_energy'].values
            predicted = terrain_data['predicted_energy'].values
            
            mape = np.mean(np.abs((actual - predicted) / actual)) * 100
            r2 = r2_score(actual, predicted)
            rmse = np.sqrt(mean_squared_error(actual, predicted))
            
            terrain_metrics[terrain] = {
                'mape': mape,
                'r2': r2,
                'rmse': rmse,
                'count': len(terrain_data)
            }
        
        # Create visualization (placeholder - would need visualizer method)
        print("Terrain analysis metrics:", terrain_metrics)
    
    def generate_timing_analysis(self, visualizer):
        """Generate inference timing analysis."""
        # Placeholder - would need timing data
        print("Timing analysis not implemented (requires timing data)")
    
    def get_accuracy_metrics(self) -> Dict[str, float]:
        """
        Calculate comprehensive accuracy metrics.
        
        Returns:
            Dictionary with accuracy metrics
        """
        if self.test_predictions is None:
            self.load_test_predictions()
        
        actual = self.test_predictions['actual_energy'].values
        predicted = self.test_predictions['predicted_energy'].values
        
        # Calculate metrics
        mape = np.mean(np.abs((actual - predicted) / actual)) * 100
        r2 = r2_score(actual, predicted)
        rmse_energy = np.sqrt(mean_squared_error(actual, predicted))
        mae = mean_absolute_error(actual, predicted)
        max_error = np.max(np.abs(actual - predicted))
        
        # RMSE for stability (if available)
        rmse_stability = 0.0
        if 'actual_stability' in self.test_predictions.columns:
            actual_stab = self.test_predictions['actual_stability'].values
            pred_stab = self.test_predictions['predicted_stability'].values
            rmse_stability = np.sqrt(mean_squared_error(actual_stab, pred_stab))
        
        return {
            'mape': mape,
            'r2': r2,
            'rmse_energy': rmse_energy,
            'rmse_stability': rmse_stability,
            'mae': mae,
            'max_error': max_error
        }
    
    def get_terrain_metrics(self) -> pd.DataFrame:
        """
        Get accuracy metrics by terrain type.
        
        Returns:
            DataFrame with terrain-specific metrics
        """
        if self.test_predictions is None:
            self.load_test_predictions()
        
        if 'terrain_type' not in self.test_predictions.columns:
            return pd.DataFrame()
        
        results = []
        for terrain in self.test_predictions['terrain_type'].unique():
            terrain_data = self.test_predictions[
                self.test_predictions['terrain_type'] == terrain
            ]
            
            actual = terrain_data['actual_energy'].values
            predicted = terrain_data['predicted_energy'].values
            
            mape = np.mean(np.abs((actual - predicted) / actual)) * 100
            r2 = r2_score(actual, predicted)
            rmse = np.sqrt(mean_squared_error(actual, predicted))
            
            results.append({
                'terrain': terrain,
                'mape': mape,
                'r2': r2,
                'rmse': rmse,
                'count': len(terrain_data)
            })
        
        return pd.DataFrame(results)
