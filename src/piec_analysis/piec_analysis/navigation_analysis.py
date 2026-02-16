"""
Navigation analysis module for energy, path length, and success rate analysis.
"""

import pandas as pd
import numpy as np
from typing import Dict, List


class NavigationAnalyzer:
    """Analyze navigation performance experiments."""
    
    def __init__(self, data_dir: str):
        """
        Initialize navigation analyzer.
        
        Args:
            data_dir: Directory containing navigation data files
        """
        self.data_dir = data_dir
        self.trials_data = None
        self.nsga_data = None
        
    def load_trials_data(self, filename: str = 'navigation_trials.csv'):
        """Load navigation trial data."""
        from .data_loader import DataLoader
        loader = DataLoader(self.data_dir)
        self.trials_data = loader.load_navigation_trials(filename)
        return self.trials_data
    
    def load_nsga_data(self, filename: str = 'nsga_evolution.csv'):
        """Load NSGA-II evolution data."""
        from .data_loader import DataLoader
        loader = DataLoader(self.data_dir)
        self.nsga_data = loader.load_nsga_evolution(filename)
        return self.nsga_data
    
    def generate_energy_comparison(self, visualizer):
        """Generate energy comparison figures."""
        if self.trials_data is None:
            self.load_trials_data()
        
        # Get energy data by method and environment
        from .data_loader import DataLoader
        loader = DataLoader(self.data_dir)
        energy_by_env = loader.get_energy_by_environment(self.trials_data)
        
        # Get list of environments
        environments = list(next(iter(energy_by_env.values())).keys())
        
        visualizer.plot_energy_comparison(energy_by_env, environments)
    
    def generate_path_characteristics(self, visualizer):
        """Generate path characteristics comparison."""
        if self.trials_data is None:
            self.load_trials_data()
        
        # Placeholder - would need more detailed implementation
        print("Path characteristics analysis not fully implemented")
    
    def generate_success_analysis(self, visualizer):
        """Generate mission success rate and failure analysis."""
        if self.trials_data is None:
            self.load_trials_data()
        
        # Calculate success rates by method
        success_rates = self.trials_data.groupby('method')['success'].mean() * 100
        print("Success rates by method:")
        print(success_rates)
    
    def generate_pareto_fronts(self, visualizer):
        """Generate Pareto front visualizations."""
        if self.nsga_data is None:
            try:
                self.load_nsga_data()
                visualizer.plot_pareto_front(self.nsga_data)
            except FileNotFoundError:
                print("Warning: NSGA evolution data not found, skipping Pareto front analysis")
    
    def generate_nsga_convergence(self, visualizer):
        """Generate NSGA-II convergence analysis."""
        if self.nsga_data is None:
            try:
                self.load_nsga_data()
            except FileNotFoundError:
                print("Warning: NSGA evolution data not found, skipping convergence analysis")
                return
        
        # Placeholder for convergence plots
        print("NSGA convergence analysis not fully implemented")
    
    def get_energy_data(self) -> Dict[str, np.ndarray]:
        """
        Get energy consumption data by method.
        
        Returns:
            Dictionary mapping method names to energy arrays
        """
        if self.trials_data is None:
            self.load_trials_data()
        
        from .data_loader import DataLoader
        loader = DataLoader(self.data_dir)
        return loader.get_energy_by_method(self.trials_data)
    
    def get_summary_statistics(self) -> pd.DataFrame:
        """
        Get summary statistics for all methods.
        
        Returns:
            DataFrame with summary statistics
        """
        if self.trials_data is None:
            self.load_trials_data()
        
        results = []
        
        for method in self.trials_data['method'].unique():
            method_data = self.trials_data[self.trials_data['method'] == method]
            
            stats = {
                'method': method,
                'energy_mean': method_data['energy_consumed'].mean(),
                'energy_std': method_data['energy_consumed'].std(),
                'success_rate': method_data['success'].mean() * 100,
                'n_trials': len(method_data)
            }
            
            # Add optional columns if available
            if 'path_length' in method_data.columns:
                stats['path_length_mean'] = method_data['path_length'].mean()
            if 'mission_time' in method_data.columns:
                stats['mission_time_mean'] = method_data['mission_time'].mean()
            if 'clearance_min' in method_data.columns:
                stats['clearance_mean'] = method_data['clearance_min'].mean()
            
            results.append(stats)
        
        return pd.DataFrame(results)
