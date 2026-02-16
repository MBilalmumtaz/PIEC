"""
Visualization module for creating publication-quality plots.
"""

import matplotlib.pyplot as plt
import matplotlib as mpl
import seaborn as sns
import numpy as np
import pandas as pd
import os
from typing import Optional, Dict, List


class Visualizer:
    """Generate publication-quality figures for thesis and papers."""
    
    def __init__(self, output_dir: str, format: str = 'both', style: str = 'nature'):
        """
        Initialize visualizer.
        
        Args:
            output_dir: Directory to save generated figures
            format: Output format ('pdf', 'png', or 'both')
            style: Plot style ('nature' for Nature journal standards)
        """
        self.output_dir = output_dir
        self.format = format
        self.figures_dir = os.path.join(output_dir, 'figures')
        os.makedirs(self.figures_dir, exist_ok=True)
        
        if style == 'nature':
            self._setup_nature_style()
        
    def _setup_nature_style(self):
        """Configure matplotlib for Nature journal figures."""
        # Nature requirements: 89mm (single column) or 183mm (double column)
        mpl.rcParams['figure.figsize'] = (7.2, 5.4)  # 183mm width
        mpl.rcParams['font.size'] = 8
        mpl.rcParams['font.family'] = 'sans-serif'
        mpl.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']
        mpl.rcParams['axes.labelsize'] = 8
        mpl.rcParams['axes.titlesize'] = 9
        mpl.rcParams['xtick.labelsize'] = 7
        mpl.rcParams['ytick.labelsize'] = 7
        mpl.rcParams['legend.fontsize'] = 7
        mpl.rcParams['lines.linewidth'] = 1.0
        mpl.rcParams['axes.linewidth'] = 0.5
        mpl.rcParams['pdf.fonttype'] = 42  # TrueType fonts for editing
        mpl.rcParams['ps.fonttype'] = 42
        
        # Use Nature color palette
        self.colors = {
            'piec': '#0C5DA5',     # Blue
            'astar': '#FF2C00',    # Red
            'rrt': '#FF9500',      # Orange
            'dwa': '#00B945',      # Green
            'nsga': '#845B97',     # Purple
            'ukf': '#0C5DA5',
            'ekf': '#FF9500',
            'odom': '#FF2C00'
        }
        
    def save_figure(self, fig, filename: str):
        """
        Save figure in requested format(s).
        
        Args:
            fig: Matplotlib figure object
            filename: Base filename (without extension)
        """
        if self.format in ['pdf', 'both']:
            fig.savefig(os.path.join(self.figures_dir, f"{filename}.pdf"), 
                       dpi=300, bbox_inches='tight')
        if self.format in ['png', 'both']:
            fig.savefig(os.path.join(self.figures_dir, f"{filename}.png"), 
                       dpi=300, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved figure: {filename}")
        
    def plot_training_curves(self, history_df: pd.DataFrame):
        """Generate PINN training curves figure."""
        fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.4))
        
        # (a) Total loss
        axes[0, 0].plot(history_df['epoch'], history_df['train_loss'], 
                       label='Train', linewidth=1.5)
        axes[0, 0].plot(history_df['epoch'], history_df['val_loss'], 
                       label='Validation', linewidth=1.5)
        axes[0, 0].set_xlabel('Epoch')
        axes[0, 0].set_ylabel('Total Loss')
        axes[0, 0].legend()
        axes[0, 0].grid(alpha=0.3, linewidth=0.5)
        axes[0, 0].set_title('(a) Training and Validation Loss')
        
        # (b) Loss components
        if 'data_loss' in history_df.columns and 'physics_loss' in history_df.columns:
            axes[0, 1].plot(history_df['epoch'], history_df['data_loss'], 
                           label='Data Loss', linewidth=1.5)
            axes[0, 1].plot(history_df['epoch'], history_df['physics_loss'], 
                           label='Physics Loss', linewidth=1.5)
            axes[0, 1].set_xlabel('Epoch')
            axes[0, 1].set_ylabel('Loss')
            axes[0, 1].legend()
            axes[0, 1].grid(alpha=0.3, linewidth=0.5)
            axes[0, 1].set_title('(b) Loss Components')
        
        # (c) Learning rate schedule
        if 'learning_rate' in history_df.columns:
            axes[1, 0].plot(history_df['epoch'], history_df['learning_rate'], 
                           linewidth=1.5, color='red')
            axes[1, 0].set_xlabel('Epoch')
            axes[1, 0].set_ylabel('Learning Rate')
            axes[1, 0].set_yscale('log')
            axes[1, 0].grid(alpha=0.3, linewidth=0.5)
            axes[1, 0].set_title('(c) Learning Rate Schedule')
        
        # (d) Early stopping indicator
        best_epoch = history_df['val_loss'].idxmin()
        axes[1, 1].plot(history_df['epoch'], history_df['val_loss'], 
                       linewidth=1.5)
        axes[1, 1].axvline(best_epoch, color='red', linestyle='--', 
                          label=f'Best Epoch ({best_epoch})')
        axes[1, 1].set_xlabel('Epoch')
        axes[1, 1].set_ylabel('Validation Loss')
        axes[1, 1].legend()
        axes[1, 1].grid(alpha=0.3, linewidth=0.5)
        axes[1, 1].set_title('(d) Best Model Selection')
        
        plt.tight_layout()
        self.save_figure(fig, 'pinn_training_curves')
        
    def plot_prediction_accuracy(self, predictions_df: pd.DataFrame):
        """Generate PINN prediction accuracy figure."""
        fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.4))
        
        actual = predictions_df['actual_energy'].values
        predicted = predictions_df['predicted_energy'].values
        residuals = actual - predicted
        
        # (a) Scatter plot: Predicted vs Actual
        axes[0, 0].scatter(actual, predicted, alpha=0.5, s=10)
        min_val = min(actual.min(), predicted.min())
        max_val = max(actual.max(), predicted.max())
        axes[0, 0].plot([min_val, max_val], [min_val, max_val], 'r--', linewidth=1, label='Perfect')
        # ±5% error bands
        axes[0, 0].plot([min_val, max_val], [min_val*1.05, max_val*1.05], 'g--', linewidth=0.5, alpha=0.5)
        axes[0, 0].plot([min_val, max_val], [min_val*0.95, max_val*0.95], 'g--', linewidth=0.5, alpha=0.5)
        axes[0, 0].set_xlabel('Actual Energy (J)')
        axes[0, 0].set_ylabel('Predicted Energy (J)')
        # Calculate R²
        from sklearn.metrics import r2_score
        r2 = r2_score(actual, predicted)
        axes[0, 0].text(0.05, 0.95, f'R² = {r2:.4f}', transform=axes[0, 0].transAxes, 
                       verticalalignment='top')
        axes[0, 0].legend()
        axes[0, 0].grid(alpha=0.3, linewidth=0.5)
        axes[0, 0].set_title('(a) Predicted vs Actual Energy')
        
        # (b) Residual histogram
        axes[0, 1].hist(residuals, bins=30, density=True, alpha=0.7, edgecolor='black')
        # Gaussian fit overlay
        mu, std = residuals.mean(), residuals.std()
        x = np.linspace(residuals.min(), residuals.max(), 100)
        axes[0, 1].plot(x, 1/(std * np.sqrt(2 * np.pi)) * np.exp(-0.5 * ((x - mu) / std) ** 2), 
                       'r-', linewidth=2, label=f'N({mu:.2f}, {std:.2f})')
        axes[0, 1].set_xlabel('Residual (J)')
        axes[0, 1].set_ylabel('Density')
        axes[0, 1].legend()
        axes[0, 1].grid(alpha=0.3, linewidth=0.5)
        axes[0, 1].set_title('(b) Residual Distribution')
        
        # (c) Residual vs Predicted
        axes[1, 0].scatter(predicted, residuals, alpha=0.5, s=10)
        axes[1, 0].axhline(0, color='r', linestyle='--', linewidth=1)
        axes[1, 0].set_xlabel('Predicted Energy (J)')
        axes[1, 0].set_ylabel('Residual (J)')
        axes[1, 0].grid(alpha=0.3, linewidth=0.5)
        axes[1, 0].set_title('(c) Residual vs Predicted')
        
        # (d) Q-Q plot
        from scipy import stats
        stats.probplot(residuals, dist="norm", plot=axes[1, 1])
        axes[1, 1].set_title('(d) Q-Q Plot')
        axes[1, 1].grid(alpha=0.3, linewidth=0.5)
        
        plt.tight_layout()
        self.save_figure(fig, 'pinn_accuracy')
        
    def plot_energy_comparison(self, energy_dict: Dict[str, np.ndarray], 
                               environments: List[str], title: str = 'Energy Comparison'):
        """Generate energy comparison bar chart."""
        fig, ax = plt.subplots(figsize=(7.2, 4.0))
        
        methods = list(energy_dict.keys())
        x = np.arange(len(environments))
        width = 0.15
        
        for i, method in enumerate(methods):
            means = []
            stds = []
            for env in environments:
                env_data = energy_dict[method].get(env, {'mean': 0, 'std': 0})
                means.append(env_data['mean'])
                stds.append(env_data['std'])
            
            offset = (i - len(methods)/2) * width
            color = self.colors.get(method.lower().replace('+', '').replace('*', '').replace('-', ''), 
                                   f'C{i}')
            ax.bar(x + offset, means, width, yerr=stds, label=method, 
                   capsize=3, color=color, alpha=0.8)
        
        ax.set_xlabel('Environment')
        ax.set_ylabel('Energy Consumption (J)')
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(environments, rotation=15, ha='right')
        ax.legend()
        ax.grid(axis='y', alpha=0.3, linewidth=0.5)
        
        plt.tight_layout()
        self.save_figure(fig, 'energy_comparison')
        
    def plot_pareto_front(self, pareto_df: pd.DataFrame):
        """Generate Pareto front visualization."""
        fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.5))
        
        # (a) 2D Pareto front: Energy vs Path Length
        front_1 = pareto_df[pareto_df['front_rank'] == 1]
        front_2 = pareto_df[pareto_df['front_rank'] == 2]
        dominated = pareto_df[pareto_df['front_rank'] > 2]
        
        if len(dominated) > 0:
            axes[0].scatter(dominated['f1_length'], dominated['f6_energy'], 
                           c='gray', alpha=0.3, s=10, label='Dominated')
        if len(front_2) > 0:
            axes[0].scatter(front_2['f1_length'], front_2['f6_energy'], 
                           c='orange', alpha=0.6, s=15, label='Front 2')
        if len(front_1) > 0:
            axes[0].scatter(front_1['f1_length'], front_1['f6_energy'], 
                           c='blue', s=20, label='Pareto Front (F1)')
        
        axes[0].set_xlabel('Path Length (m)')
        axes[0].set_ylabel('Energy (J)')
        axes[0].legend()
        axes[0].grid(alpha=0.3, linewidth=0.5)
        axes[0].set_title('(a) Pareto Front: Energy vs Length')
        
        # (b) Hypervolume convergence
        if 'generation' in pareto_df.columns:
            gens = pareto_df['generation'].unique()
            # Placeholder for hypervolume calculation
            axes[1].plot(gens, gens * 0 + 100, label='PIEC', linewidth=1.5)
            axes[1].set_xlabel('Generation')
            axes[1].set_ylabel('Hypervolume Indicator')
            axes[1].legend()
            axes[1].grid(alpha=0.3, linewidth=0.5)
            axes[1].set_title('(b) Convergence')
        
        plt.tight_layout()
        self.save_figure(fig, 'pareto_fronts')
