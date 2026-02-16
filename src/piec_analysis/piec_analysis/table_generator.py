"""
LaTeX table generator module for creating publication-ready tables.
"""

import pandas as pd
import numpy as np
import os
from typing import Dict, List, Optional


class TableGenerator:
    """Generate LaTeX tables for thesis and papers."""
    
    def __init__(self, output_dir: str):
        """
        Initialize table generator.
        
        Args:
            output_dir: Directory to save generated tables
        """
        self.output_dir = output_dir
        self.tables_dir = os.path.join(output_dir, 'tables')
        os.makedirs(self.tables_dir, exist_ok=True)
    
    def generate_pinn_accuracy_table(self, metrics: Dict[str, float], 
                                     filename: str = 'table_pinn_accuracy.tex'):
        """
        Generate PINN test set accuracy metrics table.
        
        Args:
            metrics: Dictionary with accuracy metrics
            filename: Output filename
        """
        filepath = os.path.join(self.tables_dir, filename)
        
        with open(filepath, 'w') as f:
            f.write("\\begin{table}[h]\n")
            f.write("\\centering\n")
            f.write("\\caption{PINN Test Set Accuracy Metrics}\n")
            f.write("\\label{tab:pinn_accuracy}\n")
            f.write("\\begin{tabular}{lr}\n")
            f.write("\\toprule\n")
            f.write("\\textbf{Metric} & \\textbf{Value} \\\\\n")
            f.write("\\midrule\n")
            
            # Define metric order and formatting
            metric_order = [
                ('MAPE', 'mape', '%.2f\\%%'),
                ('R²', 'r2', '%.4f'),
                ('RMSE (Energy)', 'rmse_energy', '%.2f J'),
                ('RMSE (Stability)', 'rmse_stability', '%.4f'),
                ('MAE', 'mae', '%.2f J'),
                ('Max Error', 'max_error', '%.2f J')
            ]
            
            for display_name, key, fmt in metric_order:
                if key in metrics:
                    value = metrics[key]
                    if 'MAPE' in display_name:
                        formatted_value = fmt % value
                    else:
                        formatted_value = fmt % value
                    f.write(f"{display_name} & {formatted_value} \\\\\n")
            
            f.write("\\bottomrule\n")
            f.write("\\end{tabular}\n")
            f.write("\\end{table}\n")
        
        print(f"Generated: {filename}")
    
    def generate_energy_comparison_table(self, data_dict: Dict, 
                                        experiment_type: str = 'simulation',
                                        filename: Optional[str] = None):
        """
        Generate energy consumption comparison table.
        
        Args:
            data_dict: Nested dict {method: {environment: {'mean': x, 'std': y}}}
            experiment_type: 'simulation' or 'experimental'
            filename: Output filename (auto-generated if None)
        """
        if filename is None:
            filename = f"table_energy_{experiment_type}.tex"
        
        filepath = os.path.join(self.tables_dir, filename)
        
        with open(filepath, 'w') as f:
            f.write("\\begin{table}[h]\n")
            f.write("\\centering\n")
            f.write(f"\\caption{{Energy consumption comparison ({experiment_type}, mean ± std)}}\n")
            f.write(f"\\label{{tab:energy_{experiment_type}}}\n")
            
            # Get environments and methods
            methods = list(data_dict.keys())
            if len(methods) == 0:
                return
            
            environments = list(data_dict[methods[0]].keys())
            n_cols = len(environments) + 2  # environments + method + avg vs PIEC
            
            f.write(f"\\begin{{tabular}}{{l{'c' * (n_cols-1)}}}\n")
            f.write("\\toprule\n")
            
            # Header row
            if experiment_type == 'simulation':
                env_names = ['Indoor (J)', 'Office (J)', 'Outdoor (J)']
            else:
                env_names = ['Corridor (J)', 'Plaza (J)', 'Parking (J)']
            
            header = "\\textbf{Method} & " + " & ".join([f"\\textbf{{{name}}}" for name in env_names])
            header += " & \\textbf{Avg. vs. PIEC} \\\\\n"
            f.write(header)
            f.write("\\midrule\n")
            
            # Calculate PIEC baseline
            piec_avg = 0
            if 'PIEC' in data_dict:
                piec_avg = np.mean([data_dict['PIEC'][env]['mean'] 
                                   for env in environments])
            
            # Data rows
            for method in methods:
                row_values = []
                method_avg = 0
                
                for env in environments:
                    if env in data_dict[method]:
                        mean = data_dict[method][env]['mean']
                        std = data_dict[method][env]['std']
                        row_values.append(f"{mean:.1f}±{std:.1f}")
                        method_avg += mean
                    else:
                        row_values.append("--")
                
                if len(environments) > 0:
                    method_avg /= len(environments)
                
                # Calculate percentage vs PIEC
                if piec_avg > 0:
                    vs_piec = ((method_avg - piec_avg) / piec_avg) * 100
                else:
                    vs_piec = 0
                
                # Format method name
                method_name = f"\\textbf{{{method}}}" if method == 'PIEC' else method
                
                # Write row
                row_str = method_name + " & " + " & ".join(row_values)
                if method == 'PIEC':
                    row_str += " & -- \\\\\n"
                else:
                    row_str += f" & +{vs_piec:.1f}\\% \\\\\n"
                f.write(row_str)
            
            f.write("\\bottomrule\n")
            f.write("\\end{tabular}\n")
            f.write("\\end{table}\n")
        
        print(f"Generated: {filename}")
    
    def generate_anova_table(self, anova_results: Dict, 
                            filename: str = 'table_anova_energy.tex'):
        """
        Generate ANOVA results table.
        
        Args:
            anova_results: Dictionary with ANOVA results
            filename: Output filename
        """
        filepath = os.path.join(self.tables_dir, filename)
        
        with open(filepath, 'w') as f:
            f.write("\\begin{table}[h]\n")
            f.write("\\centering\n")
            f.write("\\caption{ANOVA Results for Energy Consumption}\n")
            f.write("\\label{tab:anova_energy}\n")
            f.write("\\begin{tabular}{lrrrrr}\n")
            f.write("\\toprule\n")
            f.write("\\textbf{Source} & \\textbf{SS} & \\textbf{df} & \\textbf{MS} & ")
            f.write("\\textbf{F-statistic} & \\textbf{p-value} \\\\\n")
            f.write("\\midrule\n")
            
            # Between methods row
            f.write(f"Between methods & {anova_results['ss_between']:.2f} & ")
            f.write(f"{anova_results['df_between']} & {anova_results['ms_between']:.2f} & ")
            f.write(f"{anova_results['F_statistic']:.2f} & ")
            
            p_val = anova_results['p_value']
            if p_val < 0.001:
                f.write("< 0.001*** \\\\\n")
            elif p_val < 0.01:
                f.write(f"{p_val:.3f}** \\\\\n")
            elif p_val < 0.05:
                f.write(f"{p_val:.3f}* \\\\\n")
            else:
                f.write(f"{p_val:.3f} \\\\\n")
            
            # Within methods row
            f.write(f"Within methods & {anova_results['ss_within']:.2f} & ")
            f.write(f"{anova_results['df_within']} & {anova_results['ms_within']:.2f} & ")
            f.write("-- & -- \\\\\n")
            
            # Total row
            f.write(f"Total & {anova_results['ss_total']:.2f} & ")
            f.write(f"{anova_results['df_total']} & -- & -- & -- \\\\\n")
            
            f.write("\\bottomrule\n")
            f.write("\\end{tabular}\n")
            f.write("\\end{table}\n")
        
        print(f"Generated: {filename}")
    
    def generate_tukey_table(self, tukey_df: pd.DataFrame, 
                            filename: str = 'table_tukey_hsd.tex'):
        """
        Generate Tukey HSD post-hoc test results table.
        
        Args:
            tukey_df: DataFrame with Tukey HSD results
            filename: Output filename
        """
        filepath = os.path.join(self.tables_dir, filename)
        
        with open(filepath, 'w') as f:
            f.write("\\begin{table}[h]\n")
            f.write("\\centering\n")
            f.write("\\caption{Tukey HSD Post-Hoc Test Results}\n")
            f.write("\\label{tab:tukey_hsd}\n")
            f.write("\\begin{tabular}{llrrr}\n")
            f.write("\\toprule\n")
            f.write("\\textbf{Method Pair} & \\textbf{Mean Diff. (J)} & ")
            f.write("\\textbf{95\\% CI} & \\textbf{p-value} & \\textbf{Sig.?} \\\\\n")
            f.write("\\midrule\n")
            
            for _, row in tukey_df.iterrows():
                pair = f"{row['group1']} vs {row['group2']}"
                mean_diff = row['meandiff']
                ci_lower = row['lower']
                ci_upper = row['upper']
                ci_str = f"[{ci_lower:.2f}, {ci_upper:.2f}]"
                
                # Get p-value if available
                if 'p_value' in row:
                    p_val = row['p_value']
                    if p_val < 0.001:
                        p_str = "< 0.001"
                    else:
                        p_str = f"{p_val:.3f}"
                else:
                    p_str = "--"
                
                sig = "Yes" if row['reject'] else "No"
                
                f.write(f"{pair} & {mean_diff:.2f} & {ci_str} & {p_str} & {sig} \\\\\n")
            
            f.write("\\bottomrule\n")
            f.write("\\end{tabular}\n")
            f.write("\\end{table}\n")
        
        print(f"Generated: {filename}")
    
    def generate_effect_size_table(self, effect_df: pd.DataFrame, 
                                   filename: str = 'table_effect_size.tex'):
        """
        Generate Cohen's d effect size table.
        
        Args:
            effect_df: DataFrame with effect size results
            filename: Output filename
        """
        filepath = os.path.join(self.tables_dir, filename)
        
        with open(filepath, 'w') as f:
            f.write("\\begin{table}[h]\n")
            f.write("\\centering\n")
            f.write("\\caption{Effect Size (Cohen's d) Analysis}\n")
            f.write("\\label{tab:effect_size}\n")
            f.write("\\begin{tabular}{lrl}\n")
            f.write("\\toprule\n")
            f.write("\\textbf{Comparison} & \\textbf{Cohen's d} & ")
            f.write("\\textbf{Interpretation} \\\\\n")
            f.write("\\midrule\n")
            
            for _, row in effect_df.iterrows():
                comparison = row['comparison']
                d = row['cohens_d']
                interpretation = row['interpretation'].capitalize()
                
                f.write(f"{comparison} & {d:.3f} & {interpretation} \\\\\n")
            
            f.write("\\bottomrule\n")
            f.write("\\end{tabular}\n")
            f.write("\\end{table}\n")
        
        print(f"Generated: {filename}")
