#!/usr/bin/env python3
"""
Example script demonstrating programmatic use of the piec_analysis package.

This example shows how to:
1. Load experimental data
2. Run various analyses
3. Generate figures and tables
4. Access results programmatically
"""

import os
import sys
from piec_analysis import (
    DataLoader,
    PINNAnalyzer,
    NavigationAnalyzer,
    LocalizationAnalyzer,
    StatisticalTests,
    Visualizer,
    TableGenerator
)


def main():
    """Main example demonstrating package usage."""
    
    # Configuration
    data_dir = 'data/sample_data'
    output_dir = 'example_output'
    
    print("="*70)
    print("PIEC Analysis Package - Example Usage")
    print("="*70)
    
    # 1. Initialize components
    print("\n[1] Initializing components...")
    visualizer = Visualizer(output_dir, format='pdf', style='nature')
    table_gen = TableGenerator(output_dir)
    stats = StatisticalTests()
    loader = DataLoader(data_dir)
    
    # 2. Load and inspect data
    print("\n[2] Loading data...")
    try:
        pinn_history = loader.load_pinn_training_history()
        print(f"   Loaded PINN training history: {len(pinn_history)} epochs")
        
        pinn_predictions = loader.load_pinn_test_predictions()
        print(f"   Loaded PINN predictions: {len(pinn_predictions)} samples")
        
        nav_trials = loader.load_navigation_trials()
        print(f"   Loaded navigation trials: {len(nav_trials)} trials")
        methods = nav_trials['method'].unique()
        print(f"   Methods tested: {', '.join(methods)}")
    except FileNotFoundError as e:
        print(f"   Error loading data: {e}")
        return
    
    # 3. PINN Analysis
    print("\n[3] Running PINN analysis...")
    pinn_analyzer = PINNAnalyzer(data_dir)
    
    # Generate figures
    pinn_analyzer.generate_training_curves(visualizer)
    pinn_analyzer.generate_accuracy_plots(visualizer)
    
    # Get metrics
    metrics = pinn_analyzer.get_accuracy_metrics()
    print(f"   PINN Performance:")
    print(f"      MAPE: {metrics['mape']:.2f}%")
    print(f"      R²: {metrics['r2']:.4f}")
    print(f"      RMSE: {metrics['rmse_energy']:.2f} J")
    
    # Generate table
    table_gen.generate_pinn_accuracy_table(metrics)
    
    # Terrain-specific analysis
    terrain_metrics = pinn_analyzer.get_terrain_metrics()
    if not terrain_metrics.empty:
        print(f"   Terrain-specific results:")
        for _, row in terrain_metrics.iterrows():
            print(f"      {row['terrain']}: MAPE={row['mape']:.2f}%, R²={row['r2']:.4f}")
    
    # 4. Navigation Analysis
    print("\n[4] Running navigation analysis...")
    nav_analyzer = NavigationAnalyzer(data_dir)
    
    # Generate figures
    nav_analyzer.generate_energy_comparison(visualizer)
    
    # Get energy data for statistical tests
    energy_data = nav_analyzer.get_energy_data()
    
    print(f"   Energy consumption by method:")
    for method, energies in energy_data.items():
        print(f"      {method}: {energies.mean():.1f} ± {energies.std():.1f} J")
    
    # Get summary statistics
    summary = nav_analyzer.get_summary_statistics()
    print(f"\n   Summary statistics:")
    print(summary[['method', 'energy_mean', 'energy_std', 'success_rate']].to_string(index=False))
    
    # 5. Statistical Analysis
    print("\n[5] Running statistical tests...")
    
    # ANOVA
    anova_results = stats.energy_anova(energy_data)
    print(f"   ANOVA Results:")
    print(f"      F-statistic: {anova_results['F_statistic']:.2f}")
    print(f"      p-value: {anova_results['p_value']:.6f}")
    print(f"      Significant: {anova_results['significant']}")
    
    table_gen.generate_anova_table(anova_results)
    
    # Tukey HSD
    tukey_results = stats.tukey_hsd(energy_data)
    print(f"\n   Tukey HSD post-hoc test:")
    print(f"      {len(tukey_results)} pairwise comparisons")
    significant = tukey_results[tukey_results['reject'] == True]
    print(f"      {len(significant)} significant differences found")
    
    table_gen.generate_tukey_table(tukey_results)
    
    # Effect sizes
    effect_sizes = stats.cohens_d_analysis(energy_data, baseline='PIEC')
    print(f"\n   Effect sizes (Cohen's d):")
    for _, row in effect_sizes.iterrows():
        print(f"      {row['comparison']}: d={row['cohens_d']:.3f} ({row['interpretation']})")
    
    table_gen.generate_effect_size_table(effect_sizes)
    
    # 6. Localization Analysis (if data available)
    print("\n[6] Running localization analysis...")
    try:
        loc_analyzer = LocalizationAnalyzer(data_dir)
        loc_metrics = loc_analyzer.get_accuracy_metrics()
        
        print(f"   Localization accuracy:")
        for method, metrics in loc_metrics.items():
            print(f"      {method}: RMSE={metrics['position_rmse']:.3f} m")
    except FileNotFoundError:
        print("   Localization data not found, skipping...")
    
    # 7. Generate comparison tables
    print("\n[7] Generating comparison tables...")
    energy_by_env = loader.get_energy_by_environment(nav_trials)
    table_gen.generate_energy_comparison_table(energy_by_env, 'simulation')
    
    # 8. Summary
    print("\n" + "="*70)
    print("ANALYSIS COMPLETE")
    print("="*70)
    print(f"\n📊 Results saved to: {output_dir}/")
    print(f"   Figures: {os.path.join(output_dir, 'figures')}/")
    print(f"   Tables: {os.path.join(output_dir, 'tables')}/")
    
    # List generated files
    figures_dir = os.path.join(output_dir, 'figures')
    tables_dir = os.path.join(output_dir, 'tables')
    
    if os.path.exists(figures_dir):
        figures = [f for f in os.listdir(figures_dir) if f.endswith('.pdf')]
        print(f"\n   Generated {len(figures)} figures:")
        for fig in sorted(figures):
            print(f"      - {fig}")
    
    if os.path.exists(tables_dir):
        tables = [f for f in os.listdir(tables_dir) if f.endswith('.tex')]
        print(f"\n   Generated {len(tables)} tables:")
        for table in sorted(tables):
            print(f"      - {table}")
    
    print("\n" + "="*70)
    print("Use these results in your thesis/paper:")
    print("   LaTeX figure: \\includegraphics{figures/pinn_training_curves.pdf}")
    print("   LaTeX table: \\input{tables/table_energy_simulation.tex}")
    print("="*70)


if __name__ == '__main__':
    main()
