#!/usr/bin/env python3
"""
Main script to generate all thesis/paper results.
"""

import argparse
import os
import sys


def main():
    """Main entry point for generating all results."""
    parser = argparse.ArgumentParser(
        description='Generate all PIEC thesis/paper results'
    )
    parser.add_argument(
        '--data-dir',
        type=str,
        required=True,
        help='Directory containing experimental data'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='output',
        help='Directory for generated figures and tables'
    )
    parser.add_argument(
        '--format',
        choices=['pdf', 'png', 'both'],
        default='both',
        help='Output format for figures'
    )
    parser.add_argument(
        '--skip-pinn',
        action='store_true',
        help='Skip PINN analysis'
    )
    parser.add_argument(
        '--skip-navigation',
        action='store_true',
        help='Skip navigation analysis'
    )
    parser.add_argument(
        '--skip-localization',
        action='store_true',
        help='Skip localization analysis'
    )
    parser.add_argument(
        '--skip-ablation',
        action='store_true',
        help='Skip ablation analysis'
    )
    parser.add_argument(
        '--skip-statistics',
        action='store_true',
        help='Skip statistical tests'
    )
    args = parser.parse_args()
    
    # Import analysis modules
    try:
        from piec_analysis import (
            PINNAnalyzer,
            NavigationAnalyzer,
            LocalizationAnalyzer,
            AblationAnalyzer,
            StatisticalTests,
            TableGenerator,
            Visualizer
        )
    except ImportError as e:
        print(f"Error importing piec_analysis modules: {e}")
        print("Make sure the package is installed: colcon build")
        sys.exit(1)
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Initialize components
    visualizer = Visualizer(args.output_dir, format=args.format)
    table_gen = TableGenerator(args.output_dir)
    stats = StatisticalTests()
    
    print("="*70)
    print("PIEC Analysis - Generating All Results")
    print("="*70)
    print(f"Data directory: {args.data_dir}")
    print(f"Output directory: {args.output_dir}")
    print(f"Figure format: {args.format}")
    print("="*70)
    
    # PINN Analysis
    if not args.skip_pinn:
        print("\n[1/5] Generating PINN analysis figures...")
        try:
            pinn_analyzer = PINNAnalyzer(args.data_dir)
            pinn_analyzer.generate_training_curves(visualizer)
            pinn_analyzer.generate_accuracy_plots(visualizer)
            pinn_analyzer.generate_terrain_analysis(visualizer)
            pinn_analyzer.generate_timing_analysis(visualizer)
            
            # Generate PINN tables
            metrics = pinn_analyzer.get_accuracy_metrics()
            table_gen.generate_pinn_accuracy_table(metrics)
            
            terrain_metrics = pinn_analyzer.get_terrain_metrics()
            if not terrain_metrics.empty:
                print(f"  Generated terrain metrics for {len(terrain_metrics)} terrains")
            
            print("  ✓ PINN analysis complete")
        except FileNotFoundError as e:
            print(f"  ⚠ Warning: {e}")
        except Exception as e:
            print(f"  ✗ Error in PINN analysis: {e}")
    
    # Navigation Performance Analysis
    if not args.skip_navigation:
        print("\n[2/5] Generating navigation performance figures...")
        try:
            nav_analyzer = NavigationAnalyzer(args.data_dir)
            nav_analyzer.generate_energy_comparison(visualizer)
            nav_analyzer.generate_path_characteristics(visualizer)
            nav_analyzer.generate_success_analysis(visualizer)
            nav_analyzer.generate_pareto_fronts(visualizer)
            nav_analyzer.generate_nsga_convergence(visualizer)
            
            # Generate navigation tables
            energy_data = nav_analyzer.get_energy_data()
            summary_stats = nav_analyzer.get_summary_statistics()
            print(f"  Generated statistics for {len(summary_stats)} methods")
            
            print("  ✓ Navigation analysis complete")
        except FileNotFoundError as e:
            print(f"  ⚠ Warning: {e}")
        except Exception as e:
            print(f"  ✗ Error in navigation analysis: {e}")
    
    # Localization Analysis
    if not args.skip_localization:
        print("\n[3/5] Generating localization figures...")
        try:
            loc_analyzer = LocalizationAnalyzer(args.data_dir)
            loc_analyzer.generate_accuracy_comparison(visualizer)
            loc_analyzer.generate_error_cdf(visualizer)
            loc_analyzer.generate_sensor_fusion_analysis(visualizer)
            
            # Generate localization table
            loc_metrics = loc_analyzer.get_accuracy_metrics()
            print(f"  Generated metrics for {len(loc_metrics)} methods")
            
            print("  ✓ Localization analysis complete")
        except FileNotFoundError as e:
            print(f"  ⚠ Warning: {e}")
        except Exception as e:
            print(f"  ✗ Error in localization analysis: {e}")
    
    # Ablation Study
    if not args.skip_ablation:
        print("\n[4/5] Generating ablation study figures...")
        try:
            ablation_analyzer = AblationAnalyzer(args.data_dir)
            ablation_analyzer.generate_component_ablation(visualizer)
            ablation_analyzer.generate_parameter_sensitivity(visualizer)
            
            print("  ✓ Ablation analysis complete")
        except FileNotFoundError as e:
            print(f"  ⚠ Warning: {e}")
        except Exception as e:
            print(f"  ✗ Error in ablation analysis: {e}")
    
    # Statistical Tests
    if not args.skip_statistics:
        print("\n[5/5] Running statistical tests...")
        try:
            nav_analyzer = NavigationAnalyzer(args.data_dir)
            energy_data = nav_analyzer.get_energy_data()
            
            # ANOVA
            anova_results = stats.energy_anova(energy_data)
            print(f"  ANOVA: F={anova_results['F_statistic']:.2f}, "
                  f"p={anova_results['p_value']:.4f}")
            table_gen.generate_anova_table(anova_results)
            
            # Tukey HSD
            tukey_results = stats.tukey_hsd(energy_data)
            table_gen.generate_tukey_table(tukey_results)
            print(f"  Tukey HSD: {len(tukey_results)} pairwise comparisons")
            
            # Effect sizes
            effect_sizes = stats.cohens_d_analysis(energy_data)
            table_gen.generate_effect_size_table(effect_sizes)
            print(f"  Effect sizes: {len(effect_sizes)} comparisons")
            
            # Generate energy comparison tables
            from piec_analysis.data_loader import DataLoader
            loader = DataLoader(args.data_dir)
            energy_by_env = loader.get_energy_by_environment(nav_analyzer.trials_data)
            table_gen.generate_energy_comparison_table(energy_by_env, 'simulation')
            
            print("  ✓ Statistical tests complete")
        except FileNotFoundError as e:
            print(f"  ⚠ Warning: {e}")
        except Exception as e:
            print(f"  ✗ Error in statistical tests: {e}")
    
    # Summary
    print("\n" + "="*70)
    print("RESULTS GENERATION COMPLETE")
    print("="*70)
    print(f"📊 Figures saved to: {os.path.join(args.output_dir, 'figures')}/")
    print(f"📋 Tables saved to: {os.path.join(args.output_dir, 'tables')}/")
    print(f"📈 Statistics saved to: {os.path.join(args.output_dir, 'statistics')}/")
    print("="*70)
    print("\nTo use in thesis/paper:")
    print("  1. Copy figures: cp output/figures/*.pdf thesis/figures/")
    print("  2. Copy tables: cp output/tables/*.tex thesis/tables/")
    print("  3. Include in LaTeX: \\includegraphics{figures/pinn_training_curves.pdf}")
    print("  4. Include tables: \\input{tables/table_energy_simulation.tex}")
    print("="*70)


if __name__ == '__main__':
    main()
