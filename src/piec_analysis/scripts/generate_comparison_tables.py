#!/usr/bin/env python3
"""Generate comparison tables only."""

import argparse
import sys


def main():
    """Generate comparison tables."""
    parser = argparse.ArgumentParser(description='Generate comparison tables')
    parser.add_argument('--data-dir', type=str, required=True,
                       help='Directory containing data')
    parser.add_argument('--output-dir', type=str, default='output',
                       help='Output directory for tables')
    args = parser.parse_args()
    
    try:
        from piec_analysis import (
            NavigationAnalyzer,
            StatisticalTests,
            TableGenerator,
            DataLoader
        )
    except ImportError as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    table_gen = TableGenerator(args.output_dir)
    stats = StatisticalTests()
    
    print("Generating comparison tables...")
    
    # Load navigation data
    nav_analyzer = NavigationAnalyzer(args.data_dir)
    energy_data = nav_analyzer.get_energy_data()
    
    # Statistical tests
    anova_results = stats.energy_anova(energy_data)
    tukey_results = stats.tukey_hsd(energy_data)
    effect_sizes = stats.cohens_d_analysis(energy_data)
    
    # Generate tables
    table_gen.generate_anova_table(anova_results)
    table_gen.generate_tukey_table(tukey_results)
    table_gen.generate_effect_size_table(effect_sizes)
    
    # Energy comparison table
    loader = DataLoader(args.data_dir)
    energy_by_env = loader.get_energy_by_environment(nav_analyzer.trials_data)
    table_gen.generate_energy_comparison_table(energy_by_env, 'simulation')
    
    print("✓ Comparison tables complete")


if __name__ == '__main__':
    main()
