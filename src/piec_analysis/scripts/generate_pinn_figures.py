#!/usr/bin/env python3
"""Generate PINN analysis figures only."""

import argparse
import sys


def main():
    """Generate PINN-specific figures."""
    parser = argparse.ArgumentParser(description='Generate PINN analysis figures')
    parser.add_argument('--data-dir', type=str, required=True,
                       help='Directory containing PINN data')
    parser.add_argument('--output-dir', type=str, default='output',
                       help='Output directory for figures')
    parser.add_argument('--format', choices=['pdf', 'png', 'both'], default='both',
                       help='Output format')
    args = parser.parse_args()
    
    try:
        from piec_analysis import PINNAnalyzer, Visualizer, TableGenerator
    except ImportError as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    visualizer = Visualizer(args.output_dir, format=args.format)
    table_gen = TableGenerator(args.output_dir)
    
    print("Generating PINN analysis...")
    pinn_analyzer = PINNAnalyzer(args.data_dir)
    
    pinn_analyzer.generate_training_curves(visualizer)
    pinn_analyzer.generate_accuracy_plots(visualizer)
    pinn_analyzer.generate_terrain_analysis(visualizer)
    
    metrics = pinn_analyzer.get_accuracy_metrics()
    table_gen.generate_pinn_accuracy_table(metrics)
    
    print("✓ PINN analysis complete")


if __name__ == '__main__':
    main()
