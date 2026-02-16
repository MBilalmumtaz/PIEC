#!/usr/bin/env python3
"""Generate navigation performance figures only."""

import argparse
import sys


def main():
    """Generate navigation-specific figures."""
    parser = argparse.ArgumentParser(description='Generate navigation analysis figures')
    parser.add_argument('--data-dir', type=str, required=True,
                       help='Directory containing navigation data')
    parser.add_argument('--output-dir', type=str, default='output',
                       help='Output directory for figures')
    parser.add_argument('--format', choices=['pdf', 'png', 'both'], default='both',
                       help='Output format')
    args = parser.parse_args()
    
    try:
        from piec_analysis import NavigationAnalyzer, Visualizer
    except ImportError as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    visualizer = Visualizer(args.output_dir, format=args.format)
    
    print("Generating navigation analysis...")
    nav_analyzer = NavigationAnalyzer(args.data_dir)
    
    nav_analyzer.generate_energy_comparison(visualizer)
    nav_analyzer.generate_path_characteristics(visualizer)
    nav_analyzer.generate_success_analysis(visualizer)
    nav_analyzer.generate_pareto_fronts(visualizer)
    
    print("✓ Navigation analysis complete")


if __name__ == '__main__':
    main()
