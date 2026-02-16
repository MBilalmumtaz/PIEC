#!/usr/bin/env python3
"""Generate ablation study analysis."""

import argparse
import sys


def main():
    """Generate ablation study figures."""
    parser = argparse.ArgumentParser(description='Generate ablation study analysis')
    parser.add_argument('--data-dir', type=str, required=True,
                       help='Directory containing ablation data')
    parser.add_argument('--output-dir', type=str, default='output',
                       help='Output directory for figures')
    parser.add_argument('--format', choices=['pdf', 'png', 'both'], default='both',
                       help='Output format')
    args = parser.parse_args()
    
    try:
        from piec_analysis import AblationAnalyzer, Visualizer
    except ImportError as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    visualizer = Visualizer(args.output_dir, format=args.format)
    
    print("Generating ablation study analysis...")
    ablation_analyzer = AblationAnalyzer(args.data_dir)
    
    ablation_analyzer.generate_component_ablation(visualizer)
    ablation_analyzer.generate_parameter_sensitivity(visualizer)
    
    print("✓ Ablation study analysis complete")


if __name__ == '__main__':
    main()
