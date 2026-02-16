#!/usr/bin/env python3
"""
Aggregate individual trial CSV files into master datasets.
Combines data from multiple trials for analysis with piec_analysis package.
"""

import argparse
import os
import pandas as pd
import glob
from pathlib import Path


def aggregate_trajectory_files(data_dir, output_file):
    """Combine all trajectory CSV files into master timeseries."""
    pattern = os.path.join(data_dir, '*_trajectory.csv')
    files = glob.glob(pattern)
    
    if not files:
        print(f"  No trajectory files found in {data_dir}")
        return
    
    print(f"  Found {len(files)} trajectory files")
    
    # Read and concatenate all files
    dfs = []
    for file in sorted(files):
        try:
            df = pd.read_csv(file)
            dfs.append(df)
        except Exception as e:
            print(f"    Warning: Failed to read {file}: {e}")
    
    if not dfs:
        print("  No valid trajectory data to aggregate")
        return
    
    # Combine all dataframes
    combined = pd.concat(dfs, ignore_index=True)
    
    # Sort by trial_id and timestamp
    combined = combined.sort_values(['trial_id', 'timestamp'])
    
    # Save aggregated data
    combined.to_csv(output_file, index=False)
    print(f"  Saved aggregated trajectory data: {output_file}")
    print(f"    Total records: {len(combined)}")
    print(f"    Trials: {combined['trial_id'].nunique()}")


def aggregate_localization_files(data_dir, output_file):
    """Combine all localization CSV files into master timeseries."""
    pattern = os.path.join(data_dir, '*_localization.csv')
    files = glob.glob(pattern)
    
    if not files:
        print(f"  No localization files found in {data_dir}")
        return
    
    print(f"  Found {len(files)} localization files")
    
    # Read and concatenate all files
    dfs = []
    for file in sorted(files):
        try:
            df = pd.read_csv(file)
            dfs.append(df)
        except Exception as e:
            print(f"    Warning: Failed to read {file}: {e}")
    
    if not dfs:
        print("  No valid localization data to aggregate")
        return
    
    # Combine all dataframes
    combined = pd.concat(dfs, ignore_index=True)
    
    # Sort by trial_id and timestamp
    combined = combined.sort_values(['trial_id', 'timestamp'])
    
    # Save aggregated data
    combined.to_csv(output_file, index=False)
    print(f"  Saved aggregated localization data: {output_file}")
    print(f"    Total records: {len(combined)}")
    print(f"    Trials: {combined['trial_id'].nunique()}")


def check_navigation_trials(data_dir):
    """Check and report on navigation_trials.csv file."""
    trials_file = os.path.join(data_dir, 'navigation_trials.csv')
    
    if not os.path.exists(trials_file):
        print(f"  Warning: No navigation_trials.csv found in {data_dir}")
        return
    
    try:
        df = pd.read_csv(trials_file)
        print(f"  Found navigation_trials.csv:")
        print(f"    Total trials: {len(df)}")
        print(f"    Methods: {df['method'].unique().tolist()}")
        print(f"    Environments: {df['environment'].unique().tolist()}")
        print(f"    Success rate: {df['success'].sum() / len(df) * 100:.1f}%")
    except Exception as e:
        print(f"    Warning: Failed to read navigation_trials.csv: {e}")


def generate_summary_statistics(data_dir, output_dir):
    """Generate summary statistics for the dataset."""
    trials_file = os.path.join(data_dir, 'navigation_trials.csv')
    
    if not os.path.exists(trials_file):
        return
    
    try:
        df = pd.read_csv(trials_file)
        
        # Group by method and environment
        summary = df.groupby(['method', 'environment']).agg({
            'energy_consumed': ['mean', 'std', 'min', 'max'],
            'path_length': ['mean', 'std'],
            'mission_time': ['mean', 'std'],
            'success': ['sum', 'count']
        }).round(3)
        
        # Save summary
        summary_file = os.path.join(output_dir, 'summary_statistics.csv')
        summary.to_csv(summary_file)
        print(f"\n  Generated summary statistics: {summary_file}")
        
    except Exception as e:
        print(f"  Warning: Failed to generate summary statistics: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Aggregate experimental data from multiple trials'
    )
    parser.add_argument(
        '--data-dir',
        type=str,
        required=True,
        help='Directory containing trial CSV files'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Output directory (defaults to data-dir)'
    )
    args = parser.parse_args()
    
    # Set output directory
    data_dir = os.path.expanduser(args.data_dir)
    output_dir = os.path.expanduser(args.output_dir) if args.output_dir else data_dir
    
    if not os.path.exists(data_dir):
        print(f"Error: Data directory not found: {data_dir}")
        return
    
    print("=" * 70)
    print("PIEC Data Aggregation")
    print("=" * 70)
    print(f"Data directory: {data_dir}")
    print(f"Output directory: {output_dir}")
    print("=" * 70)
    print()
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Check navigation trials summary
    print("[1/4] Checking navigation trials...")
    check_navigation_trials(data_dir)
    print()
    
    # Aggregate trajectory data
    print("[2/4] Aggregating trajectory data...")
    trajectory_output = os.path.join(output_dir, 'trajectory_timeseries.csv')
    aggregate_trajectory_files(data_dir, trajectory_output)
    print()
    
    # Aggregate localization data
    print("[3/4] Aggregating localization data...")
    localization_output = os.path.join(output_dir, 'localization_timeseries.csv')
    aggregate_localization_files(data_dir, localization_output)
    print()
    
    # Generate summary statistics
    print("[4/4] Generating summary statistics...")
    generate_summary_statistics(data_dir, output_dir)
    print()
    
    # Final summary
    print("=" * 70)
    print("AGGREGATION COMPLETE")
    print("=" * 70)
    print("\nGenerated files:")
    print(f"  - navigation_trials.csv (trial summaries)")
    print(f"  - trajectory_timeseries.csv (robot state timeseries)")
    print(f"  - localization_timeseries.csv (UKF vs ground truth)")
    print(f"  - summary_statistics.csv (statistical summary)")
    print()
    print("These files are ready for analysis with piec_analysis:")
    print(f"  python3 -m piec_analysis.scripts.generate_all_results --data-dir {output_dir}")
    print("=" * 70)


if __name__ == '__main__':
    main()
