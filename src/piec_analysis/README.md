# PIEC Analysis Package

A comprehensive Python package for generating all graphs, tables, and statistical analyses mentioned in the PIEC thesis and paper.

## Overview

The `piec_analysis` package processes experimental data and produces publication-ready figures and LaTeX tables that can be directly inserted into thesis and paper documents.

## Installation

```bash
# Navigate to the workspace
cd /path/to/PIEC/workspace

# Build the package
colcon build --packages-select piec_analysis

# Source the workspace
source install/setup.bash
```

## Package Structure

```
piec_analysis/
├── piec_analysis/          # Main package code
│   ├── data_loader.py          # Load experimental data from CSV
│   ├── pinn_analysis.py        # PINN training curves, accuracy metrics
│   ├── navigation_analysis.py  # Energy, path length, success rate
│   ├── localization_analysis.py # UKF vs EKF vs odometry comparison
│   ├── ablation_analysis.py    # Component removal studies
│   ├── statistical_tests.py    # ANOVA, Tukey HSD, t-tests
│   ├── visualization.py        # Plotting utilities
│   ├── table_generator.py      # LaTeX table generation
│   └── config.yaml             # Configuration
├── scripts/                # Executable scripts
│   ├── generate_all_results.py
│   ├── generate_pinn_figures.py
│   ├── generate_navigation_figures.py
│   ├── generate_comparison_tables.py
│   └── generate_ablation_study.py
├── data/                   # Data storage
│   ├── sample_data/            # Example data files
│   └── templates/              # Data collection templates
└── output/                 # Generated outputs
    ├── figures/                # PNG/PDF figures
    ├── tables/                 # LaTeX tables
    └── statistics/             # Statistical test results
```

## Usage

### Generate All Results

```bash
ros2 run piec_analysis generate_all_results \
    --data-dir /path/to/data \
    --output-dir output \
    --format both
```

### Generate Specific Analyses

```bash
# PINN analysis only
ros2 run piec_analysis generate_pinn_figures --data-dir data/

# Navigation analysis only
ros2 run piec_analysis generate_navigation_figures --data-dir data/

# Comparison tables only
ros2 run piec_analysis generate_comparison_tables --data-dir data/

# Ablation study only
ros2 run piec_analysis generate_ablation_study --data-dir data/
```

### Command-line Options

- `--data-dir`: Directory containing experimental data files (required)
- `--output-dir`: Directory for generated figures and tables (default: 'output')
- `--format`: Output format for figures: 'pdf', 'png', or 'both' (default: 'both')
- `--skip-pinn`: Skip PINN analysis
- `--skip-navigation`: Skip navigation analysis
- `--skip-localization`: Skip localization analysis
- `--skip-ablation`: Skip ablation study
- `--skip-statistics`: Skip statistical tests

## Data Format Specifications

### 1. PINN Training History
**File**: `pinn_training_history.csv`

```csv
epoch,train_loss,val_loss,data_loss,physics_loss,learning_rate
0,2.345,2.678,1.890,0.455,0.001
1,1.234,1.567,0.987,0.247,0.001
...
```

### 2. PINN Test Predictions
**File**: `pinn_test_predictions.csv`

```csv
actual_energy,predicted_energy,terrain_type,velocity,slope,roughness
12.34,12.56,concrete,0.5,0.0,0.1
...
```

### 3. Navigation Trials
**File**: `navigation_trials.csv`

```csv
trial_id,method,environment,energy_consumed,path_length,mission_time,success,failure_mode,clearance_min
1,PIEC,indoor_corridor,142.3,40.2,87.3,True,None,0.68
2,ASTAR,indoor_corridor,189.4,38.7,68.4,True,None,0.51
...
```

### 4. Localization Time Series
**File**: `localization_timeseries.csv`

```csv
timestamp,method,estimated_x,estimated_y,estimated_theta,ground_truth_x,ground_truth_y,ground_truth_theta
0.0,UKF,0.000,0.000,0.000,0.000,0.000,0.000
...
```

### 5. NSGA-II Evolution
**File**: `nsga_evolution.csv`

```csv
generation,individual_id,front_rank,crowding_distance,f1_length,f2_curvature,f3_obstacle,f4_localization,f5_deviation,f6_energy,f7_stability
0,0,1,0.567,12.3,0.45,2.1,0.012,0.15,145.6,0.89
...
```

## Generated Outputs

### Figures (20+)
- `pinn_training_curves.pdf/png` - Training and validation loss curves
- `pinn_accuracy.pdf` - Prediction accuracy analysis
- `energy_comparison.pdf` - Energy consumption comparison
- `pareto_fronts.pdf` - Pareto front visualizations
- And more...

### Tables (15+)
- `table_pinn_accuracy.tex` - PINN accuracy metrics
- `table_energy_simulation.tex` - Energy comparison (simulation)
- `table_anova_energy.tex` - ANOVA results
- `table_tukey_hsd.tex` - Tukey HSD post-hoc tests
- `table_effect_size.tex` - Cohen's d effect sizes
- And more...

## Using Results in Thesis/Paper

### 1. Copy Generated Files

```bash
# Copy figures to thesis
cp output/figures/*.pdf /path/to/thesis/figures/

# Copy tables to thesis
cp output/tables/*.tex /path/to/thesis/tables/
```

### 2. Include in LaTeX

```latex
% Include a figure
\begin{figure}[h]
    \centering
    \includegraphics[width=\textwidth]{figures/pinn_training_curves.pdf}
    \caption{PINN training curves showing convergence behavior.}
    \label{fig:pinn_training}
\end{figure}

% Include a table
\input{tables/table_energy_simulation.tex}
```

## Testing with Sample Data

The package includes sample data to test functionality:

```bash
# Test with sample data
ros2 run piec_analysis generate_all_results \
    --data-dir install/piec_analysis/share/piec_analysis/data/sample_data \
    --output-dir test_output
```

## Dependencies

- Python >= 3.8
- matplotlib >= 3.5
- seaborn >= 0.11
- pandas >= 1.3
- numpy >= 1.21
- scipy >= 1.7
- scikit-learn >= 1.0
- statsmodels >= 0.13 (optional, for advanced statistics)

## Development

### Adding New Analyses

1. Create a new analyzer class in `piec_analysis/`
2. Add methods for data loading, analysis, and visualization
3. Create corresponding script in `scripts/`
4. Update main script to include new analysis

### Customizing Plots

Edit `piec_analysis/config.yaml` to customize:
- Plot colors
- Figure dimensions
- Statistical test parameters
- Data file names

## Troubleshooting

### Missing Data Files

If you get file not found errors:
1. Check that data files are in the correct directory
2. Verify file names match the expected format
3. Use `--skip-*` flags to skip analyses with missing data

### Import Errors

If modules can't be imported:
```bash
# Rebuild and source
colcon build --packages-select piec_analysis
source install/setup.bash
```

### Figure Quality

For publication:
- Use `--format pdf` for vector graphics
- Check DPI settings in config.yaml (default: 300)
- Verify font embedding for journals

## Citation

If you use this package in your research, please cite:

```bibtex
@misc{piec_analysis,
  title={PIEC Analysis Package},
  author={Muhammad Amjad},
  year={2024},
  publisher={GitHub},
  url={https://github.com/muhammadamjadbit/PIEC}
}
```

## License

Apache-2.0

## Contact

For questions or issues, please open an issue on GitHub or contact the maintainer.
