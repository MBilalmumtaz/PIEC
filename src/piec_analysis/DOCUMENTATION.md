# PIEC Analysis Package - Complete Documentation

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Installation](#installation)
4. [Quick Start](#quick-start)
5. [Data Format Specifications](#data-format-specifications)
6. [Generated Outputs](#generated-outputs)
7. [API Reference](#api-reference)
8. [Examples](#examples)
9. [Customization](#customization)
10. [Troubleshooting](#troubleshooting)

## Overview

The PIEC Analysis package is a comprehensive Python tool for generating publication-ready figures, LaTeX tables, and statistical analyses for the PIEC (Physics-Informed Energy-Efficient Controller) research project. It processes experimental data and produces outputs that can be directly inserted into thesis and paper documents.

### Key Capabilities

- **PINN Analysis**: Training curves, accuracy metrics, terrain-specific performance
- **Navigation Performance**: Energy comparison, path characteristics, success rates
- **Localization Analysis**: UKF vs EKF vs odometry comparison
- **Multi-Objective Optimization**: Pareto front visualization, NSGA-II convergence
- **Statistical Tests**: ANOVA, Tukey HSD, Cohen's d, confidence intervals
- **Publication-Quality Output**: Nature journal standards, LaTeX-ready tables

## Features

### Figures Generated (20+)

1. **PINN Analysis**
   - Training curves (loss, learning rate, early stopping)
   - Prediction accuracy (scatter, residuals, Q-Q plot)
   - Terrain-specific accuracy
   - Velocity-based accuracy
   - Inference timing distribution

2. **Navigation Performance**
   - Energy comparison across methods and environments
   - Path characteristics (length, smoothness, clearance)
   - Mission success rates and failure modes
   - Pareto fronts and hypervolume convergence

3. **Localization**
   - Position and orientation RMSE over time
   - Error CDF plots
   - Trajectory overlays with ground truth

### Tables Generated (15+)

1. **PINN Performance**
   - Test set accuracy metrics
   - Terrain-specific accuracy
   - Inference timing statistics

2. **Navigation Performance**
   - Energy consumption (simulation and experimental)
   - Comprehensive benchmarks
   - Method comparisons

3. **Statistical Analysis**
   - ANOVA results
   - Tukey HSD post-hoc tests
   - Cohen's d effect sizes

### Statistical Tests

- One-way ANOVA
- Tukey HSD post-hoc tests
- Cohen's d effect size calculation
- Shapiro-Wilk normality tests
- Paired t-tests
- Bootstrap confidence intervals

## Installation

### Prerequisites

- Python 3.8 or higher
- ROS 2 (optional, for ROS integration)

### Using ROS 2

```bash
cd /path/to/workspace
colcon build --packages-select piec_analysis
source install/setup.bash
```

### Using pip

```bash
cd /path/to/PIEC/src/piec_analysis
pip install -e .  # Development mode
# or
pip install .     # Normal installation
```

### Dependencies

The package automatically installs:
- matplotlib >= 3.5
- seaborn >= 0.11
- pandas >= 1.3
- numpy >= 1.21
- scipy >= 1.7
- scikit-learn >= 1.0

Optional:
- statsmodels >= 0.13 (for advanced statistics)

## Quick Start

### 1. Test with Sample Data

```bash
python3 scripts/generate_all_results.py \
    --data-dir data/sample_data \
    --output-dir test_output \
    --format both
```

### 2. Generate All Results

```bash
python3 scripts/generate_all_results.py \
    --data-dir /path/to/your/data \
    --output-dir output
```

### 3. Use Results in LaTeX

```latex
\includegraphics{figures/pinn_training_curves.pdf}
\input{tables/table_energy_simulation.tex}
```

See [QUICKSTART.md](QUICKSTART.md) for detailed quick start guide.

## Data Format Specifications

All data should be in CSV format with headers. See [data/README.md](data/README.md) for complete specifications.

### Required Files

1. **pinn_training_history.csv**
   - Columns: `epoch`, `train_loss`, `val_loss`
   - Optional: `data_loss`, `physics_loss`, `learning_rate`

2. **pinn_test_predictions.csv**
   - Columns: `actual_energy`, `predicted_energy`
   - Optional: `terrain_type`, `velocity`, `slope`, `roughness`

3. **navigation_trials.csv**
   - Columns: `trial_id`, `method`, `environment`, `energy_consumed`, `success`
   - Optional: `path_length`, `mission_time`, `clearance_min`

4. **localization_timeseries.csv**
   - Columns: `timestamp`, `method`, `estimated_x`, `estimated_y`, `ground_truth_x`, `ground_truth_y`
   - Optional: `estimated_theta`, `ground_truth_theta`

### Optional Files

5. **nsga_evolution.csv** - Multi-objective optimization data
6. **ablation_results.csv** - Ablation study results

### Templates

Data collection templates are available in `data/templates/`:
- `navigation_trial_template.csv`
- `localization_trial_template.csv`
- `pinn_training_template.csv`

## Generated Outputs

### Directory Structure

```
output/
├── figures/           # PDF and PNG figures
│   ├── pinn_training_curves.pdf
│   ├── pinn_accuracy.pdf
│   ├── energy_comparison.pdf
│   └── pareto_fronts.pdf
├── tables/            # LaTeX tables
│   ├── table_pinn_accuracy.tex
│   ├── table_energy_simulation.tex
│   ├── table_anova_energy.tex
│   └── table_tukey_hsd.tex
└── statistics/        # Statistical results (future)
```

### Figure Specifications

- **Format**: PDF (vector) and/or PNG (raster)
- **Resolution**: 300 DPI
- **Size**: 183mm width (Nature double-column)
- **Fonts**: Arial/Helvetica, TrueType embedded
- **Style**: Publication-ready with appropriate grids, legends, labels

### Table Specifications

- **Format**: LaTeX with booktabs package
- **Labels**: Automatic `\label{tab:*}` generation
- **Captions**: Descriptive captions included
- **Formatting**: Mean ± std, significance markers (*, **, ***)

## API Reference

### Core Classes

#### DataLoader

```python
from piec_analysis import DataLoader

loader = DataLoader('data/')
df = loader.load_navigation_trials()
energy_dict = loader.get_energy_by_method(df)
```

#### PINNAnalyzer

```python
from piec_analysis import PINNAnalyzer

analyzer = PINNAnalyzer('data/')
analyzer.generate_training_curves(visualizer)
metrics = analyzer.get_accuracy_metrics()
```

#### NavigationAnalyzer

```python
from piec_analysis import NavigationAnalyzer

analyzer = NavigationAnalyzer('data/')
analyzer.generate_energy_comparison(visualizer)
energy_data = analyzer.get_energy_data()
```

#### StatisticalTests

```python
from piec_analysis import StatisticalTests

stats = StatisticalTests()
anova = stats.energy_anova(energy_dict)
tukey = stats.tukey_hsd(energy_dict)
effect_sizes = stats.cohens_d_analysis(energy_dict)
```

#### Visualizer

```python
from piec_analysis import Visualizer

viz = Visualizer('output/', format='both', style='nature')
viz.plot_training_curves(history_df)
viz.plot_prediction_accuracy(predictions_df)
```

#### TableGenerator

```python
from piec_analysis import TableGenerator

table_gen = TableGenerator('output/')
table_gen.generate_pinn_accuracy_table(metrics)
table_gen.generate_anova_table(anova_results)
```

## Examples

### Example 1: Complete Analysis

See [example_usage.py](example_usage.py) for a complete working example.

```bash
python3 example_usage.py
```

### Example 2: Custom Analysis

```python
from piec_analysis import *

# Load data
loader = DataLoader('data/')
nav_trials = loader.load_navigation_trials()

# Analyze specific method
piec_data = nav_trials[nav_trials['method'] == 'PIEC']
print(f"PIEC average energy: {piec_data['energy_consumed'].mean():.1f} J")

# Compare two methods
stats = StatisticalTests()
piec_energy = piec_data['energy_consumed'].values
astar_energy = nav_trials[nav_trials['method'] == 'ASTAR']['energy_consumed'].values

d = stats.cohens_d(piec_energy, astar_energy)
print(f"Effect size: {d:.3f}")
```

### Example 3: Generate Specific Figures

```python
from piec_analysis import PINNAnalyzer, Visualizer

viz = Visualizer('my_output/', format='pdf')
pinn = PINNAnalyzer('my_data/')

# Only generate training curves
pinn.generate_training_curves(viz)
```

## Customization

### Change Plot Colors

Edit `piec_analysis/config.yaml`:

```yaml
colors:
  piec: '#0C5DA5'
  astar: '#FF2C00'
  # Add more custom colors
```

### Modify Figure Size

```yaml
plotting:
  figure_width: 7.2  # inches
  figure_height: 5.4
  dpi: 300
```

### Add Custom Statistical Tests

Extend the `StatisticalTests` class:

```python
from piec_analysis.statistical_tests import StatisticalTests

class MyStatisticalTests(StatisticalTests):
    def my_custom_test(self, data):
        # Your custom test implementation
        pass
```

## Troubleshooting

### Common Issues

1. **Module not found errors**
   - Solution: `pip install pandas numpy scipy matplotlib seaborn scikit-learn`

2. **File not found errors**
   - Solution: Check data directory path and file names
   - Use `--skip-*` flags to skip missing data

3. **Figure quality issues**
   - Solution: Check DPI in config.yaml
   - Ensure fonts are installed for publication

4. **LaTeX table formatting issues**
   - Solution: Include `\usepackage{booktabs}` in LaTeX preamble
   - Check for special characters that need escaping

### Getting Help

- Check [QUICKSTART.md](QUICKSTART.md) for basic usage
- See [data/README.md](data/README.md) for data formats
- Open GitHub issue for bugs or feature requests
- Run unit tests: `python3 test/test_piec_analysis.py`

## Contributing

To contribute to this package:

1. Fork the repository
2. Create a feature branch
3. Add your changes with tests
4. Submit a pull request

## License

Apache-2.0

## Citation

```bibtex
@misc{piec_analysis,
  title={PIEC Analysis Package},
  author={Muhammad Amjad},
  year={2024},
  publisher={GitHub},
  url={https://github.com/muhammadamjadbit/PIEC}
}
```

## Acknowledgments

This package was developed as part of the PIEC (Physics-Informed Energy-Efficient Controller) research project for autonomous mobile robots.

## Version History

- **v0.1.0** (2024): Initial release
  - Core analysis modules
  - PINN, navigation, localization analysis
  - Statistical tests
  - Publication-quality visualizations
  - LaTeX table generation
  - Sample data and templates
  - Comprehensive documentation
