# Quick Start Guide

This guide will help you get started with the PIEC Analysis package quickly.

## Prerequisites

- Python 3.8 or higher
- Required Python packages (will be installed automatically)

## Installation

### Option 1: Using ROS 2 (Recommended)

```bash
# Navigate to your ROS 2 workspace
cd /path/to/your/workspace

# Build the package
colcon build --packages-select piec_analysis

# Source the workspace
source install/setup.bash
```

### Option 2: Using pip (Development)

```bash
cd /path/to/PIEC/src/piec_analysis

# Install in development mode
pip install -e .

# Or install normally
pip install .
```

### Install Dependencies

If dependencies are not automatically installed:

```bash
pip install pandas numpy scipy matplotlib seaborn scikit-learn
```

## Quick Test

Test the package with sample data:

```bash
# Using ROS 2
ros2 run piec_analysis generate_all_results \
    --data-dir install/piec_analysis/share/piec_analysis/data/sample_data \
    --output-dir test_output

# Or directly with Python
python3 -m piec_analysis.scripts.generate_all_results \
    --data-dir src/piec_analysis/data/sample_data \
    --output-dir test_output
```

## Basic Usage

### 1. Prepare Your Data

Organize your experimental data in CSV format according to the specifications in `data/README.md`.

Required files:
- `pinn_training_history.csv` - PINN training logs
- `pinn_test_predictions.csv` - PINN test results
- `navigation_trials.csv` - Navigation experiment results
- `localization_timeseries.csv` - Localization time series data

Optional files:
- `nsga_evolution.csv` - Multi-objective optimization data
- `ablation_results.csv` - Ablation study results

### 2. Generate All Results

```bash
python3 scripts/generate_all_results.py \
    --data-dir /path/to/your/data \
    --output-dir output \
    --format both
```

This will generate:
- PDF and PNG figures in `output/figures/`
- LaTeX tables in `output/tables/`
- Statistical results in `output/statistics/`

### 3. Generate Specific Analyses

```bash
# Only PINN analysis
python3 scripts/generate_pinn_figures.py --data-dir data/

# Only navigation analysis
python3 scripts/generate_navigation_figures.py --data-dir data/

# Only comparison tables
python3 scripts/generate_comparison_tables.py --data-dir data/
```

### 4. Use Results in Your Thesis

```bash
# Copy figures
cp output/figures/*.pdf /path/to/thesis/figures/

# Copy tables
cp output/tables/*.tex /path/to/thesis/tables/
```

In your LaTeX document:

```latex
% Include a figure
\begin{figure}[h]
    \centering
    \includegraphics[width=0.9\textwidth]{figures/pinn_training_curves.pdf}
    \caption{PINN training curves showing convergence behavior.}
    \label{fig:pinn_training}
\end{figure}

% Include a table
\input{tables/table_energy_simulation.tex}
```

## Command-Line Options

### generate_all_results.py

- `--data-dir DIR` (required): Directory containing experimental data
- `--output-dir DIR`: Output directory (default: 'output')
- `--format {pdf,png,both}`: Figure format (default: 'both')
- `--skip-pinn`: Skip PINN analysis
- `--skip-navigation`: Skip navigation analysis
- `--skip-localization`: Skip localization analysis
- `--skip-ablation`: Skip ablation study
- `--skip-statistics`: Skip statistical tests

### Example with Options

```bash
# Generate only navigation and statistical analysis, PDF only
python3 scripts/generate_all_results.py \
    --data-dir data/ \
    --output-dir results/ \
    --format pdf \
    --skip-pinn \
    --skip-localization \
    --skip-ablation
```

## Python API Usage

You can also use the package programmatically:

```python
from piec_analysis import (
    PINNAnalyzer,
    NavigationAnalyzer,
    Visualizer,
    TableGenerator,
    StatisticalTests
)

# Initialize components
visualizer = Visualizer('output', format='pdf')
table_gen = TableGenerator('output')

# Analyze PINN results
pinn = PINNAnalyzer('data/')
pinn.generate_training_curves(visualizer)
pinn.generate_accuracy_plots(visualizer)

metrics = pinn.get_accuracy_metrics()
table_gen.generate_pinn_accuracy_table(metrics)

# Analyze navigation results
nav = NavigationAnalyzer('data/')
nav.generate_energy_comparison(visualizer)

energy_data = nav.get_energy_data()

# Statistical tests
stats = StatisticalTests()
anova_results = stats.energy_anova(energy_data)
tukey_results = stats.tukey_hsd(energy_data)

table_gen.generate_anova_table(anova_results)
table_gen.generate_tukey_table(tukey_results)
```

## Customization

### Change Plot Style

Edit `piec_analysis/config.yaml`:

```yaml
plotting:
  style: nature  # or 'seaborn', 'ggplot', etc.
  format: both
  dpi: 300
  figure_width: 7.2
  figure_height: 5.4

colors:
  piec: '#0C5DA5'  # Change colors here
  astar: '#FF2C00'
  # ...
```

### Add New Analysis

1. Create a new analyzer class in `piec_analysis/`
2. Add visualization methods
3. Create a script in `scripts/`
4. Update `generate_all_results.py` to include your analysis

## Troubleshooting

### "No module named 'pandas'"

Install dependencies:
```bash
pip install pandas numpy scipy matplotlib seaborn scikit-learn
```

### "FileNotFoundError: File not found"

- Check that your data files are in the correct directory
- Verify file names match the expected format (see `data/README.md`)
- Use `--skip-*` flags to skip analyses with missing data

### "ImportError: Cannot import name 'X'"

Rebuild and reinstall the package:
```bash
colcon build --packages-select piec_analysis --force
source install/setup.bash
```

Or for pip installation:
```bash
pip install -e . --force-reinstall
```

### Figures look different than expected

- Check DPI settings in config.yaml
- Verify matplotlib backend: `matplotlib.use('Agg')` for non-interactive
- Check font availability for publication-quality output

## Getting Help

- Check the main README.md for detailed documentation
- See data/README.md for data format specifications
- Open an issue on GitHub for bugs or feature requests

## Next Steps

1. Collect your experimental data
2. Format it according to specifications
3. Run the analysis
4. Review generated figures and tables
5. Include in your thesis/paper
6. Customize as needed

Happy analyzing!
