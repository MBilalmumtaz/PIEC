# PIEC Automated Data Recording System

This directory contains tools for automated experimental data recording during PIEC (Physics-Informed Energy-Conscious) navigation experiments.

## Overview

The automated data recording system captures all necessary experimental data while running PIEC experiments in simulation and real-world environments. The data is output in the format required by the `piec_analysis` package for generating thesis/paper results.

## System Components

### 1. ROS 2 Nodes

- **`experiment_recorder`** (`piec_bringup/experiment_recorder.py`)
  - Records robot state, localization, and mission metrics during experiments
  - Subscribes to: `/odom`, `/scan`, `/ukf/odom`, `/ground_truth/pose`, `/piec/path`, `/piec/goal_reached`
  - Outputs: Trial-specific CSV files + navigation_trials.csv summary

### 2. PINN Training Scripts

- **`train_pinn.py`** - Enhanced PINN training with data export
  - Saves `pinn_training_history.csv` (loss curves per epoch)
  - Saves `pinn_test_predictions.csv` (test set predictions with metadata)
  
- **`benchmark_pinn_inference.py`** - PINN inference timing benchmarks
  - Tests multiple batch sizes
  - Saves `pinn_inference_timing.csv`

### 3. Automation Scripts

- **`record_simulation_experiment.sh`** - Automated multi-trial simulation recording
- **`record_real_experiment.sh`** - Interactive real-world trial recording
- **`aggregate_experiment_data.py`** - Combines trial files into master datasets

## Quick Start

### Step 1: Train PINN Model

```bash
cd /home/runner/work/PIEC/PIEC/pinn_training

# Train PINN (saves training history)
python3 train_pinn.py --csv pinn_dataset.csv --outdir models_out --data-dir ../data

# Benchmark inference timing
python3 benchmark_pinn_inference.py --model models_out/pinn_energy.pt --output ../data/pinn_inference_timing.csv
```

### Step 2: Run Simulation Experiments

```bash
cd /home/runner/work/PIEC/PIEC

# Run 50 trials with PIEC method
./scripts/record_simulation_experiment.sh PIEC corridor 50

# Run 50 trials with A* for comparison
./scripts/record_simulation_experiment.sh ASTAR corridor 50

# Aggregate data
python3 scripts/aggregate_experiment_data.py --data-dir ~/piec_data/simulation
```

### Step 3: Run Real-World Experiments

```bash
cd /home/runner/work/PIEC/PIEC

# Interactive trial recording
./scripts/record_real_experiment.sh PIEC corridor

# Follow prompts to start each trial
# Press Ctrl+C after each trial
# Type 'done' when finished

# Aggregate data
python3 scripts/aggregate_experiment_data.py --data-dir ~/piec_data/real_world
```

### Step 4: Generate Analysis Results

```bash
# Run piec_analysis to generate all figures and tables
ros2 run piec_analysis generate_all_results --data-dir ~/piec_data/simulation

# Results will be in output/figures/ and output/tables/
```

### Step 5: Use in Thesis

```bash
# Copy figures and tables to thesis
cp output/figures/*.pdf thesis/figures/
cp output/tables/*.tex thesis/tables/
```

## Data Format Specifications

### navigation_trials.csv
Trial summary with one row per completed mission:
- `trial_id`, `method`, `environment`
- `energy_consumed`, `path_length`, `mission_time`
- `success`, `failure_mode`
- `clearance_min`, `clearance_avg`
- `num_replans`, `num_near_misses`
- `timestamp`

### trajectory_timeseries.csv
High-frequency robot state (20Hz):
- `trial_id`, `timestamp`
- `robot_x`, `robot_y`, `robot_theta`
- `robot_v`, `robot_omega`
- `energy_cumulative`, `clearance`, `obstacle_density`
- `method`, `environment`

### localization_timeseries.csv
UKF estimates vs ground truth:
- `trial_id`, `timestamp`, `method`
- `estimated_x`, `estimated_y`, `estimated_theta`
- `ground_truth_x`, `ground_truth_y`, `ground_truth_theta`
- `covariance_xx`, `covariance_yy`, `covariance_tt`

### pinn_training_history.csv
Training metrics per epoch:
- `epoch`, `train_loss`, `val_loss`, `learning_rate`

### pinn_test_predictions.csv
Test set predictions with metadata:
- `actual_energy`, `predicted_energy`
- `actual_stability`, `predicted_stability`
- `x`, `y`, `yaw`, `velocity`, `slope`, `roughness`
- `terrain_type`

### pinn_inference_timing.csv
Inference timing benchmarks:
- `batch_size`, `avg_time_ms`, `per_sample_us`
- `throughput_samples_per_sec`, `num_iterations`, `device`

## Launch File Parameters

Both launch files (`piec_complete_params.launch.py` and `piec_real_robot.launch.py`) support:

```bash
ros2 launch piec_bringup <launch_file> \
    enable_recording:=true \
    method:=PIEC \
    environment:=corridor \
    trial_name:=sim_PIEC_001 \
    output_dir:=~/piec_data
```

Parameters:
- `enable_recording` - Enable experiment recorder node (default: false)
- `method` - Navigation method: PIEC, ASTAR, RRT, DWA (default: PIEC)
- `environment` - Environment type: corridor, office, outdoor (default: corridor)
- `trial_name` - Unique trial identifier (default: trial_001)
- `output_dir` - Data output directory (default: ~/piec_data)

## Directory Structure

```
PIEC/
├── data/                           # Data output directory
│   ├── examples/                   # Example CSV files
│   │   ├── navigation_trials_example.csv
│   │   ├── trajectory_timeseries_example.csv
│   │   ├── localization_timeseries_example.csv
│   │   ├── pinn_training_history_example.csv
│   │   ├── pinn_test_predictions_example.csv
│   │   └── pinn_inference_timing_example.csv
│   ├── pinn_training_history.csv  # Generated by train_pinn.py
│   ├── pinn_test_predictions.csv  # Generated by train_pinn.py
│   ├── pinn_inference_timing.csv  # Generated by benchmark_pinn_inference.py
│   └── navigation_trials.csv      # Appended by experiment_recorder
│
├── scripts/                        # Automation scripts
│   ├── record_simulation_experiment.sh
│   ├── record_real_experiment.sh
│   └── aggregate_experiment_data.py
│
├── pinn_training/                  # PINN training scripts
│   ├── train_pinn.py              # Enhanced with CSV export
│   └── benchmark_pinn_inference.py
│
└── src/piec_bringup/
    └── piec_bringup/
        └── experiment_recorder.py  # Data recording node
```

## Testing

### Test Single Trial Recording

```bash
# Simulation
ros2 launch piec_bringup piec_complete_params.launch.py \
    enable_recording:=true \
    trial_name:=test_trial_001

# Real robot
ros2 launch piec_bringup piec_real_robot.launch.py \
    enable_recording:=true \
    trial_name:=real_test_001
```

### Test Data Aggregation

```bash
# After recording some trials
python3 scripts/aggregate_experiment_data.py --data-dir ~/piec_data/simulation
```

### Test PIEC Analysis

```bash
# Generate all figures and tables
ros2 run piec_analysis generate_all_results --data-dir ~/piec_data/simulation
```

## Troubleshooting

### Issue: No data recorded
- Check that `enable_recording:=true` is set in launch command
- Verify experiment_recorder node is running: `ros2 node list`
- Check topics are publishing: `ros2 topic list`

### Issue: Missing localization data
- Ensure `/ukf/odom` and `/ground_truth/pose` topics exist
- For real robot, ground truth may not be available (OptiTrack required)

### Issue: Data files not created
- Check output directory permissions
- Verify disk space available
- Check ROS 2 logs: `ros2 log`

### Issue: Analysis fails to load data
- Verify CSV format matches examples in `data/examples/`
- Check for missing required columns
- Run aggregation script to combine trial files

## Advanced Usage

### Custom Trial Names

```bash
# Use timestamp in trial name
TRIAL="trial_$(date +%Y%m%d_%H%M%S)"
ros2 launch piec_bringup piec_complete_params.launch.py \
    enable_recording:=true \
    trial_name:=$TRIAL
```

### Multiple Methods Comparison

```bash
# Automated comparison of 4 methods
for METHOD in PIEC ASTAR RRT DWA; do
    ./scripts/record_simulation_experiment.sh $METHOD corridor 25
done

# Aggregate all data
python3 scripts/aggregate_experiment_data.py --data-dir ~/piec_data/simulation
```

### Environment Variations

```bash
# Test multiple environments
for ENV in corridor office outdoor warehouse; do
    ./scripts/record_simulation_experiment.sh PIEC $ENV 10
done
```

## Citation

If you use this data recording system in your research, please cite:

```bibtex
@phdthesis{amjad2024piec,
  title={Physics-Informed Energy-Conscious Navigation for Autonomous Robots},
  author={Amjad, Muhammad},
  year={2024},
  school={Your University}
}
```

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review example CSV files in `data/examples/`
3. Check ROS 2 logs for error messages
4. Open an issue on GitHub

## License

Apache 2.0 - See LICENSE file for details
