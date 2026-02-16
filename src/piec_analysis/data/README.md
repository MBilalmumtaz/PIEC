# Data Format Specifications

This document describes the expected format for all data files used by the PIEC analysis package.

## Overview

All data files should be in CSV format with headers. Missing optional columns will cause those specific analyses to be skipped.

## Required Data Files

### 1. PINN Training History

**Filename**: `pinn_training_history.csv`

**Purpose**: Track PINN training progress over epochs

**Required Columns**:
- `epoch` (int): Training epoch number (0, 1, 2, ...)
- `train_loss` (float): Training loss value
- `val_loss` (float): Validation loss value

**Optional Columns**:
- `data_loss` (float): Data fitting loss component
- `physics_loss` (float): Physics constraint loss component
- `learning_rate` (float): Learning rate at this epoch

**Example**:
```csv
epoch,train_loss,val_loss,data_loss,physics_loss,learning_rate
0,2.345,2.678,1.890,0.455,0.001
1,1.234,1.567,0.987,0.247,0.001
2,0.876,1.234,0.654,0.222,0.001
```

### 2. PINN Test Predictions

**Filename**: `pinn_test_predictions.csv`

**Purpose**: Evaluate PINN prediction accuracy

**Required Columns**:
- `actual_energy` (float): Ground truth energy consumption (J)
- `predicted_energy` (float): PINN predicted energy (J)

**Optional Columns**:
- `terrain_type` (string): Surface type (concrete, carpet, grass, etc.)
- `velocity` (float): Robot velocity (m/s)
- `slope` (float): Terrain slope (degrees)
- `roughness` (float): Surface roughness coefficient
- `actual_stability` (float): Ground truth stability metric
- `predicted_stability` (float): PINN predicted stability

**Example**:
```csv
actual_energy,predicted_energy,terrain_type,velocity,slope,roughness
12.34,12.56,concrete,0.5,0.0,0.1
15.67,15.23,carpet,0.6,0.0,0.3
18.90,19.12,grass,0.4,5.0,0.5
```

### 3. Navigation Trials

**Filename**: `navigation_trials.csv`

**Purpose**: Compare navigation methods across experiments

**Required Columns**:
- `trial_id` (int): Unique trial identifier
- `method` (string): Navigation method (PIEC, ASTAR, RRT, DWA, NSGA)
- `environment` (string): Test environment name
- `energy_consumed` (float): Total energy used (J)
- `success` (bool): Whether mission succeeded (True/False)

**Optional Columns**:
- `path_length` (float): Total path length (m)
- `mission_time` (float): Time to complete mission (s)
- `failure_mode` (string): Type of failure if unsuccessful (collision, timeout, etc.)
- `clearance_min` (float): Minimum obstacle clearance (m)
- `planning_time` (float): Planning computation time (ms)

**Example**:
```csv
trial_id,method,environment,energy_consumed,path_length,mission_time,success,failure_mode,clearance_min
1,PIEC,indoor_corridor,142.3,40.2,87.3,True,None,0.68
2,ASTAR,indoor_corridor,189.4,38.7,68.4,True,None,0.51
3,DWA,indoor_corridor,201.5,42.1,72.1,False,collision,0.15
```

### 4. Localization Time Series

**Filename**: `localization_timeseries.csv`

**Purpose**: Compare localization methods over time

**Required Columns**:
- `timestamp` (float): Time in seconds
- `method` (string): Localization method (UKF, EKF, Odometry-only)
- `estimated_x` (float): Estimated x position (m)
- `estimated_y` (float): Estimated y position (m)
- `ground_truth_x` (float): Ground truth x position (m)
- `ground_truth_y` (float): Ground truth y position (m)

**Optional Columns**:
- `estimated_theta` (float): Estimated orientation (rad)
- `ground_truth_theta` (float): Ground truth orientation (rad)
- `covariance_xx` (float): Position covariance (m²)
- `covariance_yy` (float): Position covariance (m²)

**Example**:
```csv
timestamp,method,estimated_x,estimated_y,estimated_theta,ground_truth_x,ground_truth_y,ground_truth_theta
0.0,UKF,0.000,0.000,0.000,0.000,0.000,0.000
0.05,UKF,0.025,0.001,0.002,0.024,0.000,0.001
0.10,UKF,0.048,0.003,0.003,0.050,0.002,0.002
```

### 5. NSGA-II Evolution

**Filename**: `nsga_evolution.csv`

**Purpose**: Track multi-objective optimization progress

**Required Columns**:
- `generation` (int): Generation number
- `individual_id` (int): Individual identifier within generation
- `front_rank` (int): Pareto front rank (1 = Pareto optimal)
- `f6_energy` (float): Energy objective value (J)

**Optional Columns**:
- `crowding_distance` (float): Crowding distance metric
- `f1_length` (float): Path length objective (m)
- `f2_curvature` (float): Path curvature objective
- `f3_obstacle` (float): Obstacle clearance objective
- `f4_localization` (float): Localization uncertainty objective
- `f5_deviation` (float): Path deviation objective
- `f7_stability` (float): Stability objective

**Example**:
```csv
generation,individual_id,front_rank,crowding_distance,f1_length,f2_curvature,f3_obstacle,f4_localization,f5_deviation,f6_energy,f7_stability
0,0,1,0.567,12.3,0.45,2.1,0.012,0.15,145.6,0.89
0,1,1,0.823,13.1,0.38,2.5,0.010,0.12,138.2,0.91
0,2,2,0.412,11.8,0.52,1.9,0.015,0.18,152.3,0.85
```

### 6. Ablation Results (Optional)

**Filename**: `ablation_results.csv`

**Purpose**: Analyze effect of removing system components

**Required Columns**:
- `configuration` (string): System configuration description
- `energy` (float): Energy consumption (J)
- `success_rate` (float): Success rate (0.0-1.0)

**Optional Columns**:
- `localization_rmse` (float): Localization RMSE (m)
- `planning_time` (float): Planning time (ms)
- `mape` (float): PINN MAPE (%) if testing PINN variants

**Example**:
```csv
configuration,energy,success_rate,localization_rmse,planning_time
Full_System,142.3,1.0,0.08,45.2
No_PINN,163.5,0.95,0.08,42.1
Single_Objective,155.7,0.98,0.08,38.5
EKF_Instead_UKF,148.2,0.92,0.12,45.0
```

## Data Collection Templates

See the `templates/` directory for:
- `navigation_trial_template.csv` - Template for recording navigation experiments
- `localization_trial_template.csv` - Template for localization experiments
- `pinn_training_template.csv` - Template for PINN training logs

## Tips for Data Collection

1. **Consistency**: Use consistent method names across all files (e.g., "PIEC", not "piec" or "Piec")
2. **Units**: Always use SI units (meters, seconds, joules, radians)
3. **Missing Data**: Use empty cells or "NaN" for missing optional data
4. **Booleans**: Use "True"/"False" for boolean columns
5. **File Encoding**: Save files as UTF-8 CSV

## Validation

To validate your data files before analysis:

```bash
python3 -c "
import pandas as pd
df = pd.read_csv('your_file.csv')
print(df.info())
print(df.head())
"
```

Check that:
- All required columns are present
- Data types are correct (int, float, string, bool)
- No unexpected NaN values in required columns
- Method/environment names are consistent

## Converting Existing Data

If you have data in other formats:

```python
import pandas as pd
import rosbag  # For ROS bag files

# From ROS bag
bag = rosbag.Bag('experiment.bag')
# Extract and convert to DataFrame...

# From numpy arrays
data = {
    'epoch': np.arange(100),
    'train_loss': train_losses,
    'val_loss': val_losses
}
df = pd.DataFrame(data)
df.to_csv('pinn_training_history.csv', index=False)
```
