# Scout Mini PIEC Robot Project

This repository contains the complete ROS 2 stack for the Scout Mini robot with PIEC (Physics-Informed Enhanced Control) system including IMU, LiDAR, UKF localization, path optimization, and PINN-based trajectory evaluation.

## Table of Contents
- [Hardware Setup](#hardware-setup)
- [Build and Install](#build-and-install)
- [Quick Start](#quick-start)
- [Hardware Calibration](#hardware-calibration)
- [Verification and Diagnostics](#verification-and-diagnostics)
- [Troubleshooting](#troubleshooting)
- [Topic Reference](#topic-reference)

## Hardware Setup

### Prerequisites
- Scout Mini robot with CAN interface
- LPMS IG1 CAN IMU sensor
- RoboSense RSHELIOS-16 LiDAR
- Ubuntu 22.04 with ROS 2 Humble

### Required Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-imu-filter-madgwick \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-teleop-twist-keyboard \
  can-utils
```

### CAN Bus Setup
The Scout Mini base communicates over CAN at 500 kbps:

```bash
# Set up CAN interface (run once per boot)
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Verify CAN is active
ip -details link show can0
# Should show: state UP, bitrate 500000

# Test CAN traffic (should see messages from robot base)
candump can0 -n 20
```

### LiDAR Network Setup
The RSHELIOS LiDAR requires a dedicated Ethernet interface:

```bash
# Replace 'enp0s31f6' with your Ethernet interface name
# Find it with: ip link show

sudo ip link set enp0s31f6 up
sudo ip addr add 192.168.1.102/24 dev enp0s31f6

# Verify
ip addr show enp0s31f6
ping 192.168.1.200  # Should respond if LiDAR is powered on
```

### IMU Permissions
The LPMS IG1 IMU may connect via USB initially for configuration:

```bash
# One-time setup - add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout and login again

# Or temporary fix
sudo chmod 666 /dev/ttyUSB0
```

## Build and Install

```bash
# Clone repository
cd ~/scoutmini_ws3/src  # or your workspace
git clone https://github.com/MBilalmumtaz/scoutmini-project.git

# Build
cd ~/scoutmini_ws3
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Quick Start

### 1. Hardware Verification (Base + Sensors Only)
Test all hardware before launching the full stack:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch base hardware (robot base, LiDAR, IMU, RViz)
ros2 launch agilex_scout scout_robot_lidar.launch.py

# Expected topics should appear:
# /odometry (50 Hz) - wheel odometry
# /scan (10 Hz) - laser scan
# /imu (100 Hz) - filtered IMU
# /openzen/data (200 Hz) - raw IMU
# /cmd_vel - command input
```

**Verification Steps:**
- RViz should open showing the robot model
- Robot should appear upright and forward-facing (not reversed)
- Run `ros2 topic list` to verify all topics are publishing
- Check `ros2 topic hz /imu` - should be ~100 Hz
- Check `ros2 topic hz /scan` - should be ~10 Hz
- Check `ros2 topic hz /odometry` - should be ~50 Hz

### 2. Full PIEC Stack
Once hardware is verified, launch the complete autonomy stack:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch everything (UKF, controller, path optimizer, PINN)
ros2 launch piec_bringup piec_real_robot.launch.py
```

**What this includes:**
- UKF localization (fuses odometry + IMU)
- PIEC controller (path following with obstacle avoidance)
- Complete path optimizer (`complete_path_optimizer.py` - GA-based with PINN integration)
- PINN service (physics-informed predictions)
- Emergency stop and safety monitors

### 3. Send Navigation Goals

```bash
# Option 1: Use RViz "2D Goal Pose" tool
# Click "2D Goal Pose" button and click/drag on map

# Option 2: Command line
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
           orientation: {w: 1.0}}}"

# Option 3: Use manual goal setter script
python3 manual_goal_setter.py
```

## Hardware Calibration

### IMU Calibration
The IMU requires proper yaw offset to align with the robot's coordinate frame:

**Current settings** (already configured in `scout_robot_lidar.launch.py`):
- Static TF yaw offset: `-0.94 rad` (-54°)
- Madgwick filter yaw offset: `-0.94 rad`
- Magnetometer declination: `0.092 rad` (adjust for your location)

**To recalibrate:**
```bash
# 1. Check IMU orientation visually
python3 check_imu_orientation.py

# 2. Calibrate magnetometer (hard/soft iron compensation)
python3 calibrate_magnetometer.py
# Follow on-screen instructions to rotate robot in figure-8 pattern

# 3. Test IMU drift
python3 test_imu_drift.py

# 4. Verify magnetometer
python3 test_magnetometer.py
```

**Expected IMU behavior:**
- When robot faces forward (X+), IMU yaw should read ~0
- Robot rotating counterclockwise should increase yaw
- Magnetometer should point to magnetic north consistently

### Motor Scaling Calibration
If the robot moves slower or faster than commanded:

Edit `/src/piec_bringup/piec_bringup/launch/piec_real_robot.launch.py`:
```python
scout_mini_params = {
    "linear_scale_factor": 0.95,  # Adjust between 0.8-1.2
    "angular_scale_factor": 0.99,  # Adjust between 0.8-1.2
    "speed_calibration_factor": 0.95,
}
```

**Controller calibration** (if angular direction is reversed):
```python
# In controller_node parameters:
"angular_sign_correction": 1.0,  # Use -1.0 if rotation is inverted
```

### Goal Completion Tuning
Adjust goal completion distance if robot doesn't reach goals:

```python
# In piec_real_robot.launch.py, controller_node parameters:
"waypoint_tolerance": 0.4,  # Distance to consider waypoint reached (meters)

# In controller_node.py:
self.goal_completion_distance = 0.40  # Final goal threshold (meters)
```

## Verification and Diagnostics

### TF Tree Verification
Ensure all transforms are correct:

```bash
# View TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf - open and verify:
#   odom -> base_footprint -> base_link -> imu_link
#                                       -> rslidar

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link

# Verify IMU transform
ros2 run tf2_ros tf2_echo base_link imu_link
# Should show: translation [0, 0, 0.2], rotation ~-54° yaw
```

### Run Diagnostic Scripts

```bash
# Check frame alignment and transforms
python3 diagnose_frames.py

# Verify IMU data flow and frame IDs
python3 diagnose_imu.py

# Test UKF performance
python3 test_ukf.py
```

### Monitor Controller Status

```bash
# Watch controller debug output
ros2 topic echo /controller/debug --no-arr

# Monitor commanded vs actual velocities
ros2 topic echo /cmd_vel &
ros2 topic echo /odometry --field twist.twist
```

## Troubleshooting

### Issue: Robot Moves Backward in RViz but Forward in Reality

**Fixed in latest version.** The URDF mesh now has a 180° yaw rotation to align RViz visualization with real robot orientation.

**To verify fix:**
1. Launch `ros2 launch agilex_scout scout_robot_lidar.launch.py`
2. Send a forward command: `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"`
3. In RViz, robot should move forward (positive X direction) matching real robot movement

**Current configuration:**
- `scout_mini.urdf.xacro` has `rpy="0 0 ${M_PI}"` for base_link mesh (180° yaw rotation)
- This ensures RViz visualization matches real robot movement direction
- Wheel axes are `xyz="0 0 1"` (standard convention)

**If visualization is still incorrect:**
- Verify you're using the latest URDF files
- The mesh rotation compensates for how the 3D mesh was originally modeled
- Different mesh files may require different rotation values

### Issue: IMU Data Different Between Standalone and Launch

**Fixed in latest version.** The yaw offset is now consistent (`-0.94 rad`) between:
- Static TF publisher (`base_link` -> `imu_link`)
- Madgwick filter yaw offset

**To verify:**
```bash
# Check standalone IMU
ros2 run openzen_driver openzen_node --ros-args -r __ns:=/openzen
ros2 topic echo /openzen/data --field orientation

# Check via launch file
ros2 launch agilex_scout scout_robot_lidar.launch.py
ros2 topic echo /imu --field orientation

# Yaw values should be consistent (accounting for -0.94 rad offset)
```

### Issue: Robot Not Reaching Goals / Path Following Fails

**Symptoms:**
- Robot stops before reaching goal
- Logs show "motor unresponsive" warnings
- Robot oscillates or gets stuck

**Recent improvements:**
- Grace period reduced to 6s (was 12s) for faster stuck detection
- Goal completion distance increased to 40cm (was 30cm) for reliability
- Stuck threshold increased to 10s to reduce false positives

**Additional debugging:**
```bash
# Enable debug mode to see detailed logs
# In piec_real_robot.launch.py, set:
"debug_mode": True,

# Check if goals are being received
ros2 topic echo /goal_pose

# Check if paths are being generated
ros2 topic echo /piec/path

# Monitor UKF odometry
ros2 topic echo /ukf/odom --field pose.pose.position
```

**Common causes:**
1. **Obstacles blocking path**: Robot will stop if obstacle within 30cm
   - Check `ros2 topic echo /scan --field ranges` for minimum distances
   - Reduce `emergency_stop_distance` if too conservative

2. **IMU/Odometry mismatch**: UKF may diverge if IMU not calibrated
   - Run `python3 check_imu_orientation.py`
   - Verify transforms: `ros2 run tf2_ros tf2_echo base_link imu_link`

3. **Low motor responsiveness**: Check CAN bus communication
   - `candump can0` should show steady traffic
   - Verify `bitrate 500000` is set correctly

4. **Path optimizer timeout**: PINN predictions may be slow
   - Increase `optimization_timeout` in launch file
   - Or disable PINN: `"use_pinn_predictions": False`

### Issue: Motor Unresponsive Warnings

**Symptoms:**
```
⚠️ Motors unresponsive - hardware issue detected
Motor responsiveness: 45.2%
```

**Causes:**
- CAN bus communication issues
- Low battery voltage
- Odometry update rate too low
- Velocity scaling mismatch

**Solutions:**
1. **Check CAN bus:**
   ```bash
   candump can0
   # Should see continuous messages, not sparse
   ```

2. **Check odometry rate:**
   ```bash
   ros2 topic hz /odometry
   # Should be ~50 Hz, not < 20 Hz
   ```

3. **Adjust responsiveness threshold:**
   Edit `controller_node.py`:
   ```python
   # Reduce sensitivity if false positives
   self.motor_responsiveness_threshold = 50.0  # was 70.0
   ```

4. **Check battery voltage** - Scout Mini may limit power when low

### Issue: Oscillation or Spinning in Place

**Symptoms:**
- Robot spins without making forward progress
- Logs show "Oscillation detected"

**Causes:**
- Free space analysis finding only narrow gaps
- Goal too close to obstacles
- Angular gain too high

**Solutions:**
1. **Increase linear velocity bias:**
   ```python
   # In controller_node parameters:
   "min_linear_vel": 0.20,  # Force more forward motion
   ```

2. **Reduce angular sensitivity:**
   ```python
   "angular_scale_factor": 0.9,  # Was 0.99
   ```

3. **Disable oscillation recovery temporarily:**
   ```python
   "enable_oscillation_detection": False,
   ```

## Topic Reference

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odometry` | `nav_msgs/Odometry` | 50 Hz | Wheel encoder odometry from Scout base |
| `/imu` | `sensor_msgs/Imu` | 100 Hz | Filtered IMU (Madgwick) with mag correction |
| `/openzen/data` | `sensor_msgs/Imu` | 200 Hz | Raw IMU data from LPMS IG1 |
| `/openzen/mag` | `sensor_msgs/MagneticField` | 200 Hz | Magnetometer data |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | 2D laser scan from LiDAR |
| `/ukf/odom` | `nav_msgs/Odometry` | 20 Hz | Fused odometry (UKF: wheels + IMU) |
| `/piec/path` | `nav_msgs/Path` | 1.5 Hz | Optimized path from GA planner |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | 10 Hz | Safety-filtered velocity commands |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal in `odom` frame |
| `/cmd_vel` | `geometry_msgs/Twist` | Final velocity command to robot |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/evaluate_trajectory` | `EvaluateTrajectory` | PINN-based trajectory evaluation |

### TF Frames

| Parent | Child | Transform | Notes |
|--------|-------|-----------|-------|
| `odom` | `base_footprint` | Dynamic | Published by Scout base or UKF |
| `base_footprint` | `base_link` | Static (0, 0, 0.237m) | Height offset |
| `base_link` | `imu_link` | Static (0, 0, 0.2m, yaw: -0.94 rad) | IMU mount |
| `base_link` | `rslidar` | Static (0.15, 0, 0.3m) | LiDAR mount |

## Advanced Configuration

### Controller Parameters
See `src/piec_bringup/piec_bringup/launch/piec_real_robot.launch.py`:
- `max_linear_vel`: Maximum forward speed (m/s)
- `max_angular_vel`: Maximum rotation speed (rad/s)
- `emergency_stop_distance`: Obstacle distance for immediate stop (m)
- `waypoint_tolerance`: Distance to consider waypoint reached (m)
- `lookahead_distance`: Pure pursuit lookahead (m)

### UKF Parameters
See `piec_real_robot.launch.py`, UKF section:
- `use_imu_orientation`: Fuse IMU yaw (recommended: `True`)
- `imu_yaw_offset`: IMU heading correction (rad)
- `Q_position`, `Q_orientation`: Process noise
- `R_imu_orientation`, `R_odom_position`: Measurement noise

### Path Optimizer
The system uses `complete_path_optimizer` (file: `complete_path_optimizer.py`) which integrates PINN predictions with genetic algorithm-based trajectory planning. This is the enhanced version with full obstacle awareness and physics-informed optimization.

**Note:** A simpler `path_optimizer` executable also exists but is not used by the main launch file.

Parameters in `piec_real_robot.launch.py`:
- `population_size`: GA population (default: 15)
- `generations`: GA iterations (default: 6)
- `optimization_timeout`: Max planning time (s)
- `use_pinn_predictions`: Enable PINN integration (default: True)

## Contributing
Please ensure all changes pass existing tests and add new tests for new features.

## License
See LICENSE file for details.
