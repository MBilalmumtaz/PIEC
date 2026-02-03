# PIEC Robot Troubleshooting Checklist

This document provides systematic troubleshooting steps for common issues with the Scout Mini PIEC robot.

## Pre-Flight Checklist

Before reporting an issue, verify these basics:

- [ ] CAN bus is configured and UP: `ip -d link show can0`
- [ ] LiDAR network is configured: `ping 192.168.1.200`
- [ ] All required topics are publishing: `ros2 topic list`
- [ ] TF tree is complete: `ros2 run piec_bringup tf_validator`
- [ ] IMU is publishing: `ros2 topic hz /imu` (should be ~100 Hz)
- [ ] Odometry is publishing: `ros2 topic hz /odometry` (should be ~50 Hz)
- [ ] Battery is charged (> 11V)

## Issue Categories

### 1. Visualization Issues

#### 1A. Robot Appears Reversed in RViz

**Symptom:** Robot moves correctly, but visual model in RViz shows front as back.

**Diagnosis:**
```bash
# 1. Check URDF mesh orientation
ros2 param get /robot_state_publisher robot_description | grep -A 3 "mobile_robot_base_link"
# Should show rpy="1.57 0 -1.57" (90° roll, -90° yaw)

# 2. Test movement and orientation
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"
# Robot should:
# - Move forward in RViz (positive X direction) ✓
# - Visual model's front should face forward ✓
```

**Fix:**
- Ensure using correct URDF configuration
- File: `src/scout_lidar_imu/agilex_scout/urdf/mobile_robot/scout_mini.urdf.xacro`
- Line 56 should be: `<origin xyz="0 0 0" rpy="1.57 0 -1.57"/>` ✓ CORRECT
- Roll: 90° (1.57) aligns mesh vertical axis
- Yaw: -90° (-1.57) corrects front/back visual orientation
- **NOT**: `rpy="1.57 0 1.57"` (shows robot backward)

#### 1B. TF Tree Broken or Missing Transforms

**Symptom:** RViz shows "No transform from X to Y" errors.

**Diagnosis:**
```bash
# Validate TF tree
ros2 run piec_bringup tf_validator

# Visualize full tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

**Fix:**
- Ensure `scout_robot_lidar.launch.py` is running
- Check that static TF publishers are active:
  ```bash
  ros2 node list | grep static_transform_publisher
  ```
- Verify frames match expected names (not `base_footprint_fixed` etc.)

---

### 2. IMU Issues

#### 2A. IMU Data Inconsistent Between Standalone and Launch

**Symptom:** Running `openzen_node` standalone gives different values than when launched via `scout_robot_lidar.launch.py`.

**Diagnosis:**
```bash
# Test standalone
ros2 run openzen_driver openzen_node --ros-args -r __ns:=/openzen
ros2 topic echo /openzen/data --field orientation --once

# Test via launch
ros2 launch agilex_scout scout_robot_lidar.launch.py
ros2 topic echo /imu --field orientation --once

# Compare yaw values (should differ by ~-0.94 rad due to yaw_offset)
```

**Root Causes:**
1. **Frame ID mismatch** - Check that both use `imu_link`
2. **Yaw offset mismatch** - Static TF and Madgwick filter must agree
3. **QoS mismatch** - Some nodes may lose messages with default QoS

**Fix:**
Verify in `scout_robot_lidar.launch.py`:
```python
# Line 167: OpenZen frame_id
'frame_id': 'imu_link',

# Line 191: Madgwick yaw_offset
'yaw_offset': -0.94,

# Line 243: Static TF yaw
arguments=['0', '0', '0.2', '0', '0', '-0.94', 'base_link', 'imu_link'],
```

All three yaw values should match (-0.94 rad).

#### 2B. IMU Orientation Incorrect

**Symptom:** Robot thinks it's facing wrong direction, UKF diverges.

**Diagnosis:**
```bash
# Check IMU orientation visually
python3 check_imu_orientation.py

# Monitor IMU yaw while rotating robot
ros2 topic echo /imu --field orientation
# Rotate robot 90° CCW - yaw should increase by ~1.57 rad

# Check magnetometer
python3 test_magnetometer.py
# Rotate robot in circle - mag vector should rotate smoothly
```

**Fix:**
1. **Recalibrate magnetometer:**
   ```bash
   python3 calibrate_magnetometer.py
   # Follow instructions to rotate robot in figure-8
   ```

2. **Adjust yaw offset:**
   - Place robot facing known direction (e.g., magnetic north)
   - Read IMU yaw: `ros2 topic echo /imu --field orientation`
   - Calculate needed offset = (desired_yaw - measured_yaw)
   - Update `yaw_offset` in `scout_robot_lidar.launch.py`

3. **Verify transform:**
   ```bash
   ros2 run tf2_ros tf2_echo base_link imu_link
   # Should show yaw rotation matching yaw_offset
   ```

---

### 3. Path Following / Navigation Issues

#### 3A. Robot Not Reaching Goals

**Symptom:** Robot stops before goal, or never attempts to move toward goal.

**Diagnosis:**
```bash
# Check if goal is received
ros2 topic echo /goal_pose --once

# Check if path is generated
ros2 topic echo /piec/path

# Check controller status
ros2 topic echo /controller/status

# Monitor velocity commands
ros2 topic echo /cmd_vel &
ros2 topic echo /cmd_vel_safe
```

**Checklist:**
- [ ] Goal is in correct frame (`odom` not `map`)
- [ ] Path optimizer is running: `ros2 node list | grep pinn_path_optimizer`
- [ ] Controller is running: `ros2 node list | grep controller`
- [ ] UKF is publishing: `ros2 topic hz /ukf/odom`
- [ ] No obstacles blocking path: `ros2 topic echo /scan --field ranges | head`

**Tuning Parameters:**

If robot stops too far from goal:
```python
# In piec_real_robot.launch.py, controller_node:
"goal_completion_distance": 0.50,  # Increase from 0.40
"waypoint_tolerance": 0.50,        # Increase from 0.40
```

If robot is too slow:
```python
"max_linear_vel": 1.0,      # Increase from 0.8
"min_linear_vel": 0.20,     # Ensure minimum motion
"linear_scale_factor": 1.0, # Increase from 0.95
```

#### 3B. Motor Unresponsive Warnings

**Symptom:**
```
⚠️ Motors unresponsive - hardware issue detected
Motor responsiveness: 42.5%
```

**Diagnosis:**
```bash
# 1. Check CAN bus communication
candump can0 -n 50
# Should see continuous messages from motor controller

# 2. Check odometry update rate
ros2 topic hz /odometry
# Should be 40-50 Hz, not < 20 Hz

# 3. Send direct command and monitor
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}" &
ros2 topic echo /odometry --field twist.twist.linear.x
# Should see velocity response within 1 second
```

**Common Causes:**
1. **CAN bus error** - Loose cable, wrong bitrate, or no power to base
2. **Low battery** - Robot limits power when < 11V
3. **Emergency stop engaged** - Check for obstacles
4. **Velocity scaling mismatch** - Commanded != actual

**Fix:**

1. **CAN bus:**
   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set can0 up
   ip -d link show can0  # Verify bitrate
   ```

2. **Reduce sensitivity (if false positives):**
   Edit `controller_node.py`:
   ```python
   self.motor_responsiveness_threshold = 50.0  # Was 70.0
   ```

3. **Check velocity scaling:**
   ```python
   # In piec_real_robot.launch.py:
   "linear_scale_factor": 0.95,  # Try 1.0 if robot is slow
   "angular_scale_factor": 0.99,
   ```

#### 3C. Robot Oscillates or Spins in Place

**Symptom:** Robot rotates rapidly without making forward progress, or oscillates left-right.

**Diagnosis:**
```bash
# Monitor angular velocity
ros2 topic echo /cmd_vel --field angular.z

# Check oscillation detection
# Enable debug mode in launch file, then:
ros2 launch piec_bringup piec_real_robot.launch.py

# Look for logs:
# "🔄 Oscillation detected"
# "Avg angular: 0.65 rad/s, avg linear: 0.15 m/s"
```

**Root Causes:**
1. **Narrow passage** - Robot sees only small gaps, tries to thread through
2. **Goal too close to obstacle** - No clear path
3. **Angular gain too high** - Over-corrects steering

**Fix:**

1. **Increase linear bias:**
   ```python
   "min_linear_vel": 0.25,  # Force more forward motion (was 0.15)
   ```

2. **Reduce angular sensitivity:**
   ```python
   "angular_scale_factor": 0.85,  # Was 0.99
   "max_angular_vel": 0.5,        # Limit rotation speed
   ```

3. **Increase oscillation threshold:**
   ```python
   # In controller_node.py, detect_oscillation():
   is_oscillating = avg_angular > 0.5 and avg_linear < 0.15  # Was 0.4, 0.2
   ```

4. **Widen free space requirement:**
   ```python
   "safe_distance": 0.8,  # Require more clearance (was 1.0)
   ```

#### 3D. Robot Gets Stuck and Doesn't Recover

**Symptom:** Robot stops moving, recovery maneuvers fail.

**Diagnosis:**
```bash
# Check stuck detection
# Look for logs:
# "⚠️ Low movement detected: 0.045m"
# "🔄 INITIATING RECOVERY"

# Monitor position history
ros2 topic echo /ukf/odom --field pose.pose.position
# Should change over time, not static
```

**Tuning Recovery:**

If recovery triggers too often (false positives):
```python
"stuck_threshold_time": 12.0,     # Increase from 10.0
"stuck_threshold_distance": 0.10, # Increase from 0.08
```

If recovery never triggers:
```python
"stuck_threshold_time": 6.0,      # Decrease from 10.0
"enable_stuck_recovery": True,    # Verify enabled
```

If recovery fails to escape:
```python
"max_backup_distance": 1.5,       # Increase from 1.0
"recovery_duration": 6.0,         # Increase from 4.0
```

---

### 4. Sensor Issues

#### 4A. No LiDAR Data

**Symptom:** `/scan` topic not publishing, or all ranges are `inf`.

**Diagnosis:**
```bash
# 1. Check network connectivity
ping 192.168.1.200
# Should respond if LiDAR powered on

# 2. Check topic
ros2 topic hz /rslidar_points  # Raw 3D points
ros2 topic hz /scan            # Converted 2D scan

# 3. Check laser scan ranges
ros2 topic echo /scan --field ranges
# Should show varying values, not all inf
```

**Fix:**
1. **Network not configured:**
   ```bash
   sudo ip link set <your_ethernet_interface> up
   sudo ip addr add 192.168.1.102/24 dev <your_ethernet_interface>
   ```

2. **Wrong interface name:**
   Edit `scout_robot_lidar.launch.py` if your Ethernet interface isn't `enp0s31f6`

3. **LiDAR offline:**
   - Check power cable
   - Check if LED on LiDAR is lit
   - Try rebooting LiDAR (power cycle)

#### 4B. Odometry Jumps or Drifts

**Symptom:** Robot position in RViz jumps suddenly, or drifts over time.

**Diagnosis:**
```bash
# Monitor odometry
ros2 topic echo /odometry --field pose.pose.position

# Monitor UKF odometry
ros2 topic echo /ukf/odom --field pose.pose.position

# Check update rates
ros2 topic hz /odometry  # Should be ~50 Hz
ros2 topic hz /imu       # Should be ~100 Hz
```

**Causes:**
1. **Wheel slip** - Odometry assumes no slip
2. **IMU drift** - Magnetometer not calibrated
3. **TF discontinuity** - Transform updates not smooth

**Fix:**
1. **Calibrate IMU:** `python3 calibrate_magnetometer.py`
2. **Increase UKF trust in IMU:**
   ```python
   # In piec_real_robot.launch.py, UKF:
   "use_imu_orientation": True,
   "R_imu_orientation": 0.0005,  # Trust IMU more (was 0.001)
   ```

3. **Reduce wheel odometry trust:**
   ```python
   "R_odom_position": 0.02,  # Trust wheels less (was 0.01)
   ```

---

## Advanced Debugging

### Enable Full Debug Logging

Edit `piec_real_robot.launch.py`:
```python
# All nodes:
"debug_mode": True,

# ROS 2 log level:
output='screen',
arguments=['--ros-args', '--log-level', 'DEBUG'],
```

### Record Diagnostic Bag

```bash
# Record all topics for 60 seconds
ros2 bag record -a -d 60

# Or specific topics only
ros2 bag record /odometry /imu /scan /ukf/odom /cmd_vel /piec/path
```

### Visualize TF Tree

```bash
# Generate PDF of TF tree
ros2 run tf2_tools view_frames

# View
evince frames.pdf

# Or live view
ros2 run rqt_tf_tree rqt_tf_tree
```

### Monitor System Resources

```bash
# CPU and memory usage
htop

# ROS 2 nodes CPU usage
ros2 run rqt_top rqt_top
```

---

## Known Limitations

1. **URDF Mesh Orientation**: Original mesh files may be rotated. Our URDF compensates, but if using different mesh, adjust rotation.

2. **IMU Yaw Offset**: Specific to physical mounting. If IMU remounted, recalibrate offset.

3. **Wheel Odometry Accuracy**: Scout Mini wheels can slip. UKF helps but localization will drift on slippery surfaces.

4. **LiDAR Height**: Fixed at 30cm. Very low obstacles may not be detected.

5. **Path Optimizer Timeout**: Complex environments may need longer `optimization_timeout`.

---

## Getting Help

If issues persist after following this guide:

1. **Collect debug information:**
   ```bash
   # Save to debug.txt
   {
     echo "=== ROS 2 Topics ==="
     ros2 topic list
     echo ""
     echo "=== ROS 2 Nodes ==="
     ros2 node list
     echo ""
     echo "=== TF Validation ==="
     ros2 run piec_bringup tf_validator
     echo ""
     echo "=== Topic Rates ==="
     timeout 5 ros2 topic hz /odometry
     timeout 5 ros2 topic hz /imu
     timeout 5 ros2 topic hz /scan
   } > debug.txt
   ```

2. **Record a short bag:**
   ```bash
   ros2 bag record -d 30 /odometry /imu /scan /ukf/odom /cmd_vel /piec/path
   ```

3. **Include in issue report:**
   - `debug.txt`
   - Bag file
   - Launch command used
   - Description of unexpected behavior
   - Photo/video if visualization issue

---

## Quick Reference

### Expected Topic Rates
- `/odometry`: 50 Hz
- `/imu`: 100 Hz
- `/openzen/data`: 200 Hz
- `/scan`: 10 Hz
- `/ukf/odom`: 20 Hz
- `/piec/path`: 1.5 Hz
- `/cmd_vel`: 10 Hz

### Critical Parameters
- **Goal completion**: 0.40 m
- **Stuck threshold**: 10 seconds, 0.08 m
- **Grace period**: 6 seconds (stuck), 5 seconds (oscillation)
- **Emergency stop**: 0.30 m
- **Max speeds**: 0.8 m/s linear, 0.6 rad/s angular

### Key Files
- **Main launch**: `src/piec_bringup/piec_bringup/launch/piec_real_robot.launch.py`
- **Hardware launch**: `src/scout_lidar_imu/agilex_scout/launch/scout_robot_lidar.launch.py`
- **URDF**: `src/scout_lidar_imu/agilex_scout/urdf/mobile_robot/scout_mini.urdf.xacro`
- **Controller**: `src/piec_controller/piec_controller/controller_node.py`
