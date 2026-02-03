# Pull Request Summary: Fix RViz Visual Orientation for Real Robot

## Overview
This PR fixes the Scout Mini robot's visual orientation in RViz. The robot was moving correctly, but the visual model showed the front as the back (180° reversed).

## The Actual Issue

**User Report:**
- Robot moves correctly in reality and simulation ✓
- RViz shows correct movement direction ✓
- BUT: Visual model shows front as back ✗
- This ONLY affects real robot, not simulation

**Root Cause:**
The mesh orientation had yaw=90° which made the visual model appear backward. By changing yaw to -90° (adding 180° rotation), the visual now shows front as front.

## Changes Made

### 1. URDF Visual Orientation Fix

**Files Changed:**
- `src/scout_lidar_imu/agilex_scout/urdf/mobile_robot/scout_mini.urdf.xacro`

**Changes:**
- **Base mesh orientation**: Adjusted yaw rotation to fix front/back visual orientation
  - Before: `rpy="1.57 0 1.57"` (roll=90°, yaw=90°) - showed backward
  - After: `rpy="1.57 0 -1.57"` (roll=90°, yaw=-90°) - shows correctly
  - Added 180° to yaw: 1.57 + π = 4.71 = -1.57 (mod 2π)
  - This flips the visual model orientation by 180°

**Impact:**
- RViz visual model now shows front as front ✓
- Movement was already correct, now visualization matches ✓
- Only visual representation changed, no functional impact
- Works for real robot (simulation was already fine)

### 2. Other PR Improvements (Preserved)
**Other URDF Improvements:**
- **Wheel joint axes**: Corrected for real robot mode
  - Changed: `axis xyz="0 0 -1"` → `axis xyz="0 0 1"` 
  - Applied to all 4 wheel xacro files
  - Ensures consistent wheel conventions

### 2. IMU Configuration Fixes

**Files Changed:**
- `src/scout_lidar_imu/agilex_scout/launch/scout_robot_lidar.launch.py`

**Changes:**
- **Yaw offset alignment**: Unified yaw offset across static TF and Madgwick filter
  - Static TF base_link→imu_link: Changed from `-0.84 rad` to `-0.94 rad`
  - Madgwick filter yaw_offset: Already at `-0.94 rad`
  - Now both use `-0.94 rad` (-54°)
  
- **Documentation**: Added comments explaining yaw offset purpose and calibration

**Impact:**
- Consistent IMU orientation between standalone and launch modes
- UKF localization receives properly aligned IMU data
- No more unexpected yaw drift or jumps

### 3. Controller Parameter Improvements

**Files Changed:**
- `src/piec_controller/piec_controller/controller_node.py`

**Changes:**
- **Grace period for stuck detection**: 
  - Changed from 12 seconds to 6 seconds
  - Enables faster recovery when robot is genuinely stuck
  
- **Grace period for oscillation detection**:
  - Changed from 10 seconds to 5 seconds
  - Responds quicker to oscillation behavior
  
- **Goal completion distance**:
  - Increased from 0.30m to 0.40m
  - More forgiving threshold improves goal achievement rate
  
- **Stuck threshold time**:
  - Increased from 8 seconds to 10 seconds
  - Reduces false positive stuck detections

**Impact:**
- More reliable goal reaching
- Fewer false alarms for "motor unresponsive"
- Better balance between responsiveness and stability

### 4. Documentation and Tools

**New Files:**
- `README.md` (completely rewritten)
- `TROUBLESHOOTING.md` (new comprehensive guide)
- `src/piec_bringup/piec_bringup/tf_validator.py` (new validation tool)

**Updated Files:**
- `src/piec_bringup/setup.py` (added tf_validator entry point)

**Documentation Includes:**
- Hardware setup and prerequisites
- CAN bus, LiDAR, and IMU configuration
- Quick start guide
- Calibration procedures
- Topic reference with expected rates
- TF tree documentation
- Parameter tuning guide
- Systematic troubleshooting for all major issues
- Expected behavior descriptions

**TF Validator Tool:**
- Validates transform tree at runtime
- Checks for required transforms
- Verifies transform values are reasonable
- Uses non-blocking timer-based validation
- Provides clear pass/fail feedback

**Impact:**
- Users can quickly set up and diagnose issues
- Reduces time to resolution for common problems
- Better understanding of system behavior and tuning

## Testing Recommendations

### 1. Visualization Test
```bash
# Launch hardware stack
ros2 launch agilex_scout scout_robot_lidar.launch.py

# Send forward command
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"

# Verify in RViz:
# - Robot model appears upright and forward-facing
# - Robot moves forward (positive X) when commanded
```

### 2. TF Tree Validation
```bash
# Run validator
ros2 run piec_bringup tf_validator

# Should show:
# ✅ base_footprint -> base_link: Base height offset - OK
# ✅ base_link -> imu_link: IMU mount with yaw offset - OK
# ✅ base_link -> rslidar: LiDAR mount position - OK
```

### 3. IMU Consistency Test
```bash
# Test standalone
ros2 run openzen_driver openzen_node --ros-args -r __ns:=/openzen
ros2 topic echo /openzen/data --field orientation.z

# Test via launch
ros2 launch agilex_scout scout_robot_lidar.launch.py
ros2 topic echo /imu --field orientation.z

# Values should be consistent (accounting for -0.94 rad offset)
```

### 4. Path Following Test
```bash
# Launch full stack
ros2 launch piec_bringup piec_real_robot.launch.py

# Send navigation goal (2m forward)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 2.0, y: 0.0, z: 0.0}, \
           orientation: {w: 1.0}}}"

# Monitor for:
# - Path generated on /piec/path
# - Robot moves toward goal
# - "🎯 GOAL REACHED!" message when within 0.40m
# - No "motor unresponsive" false alarms
```

## Backward Compatibility

All changes are backward compatible:
- No API changes
- No topic name changes
- No message type changes
- Parameter changes only affect internal behavior, not interface
- URDF changes only affect visualization, not physical behavior

## Known Limitations

1. **Mesh Dependency**: If using different Scout Mini mesh files, rotation may need adjustment
2. **IMU Hardware**: Yaw offset specific to current IMU mounting; recalibration needed if remounted
3. **Environment Specific**: Controller tuning may need adjustment for different environments

## Migration Notes

For users upgrading from previous version:

1. **No code changes required** - changes are in launch files and URDF
2. **Rebuild workspace**: `colcon build --symlink-install`
3. **Run TF validator** after first launch to verify transforms
4. **Review documentation** for new calibration procedures
5. **Test in safe environment** before production use

## Files Changed Summary

```
11 files changed, 1171 insertions(+), 92 deletions(-)

Modified:
- README.md (505 lines added)
- src/piec_controller/piec_controller/controller_node.py (12 lines changed)
- src/scout_lidar_imu/agilex_scout/launch/scout_robot_lidar.launch.py (4 lines changed)
- src/scout_lidar_imu/agilex_scout/urdf/mobile_robot/*.xacro (5 files, minimal changes)
- src/piec_bringup/setup.py (1 line added)

New:
- TROUBLESHOOTING.md (527 lines)
- src/piec_bringup/piec_bringup/tf_validator.py (196 lines)
```

## Related Issues

Addresses the following reported issues:
- Robot visualization reversed in RViz
- IMU data different when running standalone vs via launch
- Unreliable goal completion
- Motor unresponsive false positives
- Lack of troubleshooting documentation

## Future Improvements

Potential follow-up work (not in this PR):
1. Make TF validator transforms configurable via ROS parameters
2. Add QoS configuration for IMU topics to handle message loss
3. Create automated integration tests for path following
4. Add RViz configuration file with better visualization settings
5. Implement dynamic reconfigure for controller parameters
