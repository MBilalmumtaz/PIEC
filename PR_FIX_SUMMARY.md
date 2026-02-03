# Scout Mini Navigation Reliability Fixes - Summary

## Issues Fixed

This PR addresses three critical navigation reliability issues reported for the Scout Mini real robot:

### 1. Angular Sign Mismatch (Primary Issue)
**Problem:** Robot turns in wrong direction for right-side goals, enters recovery loops
- Scout Mini hardware interprets positive `cmd_vel.angular.z` as clockwise (CW) rotation
- This is opposite to ROS standard (positive = counter-clockwise/CCW)
- Controller computes correct heading error but sends inverted angular command to robot

**Fix:** Changed `angular_sign_correction` from 1.0 to -1.0 in `piec_real_robot.launch.py`
- Now multiplies controller output by -1.0 before sending to robot
- Right-side goals now correctly produce CW rotation
- Left-side goals now correctly produce CCW rotation

**Files Changed:**
- `src/piec_bringup/piec_bringup/launch/piec_real_robot.launch.py` (line 263)

### 2. TF Transform Conflict (Secondary Issue)
**Problem:** TF validator reports incorrect base_link→imu_link transform
- URDF defined `imu_link` as child of `os_sensor` 
- Launch file also published static TF from `base_link` to `imu_link`
- Result: Two conflicting transform paths, TF validator failures

**Fix:** Made URDF the single authoritative source for imu_link transform
- Updated `os1.urdf` to attach `imu_link` directly to `mobile_robot_base_link`
- Removed static TF publisher from `scout_robot_lidar.launch.py`
- Transform values: xyz=(0, 0, 0.2), rpy=(0, 0, -0.94) matching expected calibration

**Files Changed:**
- `src/scout_lidar_imu/agilex_scout/urdf/sensors/os1.urdf` (line 27-33)
- `src/scout_lidar_imu/agilex_scout/launch/scout_robot_lidar.launch.py` (removed lines 238-249)

### 3. Missing TF Validation
**Problem:** TF errors only discovered during operation, hard to diagnose

**Fix:** Added TF validator node to real robot launch
- Runs automatically at startup
- Validates all critical transforms (odom→base_footprint, base_link→imu_link, etc.)
- Provides clear pass/fail messages with actual vs expected values

**Files Changed:**
- `src/piec_bringup/piec_bringup/launch/piec_real_robot.launch.py` (lines 387-400)

## Testing

### Unit Tests
Added 4 new tests for angular sign correction in `test_controller_heading.py`:
1. `test_sign_correction_positive` - Validates standard ROS convention (sign=1.0)
2. `test_sign_correction_negative` - Validates Scout Mini correction (sign=-1.0)
3. `test_right_side_goal_with_sign_correction` - Full workflow for right turns
4. `test_left_side_goal_with_sign_correction` - Full workflow for left turns

**Result:** All 16 tests pass (12 existing + 4 new)

### Security
- CodeQL scan completed with 0 vulnerabilities
- No secrets or sensitive data exposed

## Documentation Updates

1. **TROUBLESHOOTING.md**
   - Updated angular_sign_correction default from 1.0 to -1.0
   - Marked "wrong direction" issue as fixed
   - Added note about corrected launch file defaults

2. **README.md**
   - Updated controller calibration section
   - Changed TF frame table to reflect URDF source for imu_link
   - Clarified Scout Mini-specific angular sign behavior

## Expected Behavior After Fix

With `ros2 launch piec_bringup piec_real_robot.launch.py`:

✅ **TF Validator**
- All transforms validated at startup
- base_link→imu_link shows correct values: xyz=(0, 0, 0.2), yaw=-0.94 rad
- No duplicate transform warnings

✅ **Controller Behavior**
- Goal on right side → robot turns right (CW) toward goal
- Goal on left side → robot turns left (CCW) toward goal
- No recovery loops or oscillation in open space

✅ **Path Following**
- Robot follows `/piec/path` without requiring explicit goal
- Completes goals consistently in empty environment
- Smooth navigation without unexpected divergence

## Minimal Changes Philosophy

This PR makes surgical, focused changes:
- 1 parameter value changed (angular_sign_correction)
- 1 URDF joint redefined (parent changed from os_sensor to base_link)
- 1 static TF publisher removed (to eliminate duplication)
- 1 node added to launch (TF validator for diagnostics)
- Documentation updated to reflect fixes

**Total:** 6 files modified, +110 lines, -23 lines (net +87)

No changes to:
- Core controller algorithm logic
- Path planning or optimization
- UKF localization
- Emergency stop or obstacle avoidance
- Any simulation or Gazebo files

## Validation Checklist

Before merging, verify on real Scout Mini hardware:

- [ ] TF validator passes all checks
- [ ] Send goal to robot's right side → robot turns right (CW)
- [ ] Send goal to robot's left side → robot turns left (CCW)  
- [ ] Send goal straight ahead → robot moves forward with minimal turning
- [ ] Robot completes multiple goals in sequence without getting stuck
- [ ] No "No transform from X to Y" errors in logs
- [ ] `ros2 run tf2_tools view_frames` shows single path: odom→base_footprint→base_link→imu_link

## Rollback Plan

If issues occur, revert by changing one line in `piec_real_robot.launch.py`:
```python
"angular_sign_correction": 1.0,  # Revert to old behavior
```

This will restore previous behavior while keeping TF fixes.
