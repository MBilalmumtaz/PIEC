# Issue Resolution: RViz Shows Front as Back

## Problem (As Clarified by User)

**Simple and Clear Issue:**
- Real robot moves correctly ✓
- RViz motion is correct (forward shows as forward) ✓  
- BUT: Visual model orientation is backward (front appears as back) ✗
- Only affects real robot, not simulation

**User's Words:**
> "in rviz motion was good just opposite direction"  
> "just in rviz front side of robot shows back side"

## Solution

**Changed one value in URDF to flip visual orientation by 180°**

### The Fix
```xml
File: src/scout_lidar_imu/agilex_scout/urdf/mobile_robot/scout_mini.urdf.xacro

BEFORE:
<origin xyz="0 0 0" rpy="1.57 0 1.57"/>
                              ^^^^
                            yaw = 90°

AFTER:
<origin xyz="0 0 0" rpy="1.57 0 -1.57"/>
                              ^^^^^
                            yaw = -90° (= 270°)
```

### What This Does
- **Roll** stays at 90° (1.57 rad) - aligns mesh vertical axis
- **Pitch** stays at 0° - no tilt
- **Yaw** changes from 90° to -90° - **adds 180° rotation to flip front/back**

Math: 90° + 180° = 270° = -90° (modulo 360°)

## Result

✅ **Visual model now shows front as front**  
✅ **IMU arrow points to actual front**  
✅ **RViz visualization matches reality**  

## What Didn't Change

- Robot control system (was already working)
- Movement direction (was already correct)  
- Odometry, IMU, sensors (all unchanged)
- Any robot functionality

## Testing

```bash
# Launch the robot
ros2 launch agilex_scout scout_robot_lidar.launch.py

# Command forward motion
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"

# Verify in RViz:
# 1. Robot moves forward (positive X) ✓
# 2. Front of visual model faces forward ✓
# 3. Everything matches reality ✓
```

## Summary

This was a **pure visual orientation fix**:
- Changed 1 number: yaw from `1.57` to `-1.57`
- Added 180° rotation to flip the mesh
- Fixed how the robot LOOKS in RViz
- Did NOT change how the robot WORKS

The robot was always moving correctly - now the visualization shows it correctly too!
