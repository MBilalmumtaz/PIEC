# RViz Visualization Fix - 180° Rotation Explanation

## Problem Statement

**Symptoms:**
- Real Scout Mini robot moves **FORWARD** when given forward command ✓
- RViz visualization shows robot moving **BACKWARD** ✗
- IMU arrow in RViz points to **BACK** of robot instead of front ✗
- When setting goal in front of robot, robot moves forward correctly but RViz shows backward motion

## Root Cause

The 3D mesh file (`scout_mini_base_link2.dae`) was modeled with its "front" facing in the **negative X direction** relative to the ROS convention where forward is **positive X**.

### ROS Convention:
- **X-axis**: Forward (positive X = front)
- **Y-axis**: Left (positive Y = left side)
- **Z-axis**: Up (positive Z = up)

### Mesh Model:
- The original CAD/mesh was created with the "front" of the robot facing the **-X direction**
- This is common in different CAD software where coordinate systems vary

## Solution

Apply a **180° rotation around the Z-axis (yaw)** to flip the mesh so its front aligns with positive X.

### URDF Change:
```xml
<!-- BEFORE (Incorrect - showed reversed) -->
<origin xyz="0 0 0" rpy="0 0 0"/>

<!-- AFTER (Correct - matches reality) -->
<origin xyz="0 0 0" rpy="0 0 ${M_PI}"/>
```

Where `${M_PI}` = 3.14159 radians = 180°

### What This Does:
1. Rotates the visual mesh 180° around Z-axis (yaw)
2. Does NOT affect the robot's actual kinematics or control
3. Only changes how the mesh is displayed in RViz
4. Collision geometry also rotated to match visual

## Verification

### Before Fix:
```
Real Robot:  →→→ (moves forward)
RViz Shows:  ←←← (shows backward)  ✗ WRONG
```

### After Fix:
```
Real Robot:  →→→ (moves forward)
RViz Shows:  →→→ (shows forward)   ✓ CORRECT
```

### Test Commands:
```bash
# 1. Launch robot visualization
ros2 launch agilex_scout scout_robot_lidar.launch.py

# 2. Command forward motion
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"

# Expected: Robot model in RViz moves in +X direction (forward)
```

## Technical Details

### Rotation Matrices
A 180° yaw rotation (around Z-axis) transforms coordinates as:
```
[x']   [-1  0  0] [x]
[y'] = [ 0 -1  0] [y]
[z']   [ 0  0  1] [z]
```

This effectively:
- Flips X: front becomes back, back becomes front
- Flips Y: left becomes right, right becomes left
- Keeps Z: up stays up, down stays down

### Why Not Other Rotations?

- **Roll (rotation around X)**: Would tilt the robot sideways - incorrect
- **Pitch (rotation around Y)**: Would tilt the robot forward/backward - incorrect
- **Yaw (rotation around Z)**: Rotates horizontally - THIS IS CORRECT ✓

### Frame Hierarchy
```
odom (world frame)
  └─ base_footprint (ground contact)
      └─ base_link (robot center)
          └─ mobile_robot_base_link (mesh parent)
              ├─ Visual mesh (rotated 180° yaw)
              ├─ Collision mesh (rotated 180° yaw)
              └─ Wheels, sensors, etc.
```

The rotation is applied at the `mobile_robot_base_link` visual/collision level, so:
- All child frames (wheels, sensors) are NOT affected
- Only the visual representation is rotated
- Kinematics calculations remain unchanged

## Why This Issue Occurred

1. **Mesh Origin**: The 3D artist/CAD designer modeled the robot facing their preferred direction
2. **Export Convention**: When exported to .dae format, that orientation was preserved
3. **ROS Integration**: ROS expects forward = +X, but mesh had forward = -X
4. **Simulation vs Reality**: 
   - In simulation (Gazebo), orientation might have been compensated elsewhere
   - On real robot, this compensation was missing or different

## Alternative Approaches (Not Used)

### Approach 1: Rotate base_link frame ✗
```xml
<joint name="base_link_to_mobile_robot" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 3.14159" />  <!-- Rotate the frame -->
</joint>
```
**Problem**: This would affect ALL children including sensors and wheels - wrong!

### Approach 2: Edit mesh file ✗
- Re-export the .dae file with corrected orientation
**Problem**: Requires source CAD files and re-export - not always available

### Approach 3: Rotate mesh in URDF ✓ (CHOSEN)
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 ${M_PI}"/>  <!-- Rotate just the visual -->
</visual>
```
**Advantage**: 
- Minimal change
- No source files needed
- Clear and documented
- Easy to adjust if needed

## Impact on Other Components

### ✓ Not Affected:
- Odometry calculations (uses base_link frame)
- IMU orientation (separate frame)
- Wheel kinematics (calculated from joint positions)
- LiDAR data (separate frame)
- Path planning (uses base_link frame)
- Controller (uses base_link frame)

### ✓ Affected (Correctly):
- RViz visualization (now matches reality)
- Collision detection visualization (matches visual)
- Mesh appearance in rviz2

## Related Files

- **URDF**: `src/scout_lidar_imu/agilex_scout/urdf/mobile_robot/scout_mini.urdf.xacro`
- **Main URDF**: `src/scout_lidar_imu/agilex_scout/urdf/robot.urdf.xacro` (includes scout_mini)
- **Launch**: `src/scout_lidar_imu/agilex_scout/launch/scout_robot_lidar.launch.py`

## References

- ROS REP 103: Standard Units of Measure and Coordinate Conventions
  - Forward: +X, Left: +Y, Up: +Z
- ROS REP 105: Coordinate Frames for Mobile Platforms
  - base_link: rigidly attached to the mobile robot base
- URDF specification: http://wiki.ros.org/urdf/XML/joint#Elements

## Troubleshooting

### If RViz still shows reversed after update:

1. **Clear RViz cache**:
   ```bash
   rm -rf ~/.rviz2
   ```

2. **Restart robot_state_publisher**:
   ```bash
   ros2 lifecycle set /robot_state_publisher shutdown
   ros2 launch agilex_scout scout_robot_lidar.launch.py
   ```

3. **Verify URDF is loaded correctly**:
   ```bash
   ros2 param get /robot_state_publisher robot_description | grep "rpy="
   # Should show rpy="0 0 3.14159" or rpy="0 0 ${M_PI}"
   ```

4. **Check if you're using simulation or real robot**:
   - The fix applies when `simulation:=false`
   - For Gazebo simulation, different configuration might be used

### If IMU arrow still points backward:

The IMU frame is separate from the base mesh. Check:
```bash
ros2 run tf2_ros tf2_echo base_link imu_link
# Should show transform with yaw offset of -0.94 rad
```

## Summary

✅ **Simple Fix**: 180° yaw rotation of base mesh  
✅ **Minimal Impact**: Only affects visualization  
✅ **Well Documented**: Comments explain why  
✅ **Easy to Verify**: Single command test  
✅ **Reversible**: Change one line if needed  

The robot now shows correct orientation in RViz matching real-world behavior!
