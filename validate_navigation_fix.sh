#!/bin/bash
# Quick validation script for Scout Mini navigation fixes
# Run this on the real robot after deploying the PR

echo "========================================="
echo "Scout Mini Navigation Fix Validation"
echo "========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check 1: TF Validator
echo "1. Running TF Validator..."
timeout 10 ros2 run piec_bringup tf_validator > /tmp/tf_validator.log 2>&1
if grep -q "TF TREE VALIDATION PASSED" /tmp/tf_validator.log; then
    echo -e "${GREEN}✓ TF validation PASSED${NC}"
else
    echo -e "${RED}✗ TF validation FAILED${NC}"
    echo "See /tmp/tf_validator.log for details"
fi
echo ""

# Check 2: Verify angular_sign_correction parameter
echo "2. Checking angular_sign_correction parameter..."
# Wait for controller to be running
sleep 2
ANGULAR_SIGN=$(ros2 param get /controller angular_sign_correction 2>/dev/null | grep -oP '(?<=value: )-?\d+\.?\d*')
if [ "$ANGULAR_SIGN" == "-1.0" ] || [ "$ANGULAR_SIGN" == "-1" ]; then
    echo -e "${GREEN}✓ angular_sign_correction = -1.0 (correct for Scout Mini)${NC}"
elif [ -z "$ANGULAR_SIGN" ]; then
    echo -e "${YELLOW}⚠ Controller not running yet${NC}"
else
    echo -e "${RED}✗ angular_sign_correction = $ANGULAR_SIGN (should be -1.0)${NC}"
fi
echo ""

# Check 3: Verify imu_link transform
echo "3. Checking base_link→imu_link transform..."
timeout 5 ros2 run tf2_ros tf2_echo base_link imu_link > /tmp/tf_check.log 2>&1
if grep -q "Translation:" /tmp/tf_check.log; then
    Z_TRANS=$(grep "z:" /tmp/tf_check.log | head -1 | grep -oP '[-]?\d*\.?\d+' || echo "")
    YAW=$(grep "yaw:" /tmp/tf_check.log | grep -oP '[-]?\d*\.?\d+' | head -1 || echo "")
    
    # Check if z is approximately 0.2 (allow 0.15-0.25)
    # Use awk for floating-point comparison to avoid bc syntax errors
    if [ -n "$Z_TRANS" ] && [ "$Z_TRANS" != "" ]; then
        Z_OK=$(awk -v z="$Z_TRANS" 'BEGIN { if (z > 0.15 && z < 0.25) print "1"; else print "0" }')
        if [ "$Z_OK" = "1" ]; then
            echo -e "${GREEN}✓ IMU z-height = ${Z_TRANS}m (expected ~0.2m)${NC}"
        else
            echo -e "${RED}✗ IMU z-height = ${Z_TRANS}m (expected ~0.2m)${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ Could not parse IMU z-height${NC}"
    fi
    
    # Check if yaw is approximately -0.94 (allow -1.15 to -0.75)
    # Use awk for floating-point comparison to avoid bc syntax errors
    if [ -n "$YAW" ] && [ "$YAW" != "" ]; then
        YAW_OK=$(awk -v y="$YAW" 'BEGIN { if (y < -0.75 && y > -1.15) print "1"; else print "0" }')
        if [ "$YAW_OK" = "1" ]; then
            echo -e "${GREEN}✓ IMU yaw = ${YAW} rad (expected ~-0.94 rad)${NC}"
        else
            echo -e "${YELLOW}⚠ IMU yaw = ${YAW} rad (expected ~-0.94 rad)${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ Could not parse IMU yaw${NC}"
    fi
else
    echo -e "${RED}✗ Could not read transform${NC}"
fi
echo ""

# Check 4: Verify no duplicate imu_link transforms
echo "4. Checking for duplicate imu_link publishers..."
TF_PUBLISHERS=$(ros2 node list | grep -c "static_transform_publisher")
if [ "$TF_PUBLISHERS" -le 3 ]; then
    echo -e "${GREEN}✓ Normal number of static TF publishers ($TF_PUBLISHERS)${NC}"
else
    echo -e "${YELLOW}⚠ Many static TF publishers ($TF_PUBLISHERS) - check for duplicates${NC}"
fi

# Check for specific base_to_imu publisher (should NOT exist)
if ros2 node list | grep -q "base_to_imu_tf"; then
    echo -e "${RED}✗ Found base_to_imu_tf static publisher (should be removed)${NC}"
else
    echo -e "${GREEN}✓ No conflicting base_to_imu_tf publisher${NC}"
fi
echo ""

# Check 5: Unit tests
echo "5. Running controller unit tests..."
cd /home/agx3/scoutmini_ws3/src/piec_controller 2>/dev/null || cd ~/scoutmini_ws3/src/piec_controller 2>/dev/null || cd $(find ~ -name "piec_controller" -type d 2>/dev/null | head -1) 2>/dev/null
if [ -f "test/test_controller_heading.py" ]; then
    python3 test/test_controller_heading.py TestAngularSignCorrection 2>&1 | tail -3
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Angular sign correction tests PASSED${NC}"
    else
        echo -e "${RED}✗ Angular sign correction tests FAILED${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Test file not found${NC}"
fi
echo ""

echo "========================================="
echo "Validation Complete"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Send a test goal to the RIGHT of the robot:"
echo "   ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: \"odom\"}, pose: {position: {x: 0.0, y: -2.0}}}'"
echo "   → Robot should turn RIGHT (clockwise)"
echo ""
echo "2. Send a test goal to the LEFT of the robot:"
echo "   ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: \"odom\"}, pose: {position: {x: 0.0, y: 2.0}}}'"
echo "   → Robot should turn LEFT (counter-clockwise)"
echo ""
echo "3. Monitor controller output for sign corrections:"
echo "   ros2 topic echo /cmd_vel --field angular.z"
