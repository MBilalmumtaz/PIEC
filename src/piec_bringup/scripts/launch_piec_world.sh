#!/bin/bash
# launch_piec_world.sh - Helper script for launching PIEC simulation with different worlds
# Usage: ./launch_piec_world.sh [world_name] [additional_args...]

# Default world
WORLD_NAME="${1:-aws_warehouse}"

# Valid world choices
VALID_WORLDS=("aws_warehouse" "office" "agriculture" "inspection" "obstacle" "orchard" "empty" "race" "accessories")

# Function to check if world name is valid
is_valid_world() {
    local world=$1
    for valid in "${VALID_WORLDS[@]}"; do
        if [[ "$valid" == "$world" ]]; then
            return 0
        fi
    done
    return 1
}

# Display usage information
show_usage() {
    echo "Usage: $0 [world_name] [additional_args...]"
    echo ""
    echo "Available worlds:"
    echo "  aws_warehouse  - AWS RoboMaker small warehouse (default)"
    echo "  office         - Indoor office environment"
    echo "  agriculture    - Agricultural field with crops"
    echo "  inspection     - Industrial inspection site"
    echo "  obstacle       - Obstacle course"
    echo "  orchard        - Orchard with trees"
    echo "  empty          - Empty world for testing"
    echo "  race           - Racing track"
    echo "  accessories    - Empty world with accessories models"
    echo ""
    echo "Examples:"
    echo "  $0 office"
    echo "  $0 agriculture enable_pinn:=false"
    echo "  $0 obstacle use_rviz:=true"
    echo ""
}

# Check for help flag
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    show_usage
    exit 0
fi

# Validate world name
if ! is_valid_world "$WORLD_NAME"; then
    echo "Error: Invalid world name '$WORLD_NAME'"
    echo ""
    show_usage
    exit 1
fi

# Source ROS2 workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: Could not find ROS2 setup.bash"
    exit 1
fi

# Remove the world name from arguments
shift

# Display what we're launching
echo "============================================"
echo "  PIEC Simulation Launch"
echo "============================================"
echo "World: $WORLD_NAME"
echo "Command: ros2 launch piec_bringup piec_complete_params.launch.py world:=$WORLD_NAME" "$@"
echo "============================================"
echo ""

# Launch! Use exec to replace the shell process
exec ros2 launch piec_bringup piec_complete_params.launch.py "world:=$WORLD_NAME" "$@"
