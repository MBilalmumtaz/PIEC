#!/bin/bash
# Interactive real-world experiment recording script
# Usage: ./record_real_experiment.sh METHOD ENVIRONMENT

set -e

# Check arguments
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 METHOD ENVIRONMENT"
    echo "  METHOD: PIEC, ASTAR, RRT, DWA"
    echo "  ENVIRONMENT: corridor, office, outdoor, etc."
    exit 1
fi

METHOD=$1
ENVIRONMENT=$2

# Configuration
OUTPUT_DIR="${HOME}/piec_data/real_world"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "================================"
echo "PIEC Real-World Experiment"
echo "================================"
echo "Method: ${METHOD}"
echo "Environment: ${ENVIRONMENT}"
echo "Output directory: ${OUTPUT_DIR}"
echo "================================"
echo ""

# Create output directory
mkdir -p "${OUTPUT_DIR}"

echo "INSTRUCTIONS:"
echo "1. Press ENTER to start each trial"
echo "2. The robot will start recording data"
echo "3. Send goal poses as needed"
echo "4. When trial is complete, press Ctrl+C"
echo "5. Type 'done' and press ENTER when all trials are complete"
echo ""

TRIAL_NUM=1

while true; do
    echo "----------------------------------------"
    echo "Ready for trial ${TRIAL_NUM}"
    echo "Press ENTER to start (or type 'done' to finish)"
    echo -n "> "
    read -r input
    
    if [ "$input" = "done" ]; then
        echo "Finishing experiments..."
        break
    fi
    
    TRIAL_NAME="real_${METHOD}_${ENVIRONMENT}_${TIMESTAMP}_${TRIAL_NUM}"
    
    echo "Starting trial: ${TRIAL_NAME}"
    echo "Recording data... (Press Ctrl+C when complete)"
    
    # Launch real robot with recording enabled
    ros2 launch piec_bringup piec_real_robot.launch.py \
        enable_recording:=true \
        method:="${METHOD}" \
        environment:="${ENVIRONMENT}" \
        trial_name:="${TRIAL_NAME}" \
        output_dir:="${OUTPUT_DIR}" \
        enable_piecnodes:=true \
        enable_pinn:=true || true
    
    echo "Trial ${TRIAL_NUM} complete!"
    echo ""
    
    TRIAL_NUM=$((TRIAL_NUM + 1))
done

echo ""
echo "================================"
echo "Experiment complete!"
echo "================================"
echo "Recorded $((TRIAL_NUM - 1)) trials"
echo "Data saved to: ${OUTPUT_DIR}"
echo ""
echo "To aggregate data, run:"
echo "  python3 scripts/aggregate_experiment_data.py --data-dir ${OUTPUT_DIR}"
echo "================================"
