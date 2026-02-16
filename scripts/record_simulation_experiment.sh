#!/bin/bash
# Automated simulation experiment recording script
# Usage: ./record_simulation_experiment.sh METHOD ENVIRONMENT NUM_TRIALS

set -e

# Check arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 METHOD ENVIRONMENT NUM_TRIALS"
    echo "  METHOD: PIEC, ASTAR, RRT, DWA"
    echo "  ENVIRONMENT: corridor, office, outdoor"
    echo "  NUM_TRIALS: Number of trials to run"
    exit 1
fi

METHOD=$1
ENVIRONMENT=$2
NUM_TRIALS=$3

# Configuration
OUTPUT_DIR="${HOME}/piec_data/simulation"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "================================"
echo "PIEC Simulation Experiment"
echo "================================"
echo "Method: ${METHOD}"
echo "Environment: ${ENVIRONMENT}"
echo "Number of trials: ${NUM_TRIALS}"
echo "Output directory: ${OUTPUT_DIR}"
echo "================================"
echo ""

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Run trials
for i in $(seq 1 ${NUM_TRIALS}); do
    TRIAL_NAME="sim_${METHOD}_${ENVIRONMENT}_${TIMESTAMP}_${i}"
    
    echo "----------------------------------------"
    echo "Running trial ${i}/${NUM_TRIALS}: ${TRIAL_NAME}"
    echo "----------------------------------------"
    
    # Launch experiment with recording enabled
    ros2 launch piec_bringup piec_complete_params.launch.py \
        enable_recording:=true \
        method:="${METHOD}" \
        environment:="${ENVIRONMENT}" \
        trial_name:="${TRIAL_NAME}" \
        output_dir:="${OUTPUT_DIR}" \
        use_sim_time:=true \
        enable_pinn:=true &
    
    LAUNCH_PID=$!
    
    # Wait for simulation to complete
    # NOTE: Adjust timeout based on environment complexity:
    #   - Simple corridor: 60-90 seconds
    #   - Complex warehouse: 120-180 seconds
    #   - Large outdoor: 180-300 seconds
    # TODO: Implement goal completion monitoring via /piec/goal_reached topic
    sleep 120  # 2 minutes per trial
    
    # Shutdown launch file
    echo "Stopping trial ${i}..."
    kill ${LAUNCH_PID} 2>/dev/null || true
    wait ${LAUNCH_PID} 2>/dev/null || true
    
    # Wait before next trial
    if [ ${i} -lt ${NUM_TRIALS} ]; then
        echo "Waiting 10 seconds before next trial..."
        sleep 10
    fi
done

echo ""
echo "================================"
echo "Experiment complete!"
echo "================================"
echo "Recorded ${NUM_TRIALS} trials"
echo "Data saved to: ${OUTPUT_DIR}"
echo ""
echo "To aggregate data, run:"
echo "  python3 scripts/aggregate_experiment_data.py --data-dir ${OUTPUT_DIR}"
echo "================================"
