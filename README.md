# First, just hardware (to verify everything works)
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agilex_scout scout_robot_lidar.launch.py

# Then, full PIEC stack
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piec_bringup piec_real_robot.launch.py

#Controller
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run piec_controller controller_node 


#Path optimizer
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run piec_path_optimizer complete_path_optimizer



source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run  piec_pinn_surrogate pinn_service
#ukf
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run piec_ukf_localization ukf_node



#lidar setting:
# Bring interface up
sudo ip link set enp0s31f6 up

# Assign IP address (you said earlier it was 192.168.1.102)
sudo ip addr add 192.168.1.102/24 dev enp0s31f6

# Verify
ip addr show enp0s31f6
ip link show enp0s31f6


#robot base
 sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 500000 2>/dev/null
sudo ip link set can0 up 2>/dev/null
echo "8. Checking CAN status..."
ip -details link show can0

#IMU (LPMS IG1 CAN)
# Kill any existing ROS nodes
ros2 daemon stop
ros2 daemon start

# Check if device exists
ls -la /dev/ttyUSB*

# Check if any process is using the serial port
sudo lsof /dev/ttyUSB0
sudo fuser -v /dev/ttyUSB0

# Kill any processes using the port
sudo pkill -9 screen
sudo pkill -9 minicom
sudo fuser -k /dev/ttyUSB0

# LPMS IG1 CAN IMU
# Start the OpenZen IMU driver
ros2 run openzen_driver openzen_node --ros-args --remap __ns:=/openzen

# Check IMU data
ros2 topic echo /openzen/data --no-arr

# Check magnetometer data
ros2 topic echo /openzen/mag --no-arr

# Check publishing rate
ros2 topic hz /openzen/data
# Set CAN parameters
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Test CAN bus
timeout 5 candump can0 &

# Send a test message (if you have can-utils)
cansend can0 123#DEADBEEF

# Check for any CAN traffic
candump can0 -L -n 10


# Check if CAN interface exists
ls /sys/class/net/ | grep can

# Check for CAN hardware
dmesg | grep -i can
lspci | grep -i can
lsusb | grep -i can

# Check kernel modules
lsmod | grep can


# Set CAN parameters
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Test CAN bus
timeout 5 candump can0 &

# Send a test message (if you have can-utils)
cansend can0 123#DEADBEEF

# Check for any CAN traffic
candump can0 -L -n 10
