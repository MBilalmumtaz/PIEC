from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. AHRS DRIVER NODE - Publishes raw IMU data from serial port
    ahrs_driver_node = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver',  # Executable from ahrs_driver.cpp
        name='ahrs_driver',
        output='screen',
        parameters=[{
            'port_name': '/dev/ttyUSB0',   # Adjust to your IMU's port (check with ls /dev/tty*)
            'port_baudrate': 921600,       # Default for Wheeltec N100
            'frame_id': 'imu_link',        # Frame for the IMU data
        }]
    )

    # 2. IMU TF NODE - Handles transform broadcasting (from imu_tf.cpp)
    # This node likely subscribes to IMU data and publishes a transform.
    # We start it with default parameters.
    imu_tf_node = Node(
        package='fdilink_ahrs',
        executable='imu_tf',  # Executable from imu_tf.cpp
        name='imu_tf',
        output='screen',
        parameters=[{
            'imu_topic': '/imu/data',      # Topic from ahrs_driver (check actual topic)
            'world_frame_id': 'world',     # Parent frame for transform
            'imu_frame_id': 'imu_link',    # Child frame for transform
            'position_x': 0.0,             # Translation offset (adjust based on your robot)
            'position_y': 0.0,
            'position_z': 0.1,             # IMU height above base_link
        }]
    )

    return LaunchDescription([
        ahrs_driver_node,
        imu_tf_node,
    ])
