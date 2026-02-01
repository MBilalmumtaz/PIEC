import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the AHRS device'
    )
    
    device_type_arg = DeclareLaunchArgument(
        'device_type',
        default_value='1',  # Changed from 3 to 1 (they're equivalent)
        description='Device type (0=IMU with bug, 1=AHRS with coordinate transform)'
    )
    
    ahrs_driver = Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        parameters=[{
            'if_debug_': False,
            'serial_port_': LaunchConfiguration('serial_port'),
            'serial_baud_': 921600,
            'imu_topic': '/imu',
            'imu_frame_id_': 'gyro_link',
            'mag_pose_2d_topic': '/mag_pose_2d',
            'Magnetic_topic': '/magnetic',
            'Euler_angles_topic': '/euler_angles',
            'gps_topic': '/gps/fix',
            'twist_topic': '/system_speed',
            'NED_odom_topic': '/NED_odometry',
            'device_type_': LaunchConfiguration('device_type')  # Use 1
        }],
        output="screen"
    )

    return LaunchDescription([
        serial_port_arg,
        device_type_arg,
        ahrs_driver
    ])
