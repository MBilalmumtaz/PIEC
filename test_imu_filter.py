# test_imu_filter.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_test',
            output='screen',
            parameters=[{
                'use_mag': True,
                'world_frame': 'enu',
                'fixed_frame': 'imu_link',
                'mag_bias_x': 0.000010,
                'mag_bias_y': -0.000112,
                'mag_bias_z': 0.000094,
                'mag_declination': 0.092,
                'yaw_offset': -0.94,
                'gain': 0.1,
            }],
            remappings=[
                ('/imu/data_raw', '/imu_raw'),
                ('/imu/data', '/imu'),
                ('/imu/mag', '/magnetic_field'),
            ]
        )
    ])
