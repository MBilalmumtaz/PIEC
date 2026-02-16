#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    launch = LaunchDescription()

    declare_goal_topic = DeclareLaunchArgument(
        'goal_topic', default_value='/goal_pose'
    )
    launch.add_action(declare_goal_topic)

    # Include AgileX simulation WITH RViz enabled
    try:
        agilex_share = get_package_share_directory('agilex_scout')
        agilex_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agilex_share, 'launch', 'simulate_control_gazebo.launch.py')
            ),
            launch_arguments={
                'lidar_type': '3d', 
                'rviz': 'true',  # THIS IS THE KEY: Enable RViz
                'world_name': 'aws_warehouse',  # Optional: specify world
                'use_sim_time': 'true'  # Important for simulation
            }.items()
        )
        launch.add_action(agilex_launch)
    except Exception as e:
        print(f"Could not load AgileX simulation: {e}")
        # Continue without simulation

    # UKF NODE
    launch.add_action(
        Node(
            package='piec_ukf_localization',
            executable='ukf_node',
            name='ukf_localization',
            output='screen',
            parameters=[{
                'odom_topic': '/odometry',  # Changed to match your bridge
                'imu_topic': '/imu/data'
            }]
        )
    )

    # PINN Service
    launch.add_action(
        Node(
            package='piec_pinn_surrogate',
            executable='pinn_service',
            name='pinn_service',
            output='screen'
        )
    )

    # NSGA-II Optimizer
    launch.add_action(
        Node(
            package='piec_path_optimizer',
            executable='path_optimizer',
            name='nsga_optimizer',
            output='screen',
            parameters=[{'goal_topic': LaunchConfiguration('goal_topic')}]
        )
    )

    # CONTROLLER
    launch.add_action(
        Node(
            package='piec_controller',
            executable='controller_node',
            name='dynamic_dwa_controller',
            output='screen'
        )
    )

    # Laser Bridge (if needed)
    launch.add_action(
        Node(
            package='piec_controller',
            executable='laser_bridge',
            name='laser_bridge',
            output='screen'
        )
    )

    return launch
