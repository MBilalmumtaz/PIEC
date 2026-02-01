#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Set ROS domain ID to avoid conflicts
    ld.add_action(SetEnvironmentVariable('ROS_DOMAIN_ID', '0'))
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use simulation time'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='aws_warehouse',
        description='Gazebo world name'
    ))
    
   # ld.add_action(DeclareLaunchArgument(
  #      'config_dir', 
    #    default_value=PathJoinSubstitution([
    #        FindPackageShare('piec_bringup'), 'config'
    #    ]),
     #   description='Directory containing parameter files'
   # ))
    
    ld.add_action(DeclareLaunchArgument(
        'enable_pinn', default_value='false',
        description='Enable PINN energy/stability predictions'
    ))
    
    # Get launch arguments
    use_sim = LaunchConfiguration('use_sim')
    world = LaunchConfiguration('world')
  #  config_dir = LaunchConfiguration('config_dir')
    enable_pinn = LaunchConfiguration('enable_pinn')
    
    # Include AgileX simulation
    try:
        agilex_share = FindPackageShare('agilex_scout').find('agilex_scout')
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agilex_share, 'launch', 'simulate_control_gazebo.launch.py')
            ),
            launch_arguments={
                'lidar_type': '3d',
                'rviz': 'true',
                'world_name': world,
                'use_sim_time': use_sim
            }.items()
        ))
    except Exception as e:
        print(f"Could not load AgileX simulation: {e}")
    
    # Set use_sim_time parameter for all nodes
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim))
    
    # Load main parameters
    #main_params = PathJoinSubstitution([config_dir, 'params.yaml'])
    
    # Laser Bridge (MUST BE FIRST) - WITH UPDATED PARAMETERS
    ld.add_action(Node(
        package='piec_controller',
        executable='laser_bridge',
        name='laser_bridge',
        output='screen',
        parameters=[{
            'min_range': 0.1,
            'max_range': 10.0,
            'range_filter_window': 3,
            'median_filter': True,
            'outlier_threshold': 0.5,
            'publish_raw': False,
            'input_topic': '/laser_scan',  # From simulation
            'output_fixed_topic': '/scan_fixed',  # Processed scan for planners
            'output_scan_topic': '/scan',  # Processed scan for other nodes
            'use_sim_time': use_sim
        }]
    ))
    
    # UKF Localization Node with parameters
   # ukf_params = PathJoinSubstitution([config_dir, 'ukf_params.yaml'])
    ld.add_action(Node(
        package='piec_ukf_localization',
        executable='ukf_node',
        name='ukf_localization',
        output='screen',
        #parameters=[ukf_params, {'use_sim_time': use_sim}]
    ))
    
    # Path Optimizer Node with parameters - WITH CRITICAL FIXES
    #optimizer_params = PathJoinSubstitution([config_dir, 'optimizer_params.yaml'])
    ld.add_action(Node(
        package='piec_path_optimizer',
        executable='complete_path_optimizer',
        name='path_optimizer',
        output='screen',
        parameters=[ {
            'use_sim_time': use_sim,
            # Override critical parameters for testing
            'population_size': 10,  # Reduced for faster optimization
            'generations': 5,  # Reduced for faster optimization
            'optimization_timeout': 2.0,
            'planning_rate': 1.0,
            'waypoint_count': 3,  # Reduced for simpler paths
            'debug_mode': True,
            'use_multithreading': True,
            'max_workers': 2,
            'path_smoothing': True,
            'require_explicit_goal': False,  # For testing - allow paths without explicit goal
        }]
    ))
    
    # Controller Node with DWA parameters - WITH CRITICAL FIXES
   # controller_params = PathJoinSubstitution([config_dir, 'controller_params.yaml'])
   # dwa_params = PathJoinSubstitution([config_dir, 'dwa_params.yaml'])
    ld.add_action(Node(
        package='piec_controller',
        executable='controller_node',
        name='controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim,
                # Override critical parameters for testing
                'max_linear_vel': 1.5,  # Reduced for safety
                'min_linear_vel': 0.5,  # Lower minimum
                'max_angular_vel': 1.0,  # Reduced for safety
                'emergency_stop_distance': 0.25,  # Reduced
                'slow_down_distance': 0.6,  # Reduced
                'safe_distance': 0.8,  # Reduced
                'waypoint_tolerance': 0.4,  # Increased
                'lookahead_distance': 1.0,  # Reduced
                'control_frequency': 10.0,  # Reduced
                'debug_mode': True,
                'require_explicit_goal': False,  # For testing
                'stuck_threshold': 0.5,  # Reduced from 2.0
                'stuck_time': 15.0,  # Increased from 10.0
                'enable_obstacle_memory': False,  # Disabled for testing
                # DWA parameters
                'dwa_max_v': 1.5,
                'dwa_min_v': 0.5,
                'dwa_max_w': 0.8,
                'dwa_sim_time': 1.5,
                'dwa_clearance_weight': 2.0,
            }
        ]
    ))
    
    # Emergency Stop Node - WITH UPDATED PARAMETERS
    ld.add_action(Node(
        package='piec_controller',
        executable='emergency_stop',
        name='emergency_stop',
        output='screen',
        parameters=[{
            'stop_distance': 0.25,  # Reduced from 0.35
            'slow_distance': 0.5,
            'enable_emergency_stop': True,
            'use_sim_time': use_sim
        }]
    ))
    
    # PINN Service (conditionally enabled)
    ld.add_action(Node(
        package='piec_pinn_surrogate',
        executable='pinn_service',
        name='pinn_service',
        output='screen',
        condition=IfCondition(enable_pinn),
        parameters=[{
            'model_path': PathJoinSubstitution([
                FindPackageShare('piec_pinn_surrogate'),
                'models',
                'pinn_physics.pt'
            ]),
            'use_sim_time': use_sim,
            'default_clearance': 2.0,
            'default_obstacle_density': 0.0,
            'terrain_map_resolution': 0.1,
            'use_laser_updates': False,
        }]
    ))
    
    # Metrics Collector
    ld.add_action(Node(
        package='piec_validation',
        executable='metrics_collector',
        name='metrics_collector',
        output='screen',
        parameters=[{
            'output_dir': '~/piec_metrics',
            'sampling_rate': 5.0,
            'use_sim_time': use_sim
        }]
    ))
    
    # Optional: Add test goal publisher for debugging
    ld.add_action(DeclareLaunchArgument(
        'enable_test_goals', default_value='false',
        description='Enable test goal publisher'
    ))
    
    from launch.conditions import LaunchConfigurationEquals
    ld.add_action(Node(
        package='piec_controller',
        executable='send_goal',
        name='test_goal_publisher',
        output='screen',
        condition=LaunchConfigurationEquals('enable_test_goals', 'true'),
        parameters=[{
            'use_sim_time': use_sim
        }]
    ))
    
    return ld
