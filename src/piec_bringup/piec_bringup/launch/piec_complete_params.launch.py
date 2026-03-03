#!/usr/bin/env python3
"""
Complete launch file for PIEC stack with PINN - INCLUDES SIMULATION
Includes aws_warehouse, agriculture, and office_cpr worlds
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # ------------------------------------------------------------------
    # 2. Launch arguments - THREE WORLDS
    # ------------------------------------------------------------------
    ld.add_action(DeclareLaunchArgument("use_sim", default_value="true"))
    ld.add_action(DeclareLaunchArgument(
        "world", 
        default_value="aws_warehouse",
        choices=["aws_warehouse", "agriculture", "office_cpr", "baylands_terrain",
                 "campus", "grasspatch", "indoor_lightmap", "simple_baylands"],
        description="Gazebo world to load for simulation (aws_warehouse, agriculture, or office_cpror baylands_terrain)"
    ))
    ld.add_action(DeclareLaunchArgument("enable_pinn", default_value="true"))
    ld.add_action(DeclareLaunchArgument("robot_type", default_value="scout_mini"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))
    
    # Data recording arguments
    ld.add_action(DeclareLaunchArgument("enable_recording", default_value="false"))
    ld.add_action(DeclareLaunchArgument("method", default_value="PIEC"))
    ld.add_action(DeclareLaunchArgument("environment", default_value="aws_warehouse"))
    ld.add_action(DeclareLaunchArgument("trial_name", default_value="trial_001"))
    ld.add_action(DeclareLaunchArgument("output_dir", default_value="~/piec_data"))
    
    use_sim = LaunchConfiguration("use_sim")
    world = LaunchConfiguration("world")
    enable_pinn = LaunchConfiguration("enable_pinn")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Data recording configurations
    enable_recording = LaunchConfiguration("enable_recording")
    method = LaunchConfiguration("method")
    environment = LaunchConfiguration("environment")
    trial_name = LaunchConfiguration("trial_name")
    output_dir = LaunchConfiguration("output_dir")
    
    # ------------------------------------------------------------------
    # 3. GLOBAL PARAMETER - CRITICAL
    # ------------------------------------------------------------------
    ld.add_action(SetParameter(name="use_sim_time", value=use_sim_time))
    
    # ------------------------------------------------------------------
    # 4. LAUNCH SIMULATION
    # ------------------------------------------------------------------
    # First, find the AgileX Scout package
    try:
        agilex_share = FindPackageShare("agilex_scout").find("agilex_scout")
        
        # Launch the simulation
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(agilex_share, "launch", "simulate_control_gazebo.launch.py")
                ),
                launch_arguments={
                    "lidar_type": "3d",
                    "rviz": use_rviz,
                    "world_name": world,
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        )
        ld.add_action(SetParameter(name="use_sim_time", value="true"))
        
    except Exception as e:
        print(f"Could not load AgileX simulation: {e}")
        # Fallback: Start Gazebo manually
        ld.add_action(
            ExecuteProcess(
                cmd=['ign', 'gazebo', '-r', 'aws_warehouse.sdf'],
                output='screen',
                condition=IfCondition(use_sim)
            )
        )
    
    # ------------------------------------------------------------------
    # 5. Wait a moment for simulation to start
    # ------------------------------------------------------------------
    ld.add_action(
        ExecuteProcess(
            cmd=['sleep', '3'],
            output='screen',
            condition=IfCondition(use_sim)
        )
    )
    
    # ------------------------------------------------------------------
    # 6. Scout Mini parameters
    # ------------------------------------------------------------------
    scout_mini_params = {
        "robot_radius": 0.3,
        "wheel_radius": 0.16,
        "wheel_separation": 0.4563536,
        "wheel_base": 0.3132556,
        "max_linear_vel": 1.2,
        "max_angular_vel": 0.8,
    }
    
    # ------------------------------------------------------------------
    # 7. PIEC STACK NODES
    # ------------------------------------------------------------------
    
    # 7.1 PINN Service
    ld.add_action(
        Node(
            package="piec_pinn_surrogate",
            executable="pinn_service",
            name="pinn_service",
            output="screen",
            condition=IfCondition(enable_pinn),
            parameters=[{
                "model_path": "/home/amjad/PIEC_2d/src/piec_pinn_surrogate/models/pinn_physics.pt",
                "default_clearance": 2.0,
                "default_obstacle_density": 0.0,
                "use_laser_updates": True,
                "use_sim_time": use_sim_time,
            }],
        )
    )
    
    # 7.2 UKF Localization
    ld.add_action(
        Node(
            package="piec_ukf_localization",
            executable="ukf_node",
            name="ukf_localization",
            output="screen",
            parameters=[{
                **scout_mini_params,
                "use_sim_time": use_sim_time,
                "odom_topic": "/odometry",
                "imu_topic": "/imu/data",
                "publish_topic": "/ukf/odom",
            }],
        )
    )
    
    
    # 7.3 Laser Bridge
    ld.add_action(
        Node(
            package="piec_controller",
            executable="laser_bridge",
            name="laser_bridge",
            output="screen",
            parameters=[{
                "min_range": 0.1,
                "max_range": 8.0,
                "range_filter_window": 3,
                "median_filter": True,
                "input_topic": "/laser_scan",
                "output_fixed_topic": "/scan_fixed",
                "output_scan_topic": "/scan",
                "use_sim_time": use_sim_time,
            }],
        )
    )
    
    # 7.4 PINN-INTEGRATED PATH OPTIMIZER
    ld.add_action(
        Node(
            package="piec_path_optimizer",
            executable="complete_path_optimizer",
            name="pinn_path_optimizer",
            output="screen",
            parameters=[{
                **scout_mini_params,
                "use_sim_time": use_sim_time,
                "use_pinn_predictions": True,
                "pinn_service_name": "/evaluate_trajectory",
                "population_size": 8,
                "generations": 4,
                "optimization_timeout": 2.0,
                "planning_rate": 1.5,
                "waypoint_count": 6,
                "max_curvature": 1.5,
                "path_smoothing": True,
                "max_pinn_calls_per_generation": 6,
                "objective_weights": [0.15, 0.1, 0.15, 0.1, 0.1, 0.25, 0.15],
                "debug_mode": True,
                "log_fitness": True,
                "goal_topic": "/goal_pose",
            }],
        )
    )
    
    # 7.5 Controller
    ld.add_action(
        Node(
            package="piec_controller",
            executable="controller_node",
            name="controller",
            output="screen",
            parameters=[{
                **scout_mini_params,
                "use_sim_time": use_sim_time,
                "max_linear_vel": 1.2,
                "min_linear_vel": 0.3,
                "max_angular_vel": 0.8,
                "emergency_stop_distance": 0.2,
                "slow_down_distance": 0.5,
                "safe_distance": 0.6,
                "waypoint_tolerance": 0.3,
                "lookahead_distance": 0.8,
                "control_frequency": 15.0,
                "debug_mode": True,
                "require_explicit_goal": True,
                "path_topic": "/piec/path",
                "cmd_vel_topic": "/cmd_vel",
            }],
        )
    )
    
    # 7.6 Emergency Stop
    ld.add_action(
        Node(
            package="piec_controller",
            executable="emergency_stop",
            name="emergency_stop",
            output="screen",
            parameters=[{
                "stop_distance": 0.2,
                "slow_distance": 0.4,
                "enable_emergency_stop": True,
                "use_sim_time": use_sim_time,
                "scan_topic": "/scan_fixed",
                "cmd_vel_topic": "/cmd_vel",
                "output_topic": "/cmd_vel_safe",
            }],
        )
    )
    
    # 7.7 Metrics Collector
    ld.add_action(
        Node(
            package="piec_validation",
            executable="metrics_collector",
            name="metrics_collector",
            output="screen",
            parameters=[{
                "output_dir": "~/piec_metrics",
                "sampling_rate": 5.0,
                "use_sim_time": use_sim,
            }],
        )
    )
    
    # 7.8 Dynamics Node
    ld.add_action(
        Node(
            package="piec_dynamics",
            executable="dynamics_node",
            name="piec_dynamics",
            output="screen",
            parameters=[{
                "output_dir": "~/piec_dynamics",
                "use_sim_time": use_sim,
            }],
        )
    )
    
    # 7.9 Experiment Recorder (optional)
    ld.add_action(
        Node(
            package="piec_bringup",
            executable="experiment_recorder",
            name="experiment_recorder",
            output="screen",
            condition=IfCondition(enable_recording),
            parameters=[{
                "method": method,
                "environment": environment,
                "trial_name": trial_name,
                "output_dir": output_dir,
                "record_rate": 20.0,
                "use_sim_time": use_sim_time,
            }],
        )
    )
    
    return ld
