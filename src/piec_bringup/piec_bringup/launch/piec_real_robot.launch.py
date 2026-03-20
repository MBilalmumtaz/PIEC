#!/usr/bin/env python3
"""
Complete launch file for PIEC stack with PINN - REAL ROBOT VERSION (FIXED)
Matches simulation configuration as closely as possible.
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    # ------------------------------------------------------------------
    # 1. Launch arguments
    # ------------------------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "enable_pinn", default_value="true"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_rviz", default_value="true"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_teleop", default_value="true"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_lidar", default_value="true"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_imu", default_value="true"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_base", default_value="true"
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_piecnodes", default_value="true"
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_recording", default_value="false"
    ))
    ld.add_action(DeclareLaunchArgument(
        "method", default_value="PIEC"
    ))
    ld.add_action(DeclareLaunchArgument(
        "environment", default_value="corridor"
    ))
    ld.add_action(DeclareLaunchArgument(
        "trial_name", default_value="real_trial_001"
    ))
    ld.add_action(DeclareLaunchArgument(
        "output_dir", default_value="~/piec_data"
    ))
    
    # Get launch configurations
    enable_pinn = LaunchConfiguration("enable_pinn")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_teleop = LaunchConfiguration("use_teleop")
    use_lidar = LaunchConfiguration("use_lidar")
    use_imu = LaunchConfiguration("use_imu")
    use_base = LaunchConfiguration("use_base")
    enable_piecnodes = LaunchConfiguration("enable_piecnodes")
    enable_recording = LaunchConfiguration("enable_recording")
    method = LaunchConfiguration("method")
    environment = LaunchConfiguration("environment")
    trial_name = LaunchConfiguration("trial_name")
    output_dir = LaunchConfiguration("output_dir")
    
    # ------------------------------------------------------------------
    # 2. Global parameter
    # ------------------------------------------------------------------
    ld.add_action(SetParameter(name="use_sim_time", value=use_sim_time))
    
    # ------------------------------------------------------------------
    # 3. Launch real robot hardware (AgileX Scout with LiDAR/IMU)
    # ------------------------------------------------------------------
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agilex_scout'),
                'launch',
                'scout_robot_lidar.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'true',      # We'll use our own RViz config later
            'use_teleop': 'false',     # We'll start teleop separately if needed
            'use_lidar': use_lidar,
            'use_imu': use_imu,
            'use_base': use_base,
        }.items()
    )
    ld.add_action(real_robot_launch)
    
    # ------------------------------------------------------------------
    # 4. Short pause for hardware initialization
    # ------------------------------------------------------------------
    ld.add_action(
        ExecuteProcess(
            cmd=['sleep', '3'],
            output='screen'
        )
    )
    
    # ------------------------------------------------------------------
    # 5. Scout Mini parameters (same as simulation, but speed limits may be reduced for safety)
    # ------------------------------------------------------------------
    scout_mini_params = {
        "robot_radius": 0.3,
        "wheel_radius": 0.16,
        "wheel_separation": 0.4563536,
        "wheel_base": 0.3132556,
        "max_linear_vel": 0.8,        # Slightly lower than simulation (1.2) for safety
        "max_angular_vel": 0.6,        # Slightly lower than simulation (0.8)
        "use_sim_time": use_sim_time,
    }
    
    # ------------------------------------------------------------------
    # 6. PIEC stack nodes (matching simulation configuration)
    # ------------------------------------------------------------------
    
    # 6.1 PINN Service
    pinn_service_node = Node(
        package="piec_pinn_surrogate",
        executable="pinn_service",
        name="pinn_service",
        output="screen",
        condition=IfCondition(enable_pinn),
        parameters=[{
            "model_path": PathJoinSubstitution([
                FindPackageShare("piec_pinn_surrogate"),
                "models",
                "pinn_physics.pt",
            ]),
            "default_clearance": 2.0,
            "default_obstacle_density": 0.0,
            "use_laser_updates": True,
            "use_sim_time": use_sim_time,
        }],
    )
    ld.add_action(pinn_service_node)
    
    # ------------------------------------------------------------------
    # 6. UKF Localization - USING RAW IMU DIRECTLY
    # ------------------------------------------------------------------
    ukf_localization_node = Node(
        package="piec_ukf_localization",
        executable="ukf_node",
        name="ukf_localization",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            **scout_mini_params,
            "odom_topic": "/odometry",
            "imu_topic": "/openzen/data",           # Use raw IMU directly
            "publish_topic": "/ukf/odom",
            
            # IMU Configuration
            "imu_yaw_offset": 0.108,                 # From your raw IMU orientation.z
            "use_imu_orientation": True,              # Use IMU's orientation
            "use_wheel_velocity": True,
            "use_wheel_odometry": True,
            
            # Robot parameters
            "wheel_radius": 0.16,
            "wheel_base": 0.313,
            "initial_yaw": 0.0,
            "debug_mode": True,
            
            # UKF tuning - optimized for raw IMU
            "Q_position": 0.05,                       # Process noise - position
            "Q_orientation": 0.01,                     # Process noise - orientation
            "R_imu_orientation": 0.005,                # IMU orientation trust
            "R_odom_position": 0.02,                    # Wheel odometry trust
            "R_pose": [0.01, 0.01, 0.005],             # Measurement noise
            "R_yawrate": [0.001],                       # Gyro noise
            
            # Frames
            "imu_frame": "imu_link",
            "base_frame": "base_link",
            "world_frame": "odom",
            
            "dt": 0.05,
            "real_robot_mode": True,
        }],
    )
    ld.add_action(ukf_localization_node)

    # 6.4 PINN‑Integrated Path Optimizer (uses /scan_processed)
    pinn_path_optimizer_node = Node(
        package="piec_path_optimizer",
        executable="complete_path_optimizer",
        name="pinn_path_optimizer",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            **scout_mini_params,
            "use_pinn_predictions": True,
            "pinn_service_name": "/evaluate_trajectory",
            "population_size": 15,
            "generations": 6,
            "optimization_timeout": 2.0,
            "planning_rate": 2.5,
            "waypoint_count": 8,
            "max_curvature": 1.0,
            "path_smoothing": True,
            "obstacle_penalty_weight": 100.0,        # was 20.0
            "min_obstacle_distance": 0.6,           # Reduced from 0.6,0.5
            "objective_weights": [0.15, 0.1, 0.15, 0.1, 0.1, 0.25, 0.15],
            "debug_mode": True,
            "log_fitness": True,
            "goal_topic": "/goal_pose",
            "odom_topic": "/ukf/odom",
            "laser_topic": "/scan",        # Use the processed scan
            "use_sim_time": use_sim_time,
        }],
    )
    ld.add_action(pinn_path_optimizer_node)
    
    # 6.5 Controller – publishes directly to /cmd_vel (no mux)
    controller_node = Node(
        package="piec_controller",
        executable="controller_node",
        name="controller",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            **scout_mini_params,
            "max_linear_vel": 0.8,
            "min_linear_vel": 0.15,
            "max_angular_vel": 0.6,
            "emergency_stop_distance": 0.62,
            "slow_down_distance": 0.92,
            "safe_distance": 1.5,
            "waypoint_tolerance": 0.4,
            "lookahead_distance": 1.2,
            "control_frequency": 10.0,
            "linear_scale_factor": 1.0,              # No scaling, rely on UKF and robot driver
            "angular_scale_factor": 1.0,
            "angular_sign_correction": 1.0,          # Scout Mini follows standard ROS convention
            "debug_mode": True,
            "require_explicit_goal": True,            # Same as simulation
            "path_topic": "/piec/path",
            "cmd_vel_topic": "/cmd_vel_piec",               # Direct output to robot
            "odom_topic": "/ukf/odom",
            "scan_topic": "/scan",          # Use processed scan
            "use_sim_time": use_sim_time,
            "use_dwa": False,                          # Same as simulation
            "use_pinn_in_controller": False,
            "heading_kp": 1.5,
            "heading_deadband_deg": 2.0,
            "max_heading_rate": 0.6,
            "rotate_in_place_angle_deg": 180.0,
            "goal_stability_time": 2.0,
        }],
    )
    ld.add_action(controller_node)
    
    # 6.6 (Optional) Emergency Stop – disabled for now to avoid command conflicts.
    # If you want to use it, you would need a mux to switch between normal and emergency commands.
    # For simplicity, we omit it.
    # 6.6 Emergency Stop
    emergency_stop_node = Node(
        package="piec_controller",
        executable="emergency_stop",
        name="emergency_stop",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            "stop_distance": 0.62,  # Increased from 0.2
            "slow_distance": 0.92,  # Increased from 0.4
            "enable_emergency_stop": True,
            "use_sim_time": use_sim_time,
            "scan_topic": "/scan",
            "cmd_vel_topic": "/cmd_vel_piec",
            "output_topic": "/cmd_vel",
        }],
    )
    ld.add_action(emergency_stop_node)
    # 6.7 Teleop – optional keyboard control (publishes to /cmd_vel_teleop, but we don't mux it)
    # Since we removed the mux, teleop would conflict with controller if both publish to /cmd_vel.
    # Either run teleop only when controller is stopped, or use a different topic.
    # We'll keep it commented for now.
    teleop_piec_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_piec',
        output='screen',
        prefix='gnome-terminal --',
        parameters=[{'use_sim_time': use_sim_time, 'speed': 0.5, 'turn': 1.0}],
        remappings=[('/cmd_vel', '/cmd_vel_teleop')],
        condition=IfCondition(use_teleop)
    )
    ld.add_action(teleop_piec_node)
    # 6.8 Dynamics node (optional)
    dynamics_node = Node(
        package="piec_dynamics",
        executable="dynamics_node",
        name="piec_dynamics",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            "robot_mass": 50.0,
            "output_dir": os.path.expanduser("~/piec_dynamics"),
            "use_sim_time": use_sim_time,
        }],
        remappings=[("/openzen/data", "/imu")],
    )
    ld.add_action(dynamics_node)
    
    # 6.9 Metrics collector (optional)
    metrics_collector_node = Node(
        package="piec_validation",
        executable="metrics_collector",
        name="metrics_collector",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            "output_dir": os.path.expanduser("~/piec_metrics"),
            "sampling_rate": 5.0,
            "use_sim_time": use_sim_time,
        }],
    )
    ld.add_action(metrics_collector_node)
    
    # 6.10 TF validator (optional)
    tf_validator_node = Node(
        package="piec_bringup",
        executable="tf_validator",
        name="tf_validator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    ld.add_action(tf_validator_node)
    
    # 6.11 Experiment recorder (optional)
    experiment_recorder_node = Node(
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
    ld.add_action(experiment_recorder_node)
    
  
    
    return ld
