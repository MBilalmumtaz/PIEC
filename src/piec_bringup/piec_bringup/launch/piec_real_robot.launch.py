#!/usr/bin/env python3
"""
Complete launch file for PIEC stack with PINN - REAL ROBOT VERSION (FIXED)
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
    # 1. Launch arguments for real robot
    # ------------------------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "enable_pinn", 
        default_value="true",
        description="Enable PINN service"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_rviz", 
        default_value="true",
        description="Enable RViz visualization"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", 
        default_value="false",
        description="Use simulation time (FALSE for real robot)"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_teleop", 
        default_value="true",
        description="Enable keyboard teleoperation"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_lidar", 
        default_value="true",
        description="Enable LiDAR"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_imu", 
        default_value="true",
        description="Enable IMU"
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_base", 
        default_value="true",
        description="Enable Scout base driver"
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_piecnodes", 
        default_value="true",
        description="Enable all PIEC algorithm nodes"
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_recording",
        default_value="false",
        description="Enable experimental data recording"
    ))
    ld.add_action(DeclareLaunchArgument(
        "method",
        default_value="PIEC",
        description="Navigation method being tested"
    ))
    ld.add_action(DeclareLaunchArgument(
        "environment",
        default_value="corridor",
        description="Environment type"
    ))
    ld.add_action(DeclareLaunchArgument(
        "trial_name",
        default_value="real_trial_001",
        description="Unique trial identifier"
    ))
    ld.add_action(DeclareLaunchArgument(
        "output_dir",
        default_value="~/piec_data",
        description="Data output directory"
    ))
    
    # Get launch arguments
    enable_pinn = LaunchConfiguration("enable_pinn")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_teleop = LaunchConfiguration("use_teleop")
    use_lidar = LaunchConfiguration("use_lidar")
    use_imu = LaunchConfiguration("use_imu")
    use_base = LaunchConfiguration("use_base")
    enable_piecnodes = LaunchConfiguration("enable_piecnodes")
    
    # Data recording configurations
    enable_recording = LaunchConfiguration("enable_recording")
    method = LaunchConfiguration("method")
    environment = LaunchConfiguration("environment")
    trial_name = LaunchConfiguration("trial_name")
    output_dir = LaunchConfiguration("output_dir")
    
    # ------------------------------------------------------------------
    # 2. GLOBAL PARAMETER
    # ------------------------------------------------------------------
    ld.add_action(SetParameter(name="use_sim_time", value=use_sim_time))
    
    # ------------------------------------------------------------------
    # 3. LOAD REAL ROBOT HARDWARE (WITH FIXES)
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
            'use_rviz': 'true',  # We'll use PIEC RViz config instead
            'use_teleop': 'true',  # PIEC has its own teleop
            'use_lidar': use_lidar,
            'use_imu': use_imu,
            'use_base': use_base,
        }.items()
    )
    ld.add_action(real_robot_launch)
    
    # ------------------------------------------------------------------
    # 4. WAIT FOR HARDWARE TO INITIALIZE
    # ------------------------------------------------------------------
    ld.add_action(
        ExecuteProcess(
            cmd=['sleep', '3'],
            output='screen'
        )
    )
    
    # ------------------------------------------------------------------
    # 5. SCOUT MINI PARAMETERS FOR REAL ROBOT
    # ------------------------------------------------------------------
    scout_mini_params = {
        "robot_radius": 0.3,
        "wheel_radius": 0.16,
        "wheel_separation": 0.4563536,
        "wheel_base": 0.3132556,
        "max_linear_vel": 0.8,
        "max_angular_vel": 0.6,
        "use_sim_time": use_sim_time,
        "real_robot_mode": True,  # NEW: Flag for real robot
        "speed_calibration_factor": 0.95,  # Calibration factor
    }
    
    # ------------------------------------------------------------------
    # 6. PIEC STACK NODES FOR REAL ROBOT (UPDATED WITH CORRECT TOPICS)
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
    
    # 6.2 UKF Localization - FIXED: Using correct IMU topic
    ukf_localization_node = Node(
        package="piec_ukf_localization",
        executable="ukf_node",
        name="ukf_localization",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            **scout_mini_params,
            "odom_topic": "/odometry",
            "imu_topic": "/openzen/data",  # OpenZen driver publishes directly to this topic (not /imu)
            "publish_topic": "/ukf/odom",
            
            # CRITICAL NEW PARAMETERS
            "imu_yaw_offset": 0.0,  # CHANGED from -0.94 to 0.0
            "use_imu_orientation": True,
            "use_wheel_velocity": True,
            "use_wheel_odometry": True,
            "wheel_radius": 0.16,
            "wheel_base": 0.313,
            "initial_yaw": 0.0,
            "debug_mode": True,
            # Tuning parameters - updated for real sensors
            "Q_position": 0.05,
            "Q_orientation": 0.01,
            "R_imu_orientation": 0.005,
            "R_odom_position": 0.02,
            "R_pose": [0.01, 0.01, 0.005],
            "R_yawrate": [0.001],
            "imu_frame": "imu_link",
            "base_frame": "base_link",
            "world_frame": "odom",
            "dt": 0.05,
            "real_robot_mode": True,
        }],
    )
    ld.add_action(ukf_localization_node)
    
    # 6.3 Laser Bridge
    laser_bridge_node = Node(
        package="piec_controller",
        executable="laser_bridge",
        name="laser_bridge",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            "min_range": 0.1,
            "max_range": 8.0,
            "range_filter_window": 3,
            "median_filter": True,
            "input_topic": "/laser_scan",
            "output_fixed_topic": "/scan_fixed",
            "output_scan_topic": "/scan_processed",
            "use_sim_time": use_sim_time,
        }],
    )
    ld.add_action(laser_bridge_node)
    
    # 6.4 PINN-INTEGRATED PATH OPTIMIZER
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
            "objective_weights": [0.15, 0.1, 0.15, 0.1, 0.1, 0.25, 0.15],
            "debug_mode": True,
            "log_fitness": True,
            "goal_topic": "/goal_pose",
            "odom_topic": "/ukf/odom",
            "laser_topic": "/scan_processed",
            "use_sim_time": use_sim_time,
        }],
    )
    ld.add_action(pinn_path_optimizer_node)
    
    # 6.5 Controller
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
            "emergency_stop_distance": 0.3,
            "slow_down_distance": 0.7,
            "safe_distance": 1.0,
            "waypoint_tolerance": 0.4,
            "lookahead_distance": 1.2,
            "control_frequency": 10.0,
            "linear_scale_factor": 0.95,
            "angular_scale_factor": 0.99,
            "angular_sign_correction": 1.0,  # FIX: Scout Mini follows standard ROS convention (+w=CCW, -w=CW), no inversion needed
            "debug_mode": True,
            "require_explicit_goal": False,  # Changed from True to False for autonomous path following
            "path_topic": "/piec/path",
            "cmd_vel_topic": "/cmd_vel_piec",
            "odom_topic": "/ukf/odom",
            "scan_topic": "/scan_processed",  # Use consistent scan topic to avoid QoS warnings
            "use_sim_time": use_sim_time,
            "use_dwa": False,
            "use_pinn_in_controller": False,
            # New heading control parameters
            "heading_kp": 1.5,
            "heading_deadband_deg": 2.0,
            "max_heading_rate": 0.6,
            "rotate_in_place_angle_deg": 180.0,
            "goal_stability_time": 2.0,
        }],
    )
    ld.add_action(controller_node)
    
    # 6.6 Emergency Stop
    emergency_stop_node = Node(
        package="piec_controller",
        executable="emergency_stop",
        name="emergency_stop",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            "stop_distance": 0.3,  # Increased from 0.2
            "slow_distance": 0.6,  # Increased from 0.4
            "enable_emergency_stop": True,
            "use_sim_time": use_sim_time,
            "scan_topic": "/scan_fixed",
            "cmd_vel_topic": "/cmd_vel_piec",
            "output_topic": "/cmd_vel_safe",
        }],
    )
    ld.add_action(emergency_stop_node)
    
    # 6.7 PIEC Command Multiplexer
    cmd_vel_mux = Node(
        package="topic_tools",
        executable="mux",
        name="piec_cmd_vel_mux",
        arguments=["/cmd_vel", "/cmd_vel_safe", "/cmd_vel_teleop", "/cmd_vel_piec"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "lazy": True,
            "active_topic": "/cmd_vel_safe",
        }],
        remappings=[
            ("/cmd_vel", "/cmd_vel"),
        ]
    )
    ld.add_action(cmd_vel_mux)
    
    # 6.8 Teleop for manual override
   # teleop_piec_node = Node(
     #   package='teleop_twist_keyboard',
      #  executable='teleop_twist_keyboard',
      #  name='teleop_piec',
       # output='screen',
      #  prefix='gnome-terminal --',
    #    parameters=[{
    #        'use_sim_time': use_sim_time,
    #        'speed': 0.5,
    #        'turn': 1.0,
     #   }],
     #   remappings=[
     #       ('/cmd_vel', '/cmd_vel_teleop'),
     #   ],
     #   condition=IfCondition(use_teleop)
   # )
   # ld.add_action(teleop_piec_node)
    
    # ------------------------------------------------------------------
    # 7. RViz2 for PIEC
    # ------------------------------------------------------------------
   # rviz2_node = Node(
      #  package='rviz2',
       # executable='rviz2',
       # name='rviz2_piec',
       # arguments=['-d', PathJoinSubstitution([
        #    FindPackageShare('agilex_scout'),
       #     'rviz',
         #   'piec_display.rviz'  # Create this RViz config for PIEC
       # ])],
       # parameters=[{'use_sim_time': use_sim_time}],
       # output='screen',
     #  condition=IfCondition(use_rviz)
   # )
  #  ld.add_action(rviz2_node)
    
    # ------------------------------------------------------------------
    # 8. DYNAMICS AND VALIDATION NODES (Optional)
    # ------------------------------------------------------------------
    dynamics_node = Node(
        package="piec_dynamics",
        executable="dynamics_node",
        name="piec_dynamics",
        output="screen",
        condition=IfCondition(enable_piecnodes),
        parameters=[{
            "robot_mass": 50.0,  # Scout Mini mass in kg
            "output_dir": os.path.expanduser("~/piec_dynamics"),
            "use_sim_time": use_sim_time,
        }],
        remappings=[
            ("/imu/data", "/imu"),
        ],
    )
    ld.add_action(dynamics_node)
    
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
    
    # ------------------------------------------------------------------
    # 9. TF VALIDATOR FOR REAL ROBOT
    # ------------------------------------------------------------------
    tf_validator_node = Node(
        package="piec_bringup",
        executable="tf_validator",
        name="tf_validator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )
    ld.add_action(tf_validator_node)
    
    # ------------------------------------------------------------------
    # 10. EXPERIMENT RECORDER (Optional)
    # ------------------------------------------------------------------
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
