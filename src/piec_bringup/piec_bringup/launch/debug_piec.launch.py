# debug_piec.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Declare arguments
    declare_goal_topic = DeclareLaunchArgument(
        'goal_topic', default_value='/goal_pose'
    )
    ld.add_action(declare_goal_topic)
    
    # Diagnostic command (delayed start)
    ld.add_action(
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 5 && ros2 topic list'],
            output='screen',
            name='list_topics'
        )
    )
    
    # UKF with correct topic mapping
    ukf_node = Node(
        package='piec_ukf_localization',
        executable='ukf_node',
        output='screen',
        parameters=[{'odom_topic': '/odometry', 'imu_topic': '/imu/data', 'dt': 0.05}],
        name='ukf_node_debug'
    )
    ld.add_action(ukf_node)
    
    # PINN service
    pinn_node = Node(
        package='piec_pinn_surrogate',
        executable='pinn_service',
        output='screen',
        parameters=[{'model_path': ''}],
        name='pinn_service_debug'
    )
    ld.add_action(pinn_node)
    
    # Path optimizer (starts after UKF is ready)
    path_optimizer = Node(
        package='piec_path_optimizer',
        executable='path_optimizer',
        output='screen',
        parameters=[
            {'goal_topic': LaunchConfiguration('goal_topic')},
            {'pop_size': 12},  # Reduced for faster debugging
            {'max_generations': 8},  # Reduced for faster debugging
            {'n_waypoints': 4}  # Reduced for faster debugging
        ],
        name='path_optimizer_debug'
    )
    ld.add_action(path_optimizer)
    
    # Controller
    controller = Node(
        package='piec_controller',
        executable='controller_node',
        output='screen',
        name='controller_node_debug'
    )
    ld.add_action(controller)
    
    # Test goal publisher (starts after a delay)
    test_publisher = Node(
        package='piec_bringup',
        executable='test_goal_publisher',
        output='screen',
        name='test_goal_publisher',
        parameters=[{'use_sim_time': False}]
    )
    
    # Add test publisher with delay
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=path_optimizer,
                on_start=[test_publisher]
            )
        )
    )
    
    return ld
