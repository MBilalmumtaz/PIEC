#!/usr/bin/env python3
"""
COMPLETE PIEC stack - FIXED VERSION with RSHELIOS LiDAR
Single RViz with everything
"""

import os
from math import pi
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    
    # ------------------------------------------------------------------
    # 1. LAUNCH ARGUMENTS
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    teleop = LaunchConfiguration('teleop')
    use_base = LaunchConfiguration('use_base')
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    fake_scan = LaunchConfiguration('fake_scan')
    enable_pinn = LaunchConfiguration('enable_pinn')
    scout_port = LaunchConfiguration('scout_port')
    imu_port = LaunchConfiguration('imu_port')
    lidar_ip = LaunchConfiguration('lidar_ip')
    host_ip = LaunchConfiguration('host_ip')
    
    args = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Launch RViz2'
        ),
        DeclareLaunchArgument(
            name='teleop',
            default_value='false',
            description='Enable keyboard teleoperation'
        ),
        DeclareLaunchArgument(
            name='use_base',
            default_value='false',
            description='Enable Scout base driver'
        ),
        DeclareLaunchArgument(
            name='use_lidar',
            default_value='true',
            description='Enable Robosense Helios 16 lidar'
        ),
        DeclareLaunchArgument(
            name='use_imu',
            default_value='true',
            description='Enable WHEELTEC N100 IMU'
        ),
        DeclareLaunchArgument(
            name='fake_scan',
            default_value='false',
            description='Use fake laser scan'
        ),
        DeclareLaunchArgument(
            name='enable_pinn',
            default_value='false',
            description='Enable PINN services'
        ),
        DeclareLaunchArgument(
            name='scout_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Scout robot'
        ),
        DeclareLaunchArgument(
            name='imu_port',
            default_value='/dev/wheeltec_imu',
            description='Serial port for WHEELTEC N100 IMU'
        ),
        DeclareLaunchArgument(
            name='lidar_ip',
            default_value='192.168.1.200',
            description='Robosense Helios 16 IP address'
        ),
        DeclareLaunchArgument(
            name='host_ip',
            default_value='192.168.1.102',
            description='Host computer IP address'
        ),
        SetParameter(name='use_sim_time', value=use_sim_time),
    ]
    
    for arg in args:
        ld.add_action(arg)
    
    # ------------------------------------------------------------------
    # 2. FIX PERMISSIONS
    # ------------------------------------------------------------------
    fix_perms = ExecuteProcess(
        cmd=['bash', '-c', 'echo "Fixing permissions..."; \
              sudo chmod 666 /dev/ttyUSB0 2>/dev/null || true; \
              sudo chmod 666 /dev/ttyUSB1 2>/dev/null || true; \
              echo "Permissions fixed"'],
        output='screen'
    )
    ld.add_action(fix_perms)
    
    # ------------------------------------------------------------------
    # 3. ROBOT STATE PUBLISHER
    # ------------------------------------------------------------------
    scout_description_file = PathJoinSubstitution([
        FindPackageShare('agilex_scout'),
        'urdf',
        'robot.urdf.xacro',
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(
                Command([
                    FindExecutable(name='xacro'), ' ',
                    scout_description_file,
                    ' load_gazebo:=false',
                    ' odometry_source:=encoders',
                    ' simulation:=false',
                    ' lidar_type:=3d',
                ]), value_type=str
            )},
        ],
        remappings=[
            ('/robot_description', '/scout/robot_description'),
        ]
    )
    ld.add_action(robot_state_publisher)
    
    # ------------------------------------------------------------------
    # 4. TF TRANSFORMS
    # ------------------------------------------------------------------
    # Map to Odom
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(map_to_odom)
    
    # Base to Lidar
    base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['--x', '0.15', '--y', '0', '--z', '0.3',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'base_link', '--child-frame-id', 'rslidar'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(base_to_lidar)
    
    # Base to IMU
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0.1',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_imu)
    )
    ld.add_action(base_to_imu)
    
    # ------------------------------------------------------------------
    # 5. RSHELIOS LIDAR DRIVER
    # ------------------------------------------------------------------
    rslidar_driver = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'common.msg_source': 1,
            'common.send_point_cloud_ros': True,
            'lidar.0.driver.lidar_type': 'RSHELIOS',
            'lidar.0.driver.msop_port': 6699,
            'lidar.0.driver.difop_port': 7788,
            'lidar.0.driver.host_address': LaunchConfiguration('host_ip'),
            'lidar.0.driver.wait_for_difop': False,
            'lidar.0.driver.use_lidar_clock': False,
            'lidar.0.driver.min_distance': 0.2,
            'lidar.0.driver.max_distance': 200.0,
            'lidar.0.driver.start_angle': 0,
            'lidar.0.driver.end_angle': 360,
            'lidar.0.ros.ros_frame_id': 'rslidar',
            'lidar.0.ros.ros_send_point_cloud_topic': '/rslidar_points',
            'lidar.0.ros.ros_queue_length': 10,
        }],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(rslidar_driver)
    
    # ------------------------------------------------------------------
    # 6. DUMMY IMU
    # ------------------------------------------------------------------
    dummy_imu_script = '''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class DummyIMU(Node):
    def __init__(self):
        super().__init__('dummy_imu')
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.1, self.publish)
        self.get_logger().info('Dummy IMU started')
    
    def publish(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.z = 9.81
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = DummyIMU()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    dummy_imu = ExecuteProcess(
        cmd=['bash', '-c', f'python3 -c "{dummy_imu_script}"'],
        output='screen',
        condition=IfCondition(use_imu),
        shell=True
    )
    ld.add_action(dummy_imu)
    
    # ------------------------------------------------------------------
    # 7. POINTCLOUD TO LASERSCAN
    # ------------------------------------------------------------------
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/rslidar_points'),
            ('scan', '/scan_raw')
        ],
        parameters=[{
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 0.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 30.0,
            'use_inf': True,
            'use_sim_time': use_sim_time,
            'target_frame': 'rslidar',
            'concurrency_level': 1,
        }],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(pointcloud_to_laserscan)
    
    # ------------------------------------------------------------------
    # 8. PIEC ALGORITHM NODES
    # ------------------------------------------------------------------
    scout_mini_params = {
        'robot_radius': 0.3,
        'wheel_radius': 0.16,
        'wheel_separation': 0.4563536,
        'wheel_base': 0.3132556,
        'max_linear_vel': 1.2,
        'min_linear_vel': 0.3,
        'max_angular_vel': 0.8,
        'k_linear': 1.0,
        'k_angular': 1.5,
    }
    
    # Laser Bridge
    laser_bridge_condition = PythonExpression([
        '"', use_lidar, '" == "true" or "', fake_scan, '" == "true"'
    ])
    
    laser_bridge = Node(
        package='piec_controller',
        executable='laser_bridge',
        name='laser_bridge',
        output='screen',
        parameters=[{
            **scout_mini_params,
            'use_sim_time': use_sim_time,
            'input_topic': '/scan_raw',
            'output_scan_topic': '/scan',
        }],
        condition=IfCondition(laser_bridge_condition)
    )
    ld.add_action(laser_bridge)
    
    # UKF Localization
    ukf_localization = Node(
        package='piec_ukf_localization',
        executable='ukf_node',
        name='ukf_localization',
        output='screen',
        parameters=[{
            **scout_mini_params,
            'use_sim_time': use_sim_time,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_theta': 0.0,
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'global_frame': 'map',
            'imu_topic': '/imu',
            'scan_topic': '/scan',
            'enable_pinn': enable_pinn,
        }],
        remappings=[
            ('/ukf/odom', '/fused_odom'),
        ]
    )
    ld.add_action(ukf_localization)
    
    # Path Optimizer
    path_optimizer = Node(
        package='piec_path_optimizer',
        executable='complete_path_optimizer',
        name='path_optimizer',
        output='screen',
        parameters=[{
            **scout_mini_params,
            'use_sim_time': use_sim_time,
            'enable_pinn': enable_pinn,
            'planning_rate': 1.0,
            'debug_mode': True,
        }]
    )
    ld.add_action(path_optimizer)
    
    # Controller
    controller = Node(
        package='piec_controller',
        executable='controller_node',
        name='controller',
        output='screen',
        parameters=[{
            **scout_mini_params,
            'use_sim_time': use_sim_time,
            'enable_pinn': enable_pinn,
            'emergency_stop_distance': 0.2,
            'control_frequency': 15.0,
            'debug_mode': True,
            'odom_topic': '/fused_odom',
            'scan_topic': '/scan',
        }],
        remappings=[
            ('/cmd_vel', '/controller_cmd_vel'),
        ]
    )
    ld.add_action(controller)
    
    # Emergency Stop
    emergency_stop = Node(
        package='piec_controller',
        executable='emergency_stop',
        name='emergency_stop',
        output='screen',
        parameters=[{
            'stop_distance': 0.2,
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',
        }]
    )
    ld.add_action(emergency_stop)
    
    # Metrics Collector
    metrics_collector = Node(
        package='piec_validation',
        executable='metrics_collector',
        name='metrics_collector',
        output='screen',
        parameters=[{
            'output_dir': '/tmp/piec_metrics',
            'sampling_rate': 5.0,
            'use_sim_time': use_sim_time,
        }]
    )
    ld.add_action(metrics_collector)
    
    # ------------------------------------------------------------------
    # 9. COMMAND VELOCITY MUX
    # ------------------------------------------------------------------
    cmd_vel_mux = Node(
        package='topic_tools',
        executable='mux',
        name='cmd_vel_mux',
        arguments=['/cmd_vel', '/manual_cmd_vel', '/controller_cmd_vel', '/emergency_cmd_vel'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(cmd_vel_mux)
    
    # ------------------------------------------------------------------
    # 10. TELEOPERATION
    # ------------------------------------------------------------------
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/manual_cmd_vel')
        ],
        condition=IfCondition(teleop)
    )
    ld.add_action(teleop_keyboard)
    
    # ------------------------------------------------------------------
    # 11. RViz2 - SINGLE INSTANCE WITH EVERYTHING
    # ------------------------------------------------------------------
    # First, create a merged RViz config
    create_rviz_config = ExecuteProcess(
        cmd=['bash', '-c', f'''
            # Merge the two RViz configs
            ROBOT_RVIZ="{FindPackageShare('agilex_scout').find('agilex_scout')}/rviz/real_scout_display.rviz"
            LIDAR_RVIZ="{FindPackageShare('rslidar_sdk').find('rslidar_sdk')}/rviz/rviz2.rviz"
            MERGED_RVIZ="/tmp/merged_scout_lidar.rviz"
            
            # Update robot RViz to use RSHELIOS topics instead of Ouster
            if [ -f "$ROBOT_RVIZ" ]; then
                cp "$ROBOT_RVIZ" "$MERGED_RVIZ"
                # Change Ouster topics to RSHELIOS
                sed -i 's|/ouster/points|/rslidar_points|g' "$MERGED_RVIZ"
                sed -i 's|/ouster/scan|/scan|g' "$MERGED_RVIZ"
                # Ensure fixed frame is odom (for robot)
                sed -i 's|Fixed Frame: rslidar|Fixed Frame: odom|' "$MERGED_RVIZ"
                echo "Created merged RViz config at $MERGED_RVIZ"
            else
                # Fallback to LiDAR config
                cp "$LIDAR_RVIZ" "$MERGED_RVIZ"
                sed -i 's|Fixed Frame: rslidar|Fixed Frame: odom|' "$MERGED_RVIZ"
            fi
        '''],
        output='screen',
        shell=True,
        condition=IfCondition(rviz)
    )
    ld.add_action(create_rviz_config)
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/tmp/merged_scout_lidar.rviz'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        # Don't launch RViz from rslidar_sdk launch file
    )
    ld.add_action(rviz2)
    
    # ------------------------------------------------------------------
    # 12. SYSTEM CHECK
    # ------------------------------------------------------------------
    system_check = ExecuteProcess(
        cmd=['bash', '-c', '''
            echo "=== Waiting for system to initialize ==="
            sleep 15
            echo ""
            echo "=== SYSTEM STATUS ==="
            echo ""
            echo "📊 Nodes:"
            timeout 3 ros2 node list 2>/dev/null | head -15
            echo ""
            echo "📡 Critical Topics:"
            timeout 3 ros2 topic list 2>/dev/null | grep -E "(imu|scan|odom|cmd_vel|rslidar)" | head -15
            echo ""
            echo "🎯 LiDAR Data:"
            timeout 3 ros2 topic hz /rslidar_points --window 3 2>/dev/null || echo "  Checking..."
            echo ""
            echo "🤖 Robot TF Frames:"
            timeout 3 ros2 run tf2_tools view_frames 2>/dev/null && echo "  Frames saved to frames.pdf"
            echo ""
            echo "✅ SINGLE RViz with RSHELIOS LiDAR is running!"
            echo "   Robot + LiDAR should both appear in one RViz window"
            echo "   Use Ctrl+C to shutdown"
        '''],
        output='screen',
        shell=True
    )
    ld.add_action(system_check)
    
    return ld
