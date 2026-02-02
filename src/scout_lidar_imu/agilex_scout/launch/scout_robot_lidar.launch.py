#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()
    
    # ------------------------------------------------------------------
    # 1. LAUNCH ARGUMENTS
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_base = LaunchConfiguration('use_base')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    
    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Enable RSHELIOS LiDAR'
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Enable OpenZen IMU (LPMS IG1 CAN)'
        ),
        DeclareLaunchArgument(
            'use_base',
            default_value='true',
            description='Enable Scout base driver'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Enable RViz2 visualization'
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Enable keyboard teleop'
        ),
        SetParameter(name='use_sim_time', value=use_sim_time),
    ]
    
    for arg in args:
        ld.add_action(arg)
    
    # ------------------------------------------------------------------
    # 2. ROBOT STATE PUBLISHER
    # ------------------------------------------------------------------
    scout_description_file = PathJoinSubstitution([
        FindPackageShare("agilex_scout"),
        "urdf",
        "robot.urdf.xacro",
    ])
    
    scout_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(
                Command([
                    FindExecutable(name="xacro"), ' ',
                    scout_description_file,
                    ' load_gazebo:=false',
                    ' odometry_source:=encoders',
                    ' simulation:=false',
                    ' lidar_type:=3d',
                    ' imu_frame:=imu_link',
                ]), value_type=str
            ),
            "publish_frequency": 30.0,
        }]
    )
    ld.add_action(scout_robot_state_publisher_node)
    
    # ------------------------------------------------------------------
    # 3. SCOUT BASE DRIVER
    # ------------------------------------------------------------------
    scout_base_node = Node(
        package='scout_base',
        executable='scout_base_node',
        name='scout_base_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port_name': 'can0',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'odom_topic_name': '/odometry',
            'is_scout_mini': True,
            'is_omni_wheel': False,
            'simulated_robot': False,
            'control_rate': 50,
            'auto_reconnect': True,
            'heartbeat_timeout': 5.0,
        }],
        condition=IfCondition(use_base),
        emulate_tty=True,
    )
    ld.add_action(scout_base_node)
    
    # ------------------------------------------------------------------
    # 4. RSHELIOS LIDAR (RoboSense Helios 16)
    # ------------------------------------------------------------------
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'common.msg_source': 1,
            'common.send_point_cloud_ros': True,
            'lidar.0.driver.lidar_type': 'RSHELIOS',
            'lidar.0.driver.model': 'RSHELIOS_16',
            'lidar.0.driver.msop_port': 6699,
            'lidar.0.driver.difop_port': 7788,
            'lidar.0.driver.host_address': '192.168.1.200',
            'lidar.0.driver.group_address': '192.168.1.102',
            'lidar.0.driver.wait_for_difop': True,
            'lidar.0.driver.use_lidar_clock': False,
            'lidar.0.driver.min_distance': 0.2,
            'lidar.0.driver.max_distance': 100.0,
            'lidar.0.driver.start_angle': 0.0,
            'lidar.0.driver.end_angle': 360.0,
            'lidar.0.driver.cut_angle': 0.0,
            'lidar.0.driver.frame_id': 'rslidar',
            'lidar.0.ros.ros_frame_id': 'rslidar',
            'lidar.0.ros.ros_send_point_cloud_topic': '/rslidar_points',
            'lidar.0.ros.ros_send_point_cloud_interval': 1.0,
            'lidar.0.ros.ros_queue_length': 10,
        }],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(rslidar_node)
    
    # ------------------------------------------------------------------
    # 5. OPENZEN IMU DRIVER (LPMS IG1 CAN)
    # ------------------------------------------------------------------
    openzen_imu_node = Node(
        package='openzen_driver',
        executable='openzen_node',
        name='openzen_imu_driver',
        namespace='openzen',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sensor_name': '',  # Empty string means auto-detect first sensor
            'baudrate': 0,  # Use 0 for auto-detect; OpenZen selects appropriate rate for the sensor
            'frame_id': 'imu_link',
        }],
        condition=IfCondition(use_imu)
    )
    ld.add_action(openzen_imu_node)
    
    # ------------------------------------------------------------------
    # 6. IMU FILTER (Madgwick filter with magnetometer)
    # ------------------------------------------------------------------
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='scout_imu_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_mag': True,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'imu_link',
            'mag_bias_x': 0.000010,
            'mag_bias_y': -0.000112,
            'mag_bias_z': 0.000094,
            'mag_declination': 0.092,
            'yaw_offset': -0.94,
            'gain': 0.1,
            'zeta': 0.05,
            'remove_gravity_vector': True,
        }],
        remappings=[
            ('/imu/data_raw', '/openzen/data'),
            ('/imu/data', '/imu'),
            ('/imu/mag', '/openzen/mag'),  # CHANGED: Direct connection to OpenZen
        ],
        condition=IfCondition(use_imu)
    )
    ld.add_action(imu_filter_node)
    
    # ------------------------------------------------------------------
    # 8. POINTCLOUD TO LASERSCAN
    # ------------------------------------------------------------------
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/rslidar_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'rslidar',
            'transform_tolerance': 0.1,
            'min_height': -0.3,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True,
            'concurrency_level': 1,
        }],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(pointcloud_to_laserscan_node)
    
    # ------------------------------------------------------------------
    # 9. TF TRANSFORMS
    # ------------------------------------------------------------------
    # Base to IMU transform
    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0', '0', '0.2', '0', '0', '-0.84', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_imu)
    )
    ld.add_action(base_to_imu_tf)
    
    # Base to LiDAR transform
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.15', '0', '0.3', '0', '0', '0', 'base_link', 'rslidar'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(base_to_lidar_tf)
    
    # Odom to base_footprint (static as backup)
    odom_to_basefoot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_basefoot_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_base)
    )
    ld.add_action(odom_to_basefoot)
    
    # ------------------------------------------------------------------
    # 10. TELEOP KEYBOARD
    # ------------------------------------------------------------------
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --',
        parameters=[{
            'use_sim_time': use_sim_time,
            'speed': 0.5,
            'turn': 1.0,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
        condition=IfCondition(use_teleop)
    )
    ld.add_action(teleop_node)
    
    # ------------------------------------------------------------------
    # 11. RViz2
    # ------------------------------------------------------------------
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('agilex_scout'),
            'rviz',
            'real_scout_display.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    ld.add_action(rviz2_node)
    
    # ------------------------------------------------------------------
    # 12. TOPIC RELAYS FOR PIEC COMPATIBILITY
    # ------------------------------------------------------------------
    # Relay for scan topic
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        arguments=['/scan', '/laser_scan'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(scan_relay)
    
    # Relay for pointcloud (optional)
    points_relay = Node(
        package='topic_tools',
        executable='relay',
        name='points_relay',
        arguments=['/rslidar_points', '/points'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_lidar)
    )
    ld.add_action(points_relay)
    
    return ld
