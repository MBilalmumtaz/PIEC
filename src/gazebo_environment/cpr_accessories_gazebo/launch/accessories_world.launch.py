import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Robot spawn position arguments
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    world_name = LaunchConfiguration('world_name')
    
    declare_robot_x = DeclareLaunchArgument('robot_x', default_value='0.0')
    declare_robot_y = DeclareLaunchArgument('robot_y', default_value='0.0')
    declare_robot_z = DeclareLaunchArgument('robot_z', default_value='0.2346')
    declare_robot_yaw = DeclareLaunchArgument('robot_yaw', default_value='0.0')
    
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_headless = DeclareLaunchArgument('headless', default_value='false')
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value=PathJoinSubstitution([
            FindPackageShare('cpr_accessories_gazebo'),
            'worlds',
            'actually_empty_world.world'
        ])
    )
    
    # Scout Mini robot description
    scout_description_file = os.path.join(
        get_package_share_directory("agilex_scout"),
        "urdf",
        "robot.urdf.xacro"
    )
    
    scout_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        scout_description_file,
        " odometry_source:=ground_truth",
        " load_gazebo:=true",
        " simulation:=true",
        " lidar_type:=3d"
    ])
    
    scout_description = {
        "robot_description": ParameterValue(scout_description_content, value_type=str)
    }
    
    # Robot state publisher for Scout Mini
    robot_state_publisher = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="scout",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            scout_description
        ],
        remappings=[
            ("/joint_states", "/scout/joint_states"),
            ("/robot_description", "/scout/robot_description"),
        ],
    )
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_name,
            'gui': gui,
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )
    
    # Spawn Scout Mini robot
    spawn_robot = Node(
        name="spawn_robot_urdf",
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "scout_mini",
            "-topic", "/scout/robot_description",
            "-x", robot_x,
            "-y", robot_y,
            "-z", robot_z,
            "-R", "0",
            "-P", "0",
            "-Y", robot_yaw,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen",
    )
    
    return LaunchDescription([
        declare_robot_x,
        declare_robot_y,
        declare_robot_z,
        declare_robot_yaw,
        declare_use_sim_time,
        declare_gui,
        declare_headless,
        declare_world_name,
        robot_state_publisher,
        gazebo_launch,
        spawn_robot
    ])
