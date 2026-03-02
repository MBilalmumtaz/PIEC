import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('baylands_terrain_gazebo').find('baylands_terrain_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'baylands_terrain.world')

    # Tell Gazebo where to find models and worlds
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'models') + ':' +
              os.path.join(pkg_share, 'worlds')
    )

    # Launch Gazebo Sim with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file]
        }.items()
    )

    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
    ])
