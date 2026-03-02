# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    no_roof_small_warehouse_world_file = os.path.join(
        get_package_share_directory("aws_robomaker_small_warehouse_world"),
        "worlds",
        "no_roof_small_warehouse",
        "no_roof_small_warehouse.world"
    )
    
    # Get the walking actor package share directory
    walking_actor_share = get_package_share_directory("walking_actor")
    
    # Get the warehouse models directory
    warehouse_share = get_package_share_directory("aws_robomaker_small_warehouse_world")
    
    # Set the Gazebo resource path to include both warehouse models and walking actor
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(warehouse_share, 'models') + ':' +
              os.path.join(walking_actor_share, 'models')
    )
    
    return LaunchDescription(
        [
            gazebo_resource_path,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory(
                            "aws_robomaker_small_warehouse_world"
                        ),
                        "/launch/small_warehouse.launch.py",
                    ]
                ),
                launch_arguments={
                    "world": no_roof_small_warehouse_world_file
                }.items(),
            ),
        ]
    )
