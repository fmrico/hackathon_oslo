# Copyright 2023 Intelligent Robotics Lab
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    scenario_maps_dir = get_package_share_directory('scenario_maps')
    param_file = os.path.join(scenario_maps_dir, 'config', 'bombs_config_hospital.yaml')
    rviz_file = os.path.join(scenario_maps_dir, 'config', 'config_hospital.rviz')

    artificier_cmd = Node(
        name='artificier',
        package='scenario_maps',
        executable='artificier',
        parameters=[
          param_file,
          {'use_sim_time': True}
        ],
        output='both',
    )

    bombs_spawn_cmd = Node(
        package='scenario_maps',
        executable='bombs_spawn_hospital',
        parameters=[
          param_file,
          {'use_sim_time': True}
        ],
        output='screen')

    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[
          {'use_sim_time': True}
        ],
        arguments=['-d', rviz_file], 
        output='screen')

    ld = LaunchDescription()

    ld.add_action(artificier_cmd)
    ld.add_action(bombs_spawn_cmd)
    ld.add_action(rviz2_cmd)

    return ld
