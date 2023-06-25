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
    container_name = LaunchConfiguration('container_name')

    scenario_dir = get_package_share_directory('scenario_train')
    param_file = os.path.join(scenario_dir, 'config', 'bombs_config.yaml')
    rviz_file = os.path.join(scenario_dir, 'config', 'config.rviz')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='train_scenario_test_container',
        description='Name of the container')

    container_cmd = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )
 
    # FIX01: Load component Artificier into de container 
    # load_composable_nodes = LoadComposableNodes(
    #     target_container=container_name,
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='...',
    #             plugin='...',
    #             name='artificier',
    #             parameters=[
    #               param_file,
    #               {'use_sim_time': True}
    #             ])
    #     ])

    # FIX02: Run executable bombs_spawn with param_file as parameter
    # bombs_spawn_cmd = Node(
    #     ...,
    #     ...,
    #     parameters=[
    #       param_file,
    #       {'use_sim_time': True}
    #     ],
    #     output='screen')

    # FIX03: Run rviz2 with rviz_file as configuration
    # rviz2_cmd = Node(
    #     ...,
    #     ...,
    #     parameters=[
    #       {'use_sim_time': True}
    #     ],
    #     arguments=..., 
    #     output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_container_name_cmd)
    ld.add_action(container_cmd)
    # ld.add_action(load_composable_nodes)
    # ld.add_action(bombs_spawn_cmd)
    # ld.add_action(rviz2_cmd)

    return ld
