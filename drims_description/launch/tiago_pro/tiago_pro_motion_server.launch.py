# Copyright 2024 National Research Council STIIMA
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
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_path = context.perform_substitution(LaunchConfiguration('motion_server_config_path'))
    arm_left_config_path = context.perform_substitution(LaunchConfiguration('motion_server_arm_left_config_path'))
    arm_right_config_path = context.perform_substitution(LaunchConfiguration('motion_server_arm_right_config_path'))
    arm_right_torso_config_path = context.perform_substitution(LaunchConfiguration('motion_server_arm_right_torso_config_path'))
    use_sim_time = context.perform_substitution(LaunchConfiguration('use_sim_time')).lower() in ['true', '1', 'yes']
    safety_scaling_str = context.perform_substitution(LaunchConfiguration('safety_scaling'))

    scaling_params = {}
    has_cli_safety_scaling = any(arg.startswith('safety_scaling:=') for arg in sys.argv)
    if has_cli_safety_scaling and safety_scaling_str != '':
        try:
            scaling_params['safety_velocity_scaling'] = float(safety_scaling_str)
        except ValueError:
            pass

    nodes = [
        Node(
            package='easy_motion',
            executable='motion_server',
            name='motion_server_node',
            output='screen',
            parameters=[config_path, {'use_sim_time': use_sim_time, **scaling_params}]
        ),
        Node(
            package='easy_motion',
            executable='motion_server',
            name='motion_server_node',
            namespace='arm_left',
            output='screen',
            parameters=[arm_left_config_path, {'use_sim_time': use_sim_time, **scaling_params}]
        ),
        Node(
            package='easy_motion',
            executable='motion_server',
            name='motion_server_node',
            namespace='arm_right',
            output='screen',
            parameters=[arm_right_config_path, {'use_sim_time': use_sim_time, **scaling_params}]
        ),
        Node(
            package='easy_motion',
            executable='motion_server',
            name='motion_server_node',
            namespace='arm_right_torso',
            output='screen',
            parameters=[arm_right_torso_config_path, {'use_sim_time': use_sim_time, **scaling_params}]
        ),
    ]
    return nodes


def generate_launch_description():
    drims_description_pkg_dir = get_package_share_directory('drims_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'motion_server_config_path',
            default_value=os.path.join(drims_description_pkg_dir, 'config/tiago_pro/motion_server_config.yaml'),
            description='Full path to the config file'
        ),
        DeclareLaunchArgument(
            'motion_server_arm_left_config_path',
            default_value=os.path.join(drims_description_pkg_dir, 'config/tiago_pro/motion_server_arm_left_config.yaml'),
            description='Full path to the arm_left config file'
        ),
        DeclareLaunchArgument(
            'motion_server_arm_right_config_path',
            default_value=os.path.join(drims_description_pkg_dir, 'config/tiago_pro/motion_server_arm_right_config.yaml'),
            description='Full path to the arm_right config file'
        ),
        DeclareLaunchArgument(
            'motion_server_arm_right_torso_config_path',
            default_value=os.path.join(drims_description_pkg_dir, 'config/tiago_pro/motion_server_arm_right_torso_config.yaml'),
            description='Full path to the arm_right_torso config file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'safety_scaling',
            default_value='1.0',
            description='Global safety velocity scaling factor (0.0-1.0), applied on top of each planned trajectory without altering per-group YAML max_velocity/max_acceleration'
        ),
        OpaqueFunction(function=launch_setup)
    ])
