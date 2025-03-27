#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to ROS2 parameters YAML file.'
    )

    env_actions = [
        SetEnvironmentVariable('EMAIL_USERNAME_NOTIFICATION', os.getenv('EMAIL_USERNAME_NOTIFICATION', '')),
        SetEnvironmentVariable('EMAIL_PASSWORD_NOTIFICATION', os.getenv('EMAIL_PASSWORD_NOTIFICATION', '')),
        SetEnvironmentVariable('EMAIL_RECIPIENTS_NOTIFICATION', os.getenv('EMAIL_RECIPIENTS_NOTIFICATION', '')),
        SetEnvironmentVariable('USE_EMAIL_NOTIFIER', os.getenv('USE_EMAIL_NOTIFIER', 'false')),
        SetEnvironmentVariable('USE_FIREBASE_NOTIFIER', os.getenv('USE_FIREBASE_NOTIFIER', 'false')),
        SetEnvironmentVariable('DEVICE_TOKENS_FIREBASE', os.getenv('DEVICE_TOKENS_FIREBASE', '')),
        SetEnvironmentVariable('DB_ADDRESS', os.getenv('DB_ADDRESS', 'localhost')),
        SetEnvironmentVariable('DB_PORT', os.getenv('DB_PORT', '8086')),
        SetEnvironmentVariable('CHIRPSTACK_API_KEY', os.getenv('CHIRPSTACK_API_KEY', '')),
    ]

    regular_node = Node(
        package='wisevision_data_black_box',
        executable='black_box',
        name='black_box_node',
        output='screen'
        )

    container = ComposableNodeContainer(
        name='ComponentManager',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='wisevision_action_executor',
                plugin='AutomaticActionService',
                name='automatic_action_service',
            ),
            ComposableNode(
                package='wisevision_gps_tools',
                plugin='GpsDeviceManager',
                name='gps_device_manager_node',
            ),
            ComposableNode(
                package='wisevision_notification_manager',
                plugin='NotificationHandler',
                name='notifications_handler',
                parameters=[LaunchConfiguration('config_file')],
            ),
            ComposableNode(
                package='wisevision_lorawan_bridge',
                plugin='wisevision::LoraWanBridge',
                name='lorawan_bridge',
                parameters=[LaunchConfiguration('config_file')],
            ),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        *env_actions,
        container,
        regular_node
    ])