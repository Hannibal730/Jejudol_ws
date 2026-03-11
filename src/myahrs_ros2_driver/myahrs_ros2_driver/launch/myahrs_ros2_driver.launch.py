# Copyright 2021 CLOBOT Co., Ltd
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_dir = get_package_share_directory('myahrs_ros2_driver')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/myahrs_imu',
        description='Serial device path for myAHRS+')

    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate for myAHRS+')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(config_dir, 'rviz', 'imu_test.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    return LaunchDescription([
        declare_serial_port_cmd,
        declare_baud_rate_cmd,
        declare_rviz_config_file_cmd,
        declare_use_rviz_cmd,
        Node(
            package='myahrs_ros2_driver',
            executable='myahrs_ros2_driver',
            name='myahrs_ros2_driver',
            output='screen',
            arguments=[serial_port, baud_rate],
            parameters=[config_file]
        ),
        rviz_cmd,
    ])
