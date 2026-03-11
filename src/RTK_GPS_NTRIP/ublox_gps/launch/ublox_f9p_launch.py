# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the ublox gps node with zed-f9p configuration."""

import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    show_fix_hz = LaunchConfiguration('show_fix_hz')
    ros_console_output_format = '[{severity}] [{time}] [{name}]: {message}'

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')
    params = os.path.join(config_directory, 'zed_f9p.yaml')

    declare_show_fix_hz = DeclareLaunchArgument(
        'show_fix_hz',
        default_value='true',
        description='Print /f9p/fix publish rate in this launch console')

    set_ros_log_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT',
        ros_console_output_format
    )

    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
                                             executable='ublox_gps_node',
                                             output='screen',
                                             parameters=[params],
                                             remappings=[('/ublox_gps_node/fix', '/f9p/fix'),
                                                         ('/ublox_gps_node/fix_velocity', '/f9p/fix_velocity'),]
                                             )

    fix_hz_monitor = ExecuteProcess(
        condition=IfCondition(show_fix_hz),
        cmd=['ros2', 'topic', 'hz', '/f9p/fix'],
        output='screen',
        output_format='[{this.process_description.final_name}] {line}'
    )

    return launch.LaunchDescription([declare_show_fix_hz,
                                     set_ros_log_format,
                                     ublox_gps_node,
                                     TimerAction(period=3.0, actions=[fix_hz_monitor]),

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
