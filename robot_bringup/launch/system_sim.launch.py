#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Author: ChanHyeong Lee
#
# Modifications Copyright 2026 XiaoHuang
# SPDX-License-Identifier: Apache-2.0
# This file is derived from ROBOTIS turtlebot3 project and modified for logistics_robot.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution([
            FindPackageShare('robot_simulation'),
            'worlds',
            'turtlebot3_home_service_challenge.world'
        ])
    )

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether execute rviz2'),

        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.'),

        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Directory of gazebo world file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_bringup'),
                    'launch',
                    'robot_base.launch.py'
                ])
            ]),
            launch_arguments={
                'start_rviz': start_rviz,
                'prefix': prefix,
                'use_sim': use_sim,
            }.items(),
        ),

        # Include 官方 gazebo_ros的启动文件，用于真正拉起Gazebo仿真进程
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'verbose': 'false',
                'world': world,
            }.items(),
        ),

        # 把机器人实体生成到Gazebo世界里
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'turtlebot3_manipulation_system',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0'
                ],
            output='screen',
        ),
    ])
