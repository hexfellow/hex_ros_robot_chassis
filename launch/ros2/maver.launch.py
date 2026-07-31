#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-07-31
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "hex_ros_robot_chassis"
    urdf_pkg_path = FindPackageShare("hex_ros_urdf_maver_x4")
    chassis_pkg_path = FindPackageShare("hex_ros_robot_chassis")
    

    # args
    robot_host_arg = DeclareLaunchArgument(
        name='robot_host',
        default_value='192.168.1.100',
        description='Robot controller IP address')
    robot_port_arg = DeclareLaunchArgument(
        name='robot_port',
        default_value='8439',
        description='Robot controller WebSocket port')

    rviz_arg = DeclareLaunchArgument(name='rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to turn on rviz')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to use simulation time (/clock)')

    # robot node
    robot_param_path = FindPackageShare(package_name).find(
        package_name) + '/config/ros2/maver_params.yaml'
    robot_node = Node(package=package_name,
                      executable='hex_ros_robot_maver',
                      name='hex_ros_robot_maver',
                      output="screen",
                      emulate_tty=True,
                      parameters=[
                          robot_param_path,
                          {
                              'robot_host': LaunchConfiguration('robot_host'),
                              'robot_port': LaunchConfiguration('robot_port'),
                          },
                      ])
    
    # rviz group
    rviz_config_path = PathJoinSubstitution(
        [chassis_pkg_path, "config", "ros2", "display_maver_x4.rviz"])
    visual_urdf_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "model.urdf"])
    description_content = ParameterValue(Command(['xacro ', visual_urdf_path]),
                                         value_type=str)
    rviz_group = GroupAction(
        [
            Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 parameters=[{
                     'robot_description': description_content,
                     'use_sim_time': LaunchConfiguration('use_sim_time'),
                 }]),
            Node(
                name="rviz2",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_path],
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        robot_host_arg,
        robot_port_arg,
        rviz_arg,
        use_sim_time_arg,
        robot_node,
        rviz_group,
    ])
