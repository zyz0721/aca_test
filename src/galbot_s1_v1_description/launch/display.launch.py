#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    urdf_file = 'galbot_S1_V1_with_GE10301.urdf'  # ← 修改成你的 URDF 文件名
    pkg_path = get_package_share_directory('galbot_s1_v1_description')
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file)

    # 用 xacro 或 urdf 文件生成 robot_description
    # robot_description = Command([
    #     'xacro',
    #     urdf_path,
    # ])
    with open(urdf_path, 'r') as inf:
        robot_description_content = inf.read()

    robot_description = ParameterValue(robot_description_content, value_type=str)


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ['/joint_states']}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_pub_node,
        rviz2
    ])
