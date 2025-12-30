#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('swervebot_sim'), 'model', 'swerve4bot', 'xacro', 'swervebot.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    controller_config_file = os.path.join(
        get_package_share_directory('swervebot_sim'),
        'model',
        'swerve4bot',
        'config',
        'swervebot_controller.yaml'
    )

    # gazebo
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen')

    # robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'swervebot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "steering_controller",
            "--param-file",
            controller_config_file,
        ],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--param-file",
            controller_config_file,
        ],
    )

    # swervebot driver
    swervebot_driver_node = Node(
        package="swervebot_sim",
        executable="swervebot_driver",
        name="swervebot_driver",
        output="screen",
        parameters=[{
            'front_dist': 0.5,
            'rear_dist': 0.5,
            'left_dist': 0.5,
            'right_dist': 0.5,
            'wheel_radius': 0.2,
        }]
    )

    ld = LaunchDescription()
    
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(steering_controller_spawner)
    ld.add_action(velocity_controller_spawner)

    ld.add_action(swervebot_driver_node)
    
    return ld
