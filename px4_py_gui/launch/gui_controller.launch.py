#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    target_system_id = LaunchConfiguration('target_system_id', default=3)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    package_name = get_package_share_directory('px4_py_gui')

    ld.add_action(DeclareLaunchArgument(
            'target_system_id',
            default_value='3',
            description='System ID of the target drone.'
    ))
    ld.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
    ))


    offboard_control = Node(
        package='px4_test',
        executable='offboard_control',
        name='offboard_control',
        parameters=[{
            'target_system_id': target_system_id,
        }]
    )

    gui = Node(
        package='px4_py_gui',
        executable='px4_controller_gui',
        name='px4_controller_gui',
        parameters=[{
            'target_system_id': target_system_id,
        }]
    )

    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_name, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    
    ld.add_action(offboard_control)
    ld.add_action(gui)
    ld.add_action(joy)

    return ld
