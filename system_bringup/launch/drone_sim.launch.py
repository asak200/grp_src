import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    this_pkg_share = get_package_share_directory('system_bringup')
    pkg_share = get_package_share_directory('grp_system_description')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    world = os.path.join(pkg_share, 'worlds', 'empty_world.world')
    # world = os.path.join(pkg_share, 'worlds', 'small_city.world')
    # world = "/home/asak/grp/frames/gazebo_models_worlds_collection/worlds/small_city.world"

    # PX4 and ros2 bridge
    delayed_px4_launcher = TimerAction(
        period=5.0,
        actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_pkg_share, 'launch', 'px4_drone_controller.launch.py')
            ),
            launch_arguments={
                'ssmr_spawner': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_pkg_share, 'launch', 'free_camera.launch.py')
            ),
        )
        ]
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
        }.items()
    )


    ld.add_action(gazebo)
    ld.add_action(delayed_px4_launcher)
    return ld