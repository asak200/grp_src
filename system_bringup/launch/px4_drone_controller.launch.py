import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Path to your bash script
    pkg_share = get_package_share_directory('system_bringup')
    px4_script = os.path.join(pkg_share, 'scripts', 'spawn_drone.bash')

    # Start PX4
    start_px4 = ExecuteProcess(
        cmd=['bash', px4_script, 'iris_camera', '1'],  # add the arguments here
        output='screen'
    )
    MicroXRCEAgent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    px4_py_gui = Node(
        package='px4_py_gui',
        executable='px4_controller_gui',
        name='px4_controller_gui',
        output='screen'
    )
    px4_test = Node(
        package='px4_test',
        executable='offboard_control',
        name='offboard_control',
        output='screen'
    )
    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ssmr',
            '-topic', 'robot_description',
            # '-x', '0', '-y', '0', '-z', '1.0',
            # '-R', '3.14',
        ],
        output='screen',
        # namespace='ssmr',
    )

    delayed_nodes = TimerAction(
        period=15.0,
        actions=[
            px4_py_gui,
            px4_test,
            robot_spawner,
        ]
    )

    ld.add_action(start_px4)
    ld.add_action(MicroXRCEAgent)
    ld.add_action(delayed_nodes)

    return ld

