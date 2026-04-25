import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, ThisLaunchFileDir
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('grp_system_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'free_camera.xacro')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    world = "/media/asak/ssd/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/sonoma_raceway.world"

    canera_robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {"robot_description": canera_robot_description}
            ],
            namespace='free_camera',
    )
    # ---------------- Gazebo ----------------
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
        }.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
    )


    # ---------------- Spawn Camera ----------------
    camera_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='free_camera',
        arguments=[
            '-entity', 'free_camera',
            '-topic', 'robot_description',
            '-x', '4', '-y', '-80', '-z', '0.0', '-P', '1.57'
        ],
        output='screen'
    )

    pose_controller = Node(
        package='uav_nav',
        executable='camera_position_controller',
        output='screen',
    )

    ld.add_action(robot_state_publisher)
    # ld.add_action(gzserver)
    # ld.add_action(gzclient)
    ld.add_action(camera_spawner)
    ld.add_action(pose_controller)

    # ---------------- world -> map ----------------
    # ld.add_action(Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    # ))

    return ld

