import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, ThisLaunchFileDir
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

# node graph

# actual againg reference
# errors

# simulink velocity
## step-ref and error

# simulink position

# gazebo velocity
# gazebo position

# a picture with drone
# drone's cam


def generate_launch_description():
    ld = LaunchDescription()

    drone_arg = DeclareLaunchArgument(
        'drone',
        default_value='false',
        description='Run drone (PX4) instead of SSMR'
    )

    drone = LaunchConfiguration('drone')

    this_pkg_share = get_package_share_directory('system_bringup')
    pkg_share = get_package_share_directory('grp_system_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ssmr.xacro')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    world = "/media/asak/ssd/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/sonoma_raceway.world"
    # world = "/media/asak/ssd/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/baylands.world"
    world = os.path.join(pkg_share, 'worlds', 'my_world.world')
    # world = '/home/asak/grp/src/grp_system_description/worlds/small_city.world'

    ssmr_robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {"robot_description": ssmr_robot_description, 'use_sim_time': True},
        ],
        # namespace='ssmr',
    )

    # ---------------- Gazebo ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
        }.items()
    )
    

    # ---------------- Spawn SSMR ----------------
    # ros2 run gazebo_ros spawn_entity.py -entity ssmr -topic robot_description
    ssmr_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ssmr',
            '-topic', 'robot_description',
            '-x', '-10', '-y', '-8', '-z', '1.0',
            # '-R', '3.14',
        ],
        output='screen',
        # namespace='ssmr',
        condition=UnlessCondition(drone),
    )

    # ---------------- Apply Torque From Voltage ----------------
    apply_torque_node = Node(
        package='ugv_control_pkg',
        executable='apply_torque_from_voltage',
        output='screen',
    )

    # ---------------- Publish To MATLAB ----------------
    publish_states_to_matlab_node = Node(
        package='ugv_control_pkg',
        executable='publish_states_to_matlab',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ---------------- Controller Manager ----------------
    wheel_effort_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_effort_cont'],
    )
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )


    # ----------------- PX4 Env ---------------------
    delayed_px4_launcher = TimerAction(
        period=5.0,
        actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_pkg_share, 'launch', 'px4_drone_controller.launch.py')
            ),
            launch_arguments={
                'ssmr_spawner': 'true'
            }.items()
        )
        ],
        condition=IfCondition(drone)
    )


    ld.add_action(drone_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(ssmr_spawner)
    # ld.add_action(apply_torque_node)
    ld.add_action(publish_states_to_matlab_node)
    # ld.add_action(wheel_effort_cont_spawner)
    ld.add_action(diff_cont_spawner)
    ld.add_action(joint_broad_spawner)
    ld.add_action(delayed_px4_launcher)

    # ---------------- world -> map transform ----------------
    # ld.add_action(Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    # ))
    return ld

