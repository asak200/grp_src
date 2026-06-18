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

    diff_cont_arg = DeclareLaunchArgument(
        'diff',
        default_value='false',
        description='Run the diffdrive controller'
    )

    diff_cont = LaunchConfiguration('diff')

    this_pkg_share = get_package_share_directory('system_bringup')
    pkg_share = get_package_share_directory('grp_system_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ssmr.xacro')
    robot_controllers = os.path.join(pkg_share, 'config', 'ssmr_real_controllers.yaml')

    ssmr_robot_description = ParameterValue(Command(['xacro ', urdf_path, ' real_sys:=', 'true']), value_type=str)
    params = {'robot_description': ssmr_robot_description, 'use_sim_time': False}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params]
    )

    # ---------------- Publish To MATLAB ----------------
    publish_states_to_matlab_node = Node(
        package='ugv_control_pkg',
        executable='publish_states_to_matlab',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ---------------- Controller Manager ----------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {'use_sim_time': False}],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )
    pwm_command_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pwm_command_controller'],
    )
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )


    ld.add_action(diff_cont_arg)
    ld.add_action(robot_state_publisher)
    
    ld.add_action(publish_states_to_matlab_node)
    ld.add_action(control_node)
    ld.add_action(pwm_command_spawner)
    ld.add_action(joint_broad_spawner)
    
    return ld

