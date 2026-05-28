import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    desc_share = get_package_share_directory('robot_description')
    sim_share = get_package_share_directory('robot_simulation')
    urdf = os.path.join(desc_share, 'urdf', 'warehouse_robot.urdf.xacro')
    world = os.path.join(sim_share, 'worlds', 'warehouse.world')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}])

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'warehouse_robot',
                            '-topic', 'robot_description',
                            '-x', '0', '-y', '0', '-z', '0.1'],
                 output='screen')

    spawn_jsb = Node(package='controller_manager', executable='spawner',
                     arguments=['joint_state_broadcaster'], parameters=[{'use_sim_time': True}])
    spawn_ur5 = Node(package='controller_manager', executable='spawner',
                     arguments=['ur5_arm_controller'], parameters=[{'use_sim_time': True}])
    spawn_diff = Node(package='controller_manager', executable='spawner',
                      arguments=['diff_drive_controller'], parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        gazebo, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb,
            on_exit=[spawn_ur5, spawn_diff])),
    ])
