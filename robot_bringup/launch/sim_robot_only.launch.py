import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
                            '-x', '0', '-y', '0', '-z', '0.05'],
                 output='screen')

    spawn_jsb = Node(package='controller_manager', executable='spawner',
                     arguments=['joint_state_broadcaster'],
                     parameters=[{'use_sim_time': True}])
    spawn_ur5 = Node(package='controller_manager', executable='spawner',
                     arguments=['ur5_arm_controller'],
                     parameters=[{'use_sim_time': True}])
    spawn_diff = Node(package='controller_manager', executable='spawner',
                      arguments=['diff_drive_controller'],
                      parameters=[{'use_sim_time': True}])

    # Once ur5_arm_controller is active, immediately command the arm to a
    # tucked "ready" pose. The arm will briefly sag under gravity during
    # the ~5s gap between spawn and controller activation, but the
    # trajectory controller will correct it within 2 seconds.
    send_home = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once',
             '/ur5_arm_controller/joint_trajectory',
             'trajectory_msgs/msg/JointTrajectory',
             '{joint_names: ['
             '  "ur5_shoulder_pan_joint",'
             '  "ur5_shoulder_lift_joint",'
             '  "ur5_elbow_joint",'
             '  "ur5_wrist_1_joint",'
             '  "ur5_wrist_2_joint",'
             '  "ur5_wrist_3_joint"],'
             ' points: [{positions:'
             '  [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],'
             '  time_from_start: {sec: 3, nanosec: 0}}]}'],
        output='screen'
    )

    return LaunchDescription([
        gazebo, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb,
            on_exit=[spawn_ur5, spawn_diff])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_ur5,
            on_exit=[send_home])),
    ])
