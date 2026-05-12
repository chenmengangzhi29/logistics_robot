#!/use/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_source import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    backend = LaunchConfiguration('backend') # sim / real
    use_sim_time = LaunchConfiguration('use_sim_time')

    hal_pkg = FindPackageShare('robot_hardware_hal')
    realsense_launch = FindPackageShare('realsense2_camera')

    # 仿真相机HAL
    camera_hal_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([hal_pkg, 'launch', 'camera_hal_sim.launch.py'])
        ]),
        condition=LaunchConfigurationEquals('backend', 'sim'),
        launch_arguments={
            'use_sim_time': 'true',
            'sim_image_topic': '/image_raw',
            'sim_camera_info_topic': '/camera_info',
            'out_image_topic': '/camera/image_raw',
            'out_camera_info_topic': '/camera/camera_info',
        }.items(),
    )

    # 真实相机驱动
    real_camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([realsense_pkg, 'launch', 'rs_launch.py'])
        ]),
        condition=LaunchConfigurationEquals('backend', 'real'),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items(),
    )

    camera_hal_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([hal_pkg, 'launch', 'camera_hal_real.launch.py'])
        ]),
        condition=LaunchConfigurationEquals('backend', 'real'),
        launch_arguments={
            'use_sim_time': 'false',
            'real_image_topic': 'camera/color/image_raw',
            'real_camera_info_topic': 'camera/color/camera_info',
            'out_image_topic': '/camera/image_raw',
            'out_camera_info_topic': 'camera/camera_info',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='sim',
            description='camera backend: sim or real'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='use sim time or not'
        ),

        camera_hal_sim,
        real_camera_driver,
        camera_hal_real,
    ])


