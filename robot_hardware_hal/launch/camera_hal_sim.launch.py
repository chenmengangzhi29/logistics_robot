from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    sim_image_topic = LaunchConfiguration('sim_image_topic')
    sim_camera_info_topic = LaunchConfiguration('sim_camera_info_topic')
    out_image_topic = LaunchConfiguration('out_image_topic')
    out_camera_info_topic = LaunchConfiguration('out_camera_info_topic')
    force_frame_id = LaunchConfiguration('force_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('sim_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('sim_camera_info_topic', default_value='/camera_info'),
        DeclareLaunchArgument('out_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('out_camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('force_frame_id', default_value=''),

        Node(
            package='robot_hardware_hal',
            executable='camera_hal_sim_node',
            name='camera_hal_sim_node',
            output='screen',
            condition=IfCondition(use_sim),
            parameters=[{
                'sim_image_topic': sim_image_topic,
                'sim_camera_info_topic': sim_camera_info_topic,
                'out_image_topic': out_image_topic,
                'out_camera_info_topic': out_camera_info_topic,
                'force_frame_id': force_frame_id,
                'use_sim_time': True,
            }]
        )
    ])
