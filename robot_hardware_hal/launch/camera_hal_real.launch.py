from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_real = LaunchConfiguration('use_real')
    real_image_topic = LaunchConfiguration('real_image_topic')
    real_camera_info_topic = LaunchConfiguration('real_camera_info_topic')
    out_image_topic = LaunchConfiguration('out_image_topic')
    out_camera_info_topic = LaunchConfiguration('out_camera_info_topic')
    force_frame_id = LaunchConfiguration('force_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('use_real', default_value='true'),
        DeclareLaunchArgument('real_image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('real_camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('out_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('out_camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgumnet('force_frame_id', default_value=''),

        # 增加相机驱动launch启动

        Node(
            package='robot_hardware_hal',
            executable='camera_hal_real_node',
            name='camera_hal_real_node',
            output='screen',
            condition=IfCondition(use_real),
            parameters=[{
                'real_image_topic': real_image_topic,
                'real_camera_info_topic': real_camera_info_topic,
                'out_image_topic': out_image_topic,
                'out_camera_info_topic': out_camera_info_topic,
                'force_frame_id': force_frame_id,
                'use_sim_time': False,
            }]
        )
    ])