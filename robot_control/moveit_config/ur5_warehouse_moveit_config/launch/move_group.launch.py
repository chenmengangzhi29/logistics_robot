from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    capabilities = LaunchConfiguration("capabilities")

    moveit_config = (
    MoveItConfigsBuilder(
        "warehouse_robot", 
        package_name="ur5_warehouse_moveit_config"
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {"capabilities": capabilities},
        ],
        arguments=[
            "--ros-args", "--log-level", "error",
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "capabilities",
            default_value="move_group/ExecuteTaskSolutionCapability",
        ),
        move_group_node,
    ])
