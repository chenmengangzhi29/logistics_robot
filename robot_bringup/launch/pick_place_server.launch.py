from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mtc = LaunchConfiguration("use_mtc")

    moveit_config = (
        MoveItConfigsBuilder(
            "warehouse_robot",
            package_name="ur5_warehouse_moveit_config"
        )
        .to_moveit_configs()
    )

    pick_place_server = Node(
        package="robot_control",
        executable="pick_place_action_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        condition = UnlessCondition(use_mtc),
    )

    pick_place_mtc_server = Node(
        package="robot_control",
        executable="pick_place_mtc_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_mtc),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_mtc",
            default_value="false",
        ),
        pick_place_server,
        pick_place_mtc_server,
        # rviz_node,
    ])