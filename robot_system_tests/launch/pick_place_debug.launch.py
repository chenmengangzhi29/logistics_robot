from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_auto_pick_node = LaunchConfiguration("start_auto_pick_node")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "sim_robot_only.launch.py",
            )
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur5_warehouse_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_hardware_hal"),
                "launch",
                "gripper_hal_sim.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_perception"),
                "launch",
                "perception_only.launch.py",
            )
        )
    )

    pick_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "pick_place_server.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_auto_pick": "false",
            "use_mtc": "false",
        }.items(),
    )

    auto_pick_node = Node(
        package="robot_decision",
        executable="auto_pick_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    delayed_auto_pick_node = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg="[pick_place_debug] Starting auto_pick_node after 30s delay"),
            auto_pick_node
        ],
        condition=IfCondition(start_auto_pick_node),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_auto_pick_node", default_value="true"),
        sim_launch,
        move_group_launch,
        gripper_launch,
        perception_launch,
        pick_place_launch,
        delayed_auto_pick_node,
    ])