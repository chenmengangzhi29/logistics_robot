import os
import unittest
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions

class TestPickPlaceLaunch(unittest.TestCase):
    def test_gtest_process_finishes(
        self,
        proc_info,
        test_process,
    ):
        proc_info.assertWaitForShutdown(
            process=test_process,
            timeout=1800.0,
        )

def generate_test_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_auto_pick_node = LaunchConfiguration("start_auto_pick_node")
    start_test_process = LaunchConfiguration("start_test_process");
    test_share = get_package_share_directory("robot_system_tests")

    config_file = os.path.join(
        test_share,
        "config",
        "pick_place_regression.yaml",
    )
    test_executable = os.path.join(
        get_package_prefix("robot_system_tests"),
        "lib",
        "robot_system_tests",
        "pick_place_regression_gtest",
    )

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

    test_process = ExecuteProcess(
        cmd=[
            test_executable,
            "--ros-args",
            "--params-file",
            config_file,
        ],
        output="screen",
        condition=IfCondition(start_test_process),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_auto_pick_node", default_value="true"),
        DeclareLaunchArgument("start_test_process", default_value="true"),
        sim_launch,
        move_group_launch,
        gripper_launch,
        perception_launch,
        pick_place_launch,
        delayed_auto_pick_node,
        test_process,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "auto_pick_node": auto_pick_node,
        "test_process": test_process,
    }
