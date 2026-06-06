from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("warehouse_robot", package_name="ur5_warehouse_moveit_config").to_moveit_configs()
    
    ld = generate_demo_launch(moveit_config)
    
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        *ld.entities,
    ])
