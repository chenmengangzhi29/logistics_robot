# WarehouseMate Smoketest Runbook

## Terminals
T1: `ros2 launch robot_bringup sim_robot_only.launch.py`
T2: `ros2 launch ur5_warehouse_moveit_config move_group.launch.py use_sim_time:=true`
T3: `ros2 launch robot_hardware_hal gripper_hal_sim.launch.py use_sim_time:=true`
T4: `ros2 launch robot_perception perception_only.launch.py`
T5(auto): `ros2 launch robot_bringup pick_place_server.launch.py use_sim_time:=true start_auto_pick:=true`

T6(verify): `rviz2` with TF, RobotModel, MarkerArray
T7: `ros2 action send_goal -f /pick_and_place robot_interfaces/action/PickAndPlace\
    "{pick_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.5, y: 0.0, z: 0.125}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}},\
    place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.0, y: -0.5, z: 0.125}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}}" `    

## Manual pick goal
