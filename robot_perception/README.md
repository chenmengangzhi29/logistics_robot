# robot_perception

Dual-backend perception
- `aruco_detector` (default, deterministic)
- `yolo_zmq_detector` (sends JPEG -> Windows YOLO, receives pose) - see `legacy/` for the v0 skeleton

Subscribes: `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`
Publishes: `/perception/detected_objects` (`robot_interfaces/msg/DetectedObjectArray`)