#!/bin/bash
# Run inside Docker: starts the RealSense + YOLOv8 pipeline using the
# official isaac_ros_examples launch fragments.
# realsense_mono_rect_depth  — RealSense camera + depth alignment + rectification
# yolov8                     — DNN encoder → TensorRT → YOLOv8 decoder → /detections_output
source /workspaces/isaac_ros-dev/install/setup.bash

ISAAC_ROS_WS=/workspaces/isaac_ros-dev

ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
    launch_fragments:=realsense_mono_rect_depth,yolov8 \
    model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx \
    engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan
