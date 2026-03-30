"""
Full detection pipeline for Assignment 8.

Launches:
  1. RealSense D435i — publishes RGB remapped to /image + /camera_info,
     and depth on /camera/aligned_depth_to_color/image_raw
  2. YOLOv8 TensorRT pipeline — reads /image, publishes /detections

Then separately run:
  ros2 run object_tracker_server object_tracker_server
  ros2 run object_tracker_client object_tracker_client
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ── Paths ─────────────────────────────────────────────────────────────────────
PLAN_PATH = (
    '/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/yolov8s.plan'
)


def generate_launch_description():

    # ── 1. RealSense camera node ──────────────────────────────────────────────
    # Remaps color image → /image and camera_info → /camera_info so YOLOv8
    # encoder can find them without modification.
    # Depth is published on /camera/aligned_depth_to_color/image_raw (used by
    # the action server).
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'enable_color':          True,
            'enable_depth':          True,
            'align_depth.enable':    True,
            'rgb_camera.profile':    '640x480x15',
            'depth_module.profile':  '640x480x15',
        }],
        remappings=[
            ('/camera/color/image_raw',  '/image'),
            ('/camera/color/camera_info', '/camera_info'),
        ],
    )

    # ── 2. YOLOv8 TensorRT pipeline ───────────────────────────────────────────
    # yolov8_tensor_rt.launch.py handles: DNN encoder → TensorRT → YOLOv8
    # decoder → publishes vision_msgs/Detection2DArray on /detections.
    #
    # YOLOv8s binding names:  input = "images",  output = "output0"
    # Network input size:     640 × 640
    yolov8_dir = get_package_share_directory('isaac_ros_yolov8')
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolov8_dir, 'launch', 'yolov8_tensor_rt.launch.py')
        ),
        launch_arguments={
            'model_file_path':      '',           # leave empty — using pre-built plan
            'engine_file_path':     PLAN_PATH,
            'input_binding_names':  '["images"]',
            'output_binding_names': '["output0"]',
            'input_tensor_names':   '["input_tensor"]',
            'output_tensor_names':  '["output_tensor"]',
            'input_image_width':    '640',
            'input_image_height':   '480',
            'network_image_width':  '640',
            'network_image_height': '640',
            'image_mean':           '[0.0, 0.0, 0.0]',
            'image_stddev':         '[1.0, 1.0, 1.0]',
            'confidence_threshold': '0.25',
            'nms_threshold':        '0.45',
            'force_engine_update':  'False',
            'verbose':              'False',
        }.items(),
    )

    return LaunchDescription([
        realsense_node,
        yolov8_launch,
    ])
