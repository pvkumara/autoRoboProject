# Assignment 8 — Finding and Tracking Objects

Track any of the 80 COCO-dataset objects with the Jetson + RealSense D435i +
YOLOv8 Isaac ROS pipeline, then drive the Waveshare robot base toward the target.

**Default target object: your cell phone** (COCO class `cell phone`).

---

## Package Overview

| Package | Role |
|---|---|
| `object_tracker_interfaces` | Custom `TrackObject.action` definition |
| `object_tracker_server` | Action server — detects, tracks, publishes `/cmd_vel` |
| `object_tracker_client` | Action client — takes user input, prints feedback & result |
| `waveshare_motor_driver` | Subscribes to `/cmd_vel`, sends JSON to Waveshare base via USB serial |

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│  Isaac ROS YOLOv8 pipeline                                           │
│  /detections (vision_msgs/Detection2DArray)                          │
│  /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image 16UC1) │
└────────────────────────┬─────────────────────────────────────────────┘
                         │
             ┌───────────▼─────────────┐
             │  object_tracker_server  │◄──── TrackObject action goal
             │  State machine:         │────► TrackObject feedback/result
             │  SEARCHING / TRACKING   │
             │  SUCCESS / FAILED       │
             └───────────┬─────────────┘
                         │ /cmd_vel (Twist)
             ┌───────────▼─────────────┐
             │  waveshare_motor_driver │──── JSON over USB serial
             │  {"T":13,"X":…,"Z":…}  │────► Waveshare robot base
             └─────────────────────────┘

   object_tracker_client ──► sends goal, prints feedback & result
```

---

## Build (inside Isaac ROS Docker container)

```bash
# Copy / clone this repo into the Isaac ROS workspace
cp -r autoRoboProject/src/* /workspaces/isaac_ros-dev/src/

cd /workspaces/isaac_ros-dev
colcon build --packages-select \
    object_tracker_interfaces \
    object_tracker_server \
    object_tracker_client \
    waveshare_motor_driver \
    --symlink-install
source install/setup.bash
```

---

## Part 1 — Test without motors

### Terminal 1 — Start the YOLOv8 + RealSense pipeline
```bash
# (as per class instructions)
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_realsense.launch.py
```

### Terminal 2 — Action server
```bash
ros2 run object_tracker_server object_tracker_server
```

### Terminal 3 — Action client
```bash
ros2 run object_tracker_client object_tracker_client
# Press Enter to default to "cell phone", or type any COCO class name
```

Hold your phone in front of the RealSense camera.  
The client will print position feedback ("left", "right", "center", …)  
and a final result ("Tracking Successful!" or "Tracking Failed.").

---

## Part 2 — Full robot tracking

### Setup: add Docker user to dialout group (one-time)
```bash
sudo usermod -aG dialout admin
exit
# Restart the container, then verify:
groups   # should show: admin dialout sudo video plugdev
```

### Terminal 1 — YOLOv8 + RealSense pipeline (same as Part 1)

### Terminal 2 — Action server **with** motor control enabled via launch file
```bash
ros2 launch object_tracker_server tracker.launch.py launch_motor_driver:=true
```

### Terminal 3 — Action client
```bash
ros2 run object_tracker_client object_tracker_client
```

The robot will now:
1. **SEARCHING** — spin slowly until the target appears in frame
2. **TRACKING** — turn left/right (angular ∝ bbox_cx offset) and move forward (linear ∝ depth)
3. **SUCCESS** — stop when the object is centred (±30 px) and ≤ 20 cm away
4. **FAILED** — stop if the object disappears after tracking starts

---

## Waveshare Motor Driver — standalone use

```bash
# Default port /dev/ttyUSB0:
ros2 run waveshare_motor_driver waveshare_motor_driver

# Custom port:
ros2 run waveshare_motor_driver waveshare_motor_driver /dev/ttyUSB1
```

If the serial port cannot be opened the node falls back to **simulation mode**
and logs the JSON commands it _would_ have sent — useful for desk testing.

---

## Tracking Parameters (edit `object_tracker_server.cpp`)

| Constant | Default | Meaning |
|---|---|---|
| `CENTER_TOL` | 30 px | ±pixel radius counted as "centred" |
| `SUCCESS_DIST_M` | 0.20 m | Distance at which tracking succeeds |
| `MAX_LINEAR_VEL` | 0.3 m/s | Forward speed cap (assignment hard limit: 0.5) |
| `MAX_ANGULAR_VEL` | 1.0 rad/s | Turn-rate cap |
| `FORWARD_GAIN` | 0.5 | `linear_x = depth × gain` |
| `SEARCH_ANGULAR` | 0.5 rad/s | Spin rate while searching |

---

## COCO Classes (80 total — excerpt)

| Class | COCO index |
|---|---|
| **cell phone** | 67 |
| person | 0 |
| cup | 41 |
| bottle | 39 |
| laptop | 63 |
| book | 73 |
| toothbrush | 79 |

All 80 classes are validated by the action server; invalid names are rejected.
