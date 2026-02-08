# Trajectory Planning
### Author: Hassan Abdelhadi

## Overview

The Trajectory Planning component converts a global path into dynamically feasible motion commands for the STERILINK autonomous mobile robot. It enforces kinematic constraints, accounts for current speed, and reacts to dynamic obstacles to produce a safe, smooth local trajectory for low-level controllers.

### Functional Details

- Follow the global path while respecting robot kinematics and velocity limits.
- Avoid dynamic obstacles detected by perception.
- Provide continuous, time-parameterized motion commands to the drive controller.
- Publish the robot pose used for visualization and monitoring.

To achieve that, the **Dynamic Window Approach (DWA)** algorithm is to be evaluated for suitability.

## ROS Parameters
- `max_speed`: maximum allowed speed in m/s (default: 1.2).
- `obstacle_safety_margin`: clearance from obstacles in m (default: 0.3).

## ROS Interfaces

| Input | Type | Description |
|---|---|---|
| /odometry | nav_msgs/Odometry | Current estimated pose and velocity of the robot. |
| /localized_objects | derived_object_msgs/ObjectArray | Detected dynamic obstacles positions. |
| /path | nav_msgs/Path | A sequence of waypoints (x,y) in meters representing the calculated path to be followed. |

| Output | Type | Description |
|---|---|---|
| /motion_command | ackermann_msgs/AckermannDriveStamped | Drive command for the robot. |
| /trajectory_status | sterilink_msgs/TrajectoryStatus | Local planning status with diagnostic codes and flags. |

### TrajectoryStatus Message
The TrajectoryStatus message is defined in [sterilink_msgs/msg/TrajectoryStatus.msg](../sterilink_msgs/msg/TrajectoryStatus.msg) and includes:
- `status` (int32): numeric status code
- `message` (string): human-readable status detail
- `speed_cmd` (float32): commanded speed in m/s
- `clearance` (float32): clearance in meters

### Status Codes
| Code | Status | Description |
|:-----:|:--------|:--------|
| -1 | UNDEFINED | Default/uninitialized value. |
| 0 | SUCCESS | Valid trajectory found and command published. |
| 1 | NO_AVAILABLE_ODOMETRY | Odometry/localization data unavailable. |
| 2 | NO_AVAILABLE_PATH | No usable global path available to follow. |
| 3 | NO_FEASIBLE_TRAJECTORY | No feasible trajectory could be generated. |
| 4 | OBSTACLE_BLOCKING | An obstacle blocks the path. |
| 5 | LOW_CLEARANCE | Clearance below configured safety margin. |
| 6 | UNKNOWN_ERROR | Unclassified failure. |

### Prerequisites
- ROS 2 (Humble or later)
- Python 3.10+
- Required packages:
  - `rclpy` — ROS 2 Python client library
  - `nav_msgs` — Standard navigation message types
  - `derived_object_msgs` — Dynamic object detection messages
  - `ackermann_msgs` — Ackermann drive command messages

### Installation
```bash
cd ~/ros2_ws/src
# Navigate to the sterilink workspace
colcon build --packages-select sterilink_trajectory_planning
source install/setup.bash
```

### Running the Node
Launch the trajectory planning node:
```bash
ros2 run sterilink_trajectory_planning trajectory_planning_node
```