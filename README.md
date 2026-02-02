# Trajectory Planning
### Author: Hassan Abdelhadi

## Overview
The Trajectory Planning component converts a global path into dynamically feasible motion commands for the STERILINK autonomous mobile robot. It enforces kinematic constraints, accounts for current speed, and reacts to dynamic obstacles to produce a safe, smooth local trajectory for low-level controllers.

### Responsibilities
- Follow the global path while respecting robot kinematics and velocity limits.
- Avoid dynamic obstacles detected by perception.
- Provide continuous, time-parameterized motion commands to the drive controller.
- Publish the robot pose used for visualization and monitoring.

## ROS Interfaces

| Input | Type | Description |
|---|---|---|
| /location | nav_msgs/Odometry | Current estimated pose and velocity of the robot. |
| /localized_objects | derived_object_msgs/ObjectArray | Detected dynamic obstacles positions. |
| /path | nav_msgs/Path | A sequence of waypoints (x,y) in meters representing the calculated path to be followed. |

| Output | Type | Description |
|---|---|---|
| /motion_command | ackermann_msgs/AckermannDriveStamped | Drive command for the robot. |
| /live_location | nav_msgs/Odometry | Actual pose for remote display. |

### Prerequisites
- ROS 2 (Humble or later)
- Python 3.10+
- Required packages:
  - `rclpy` — ROS 2 Python client library
  - `nav_msgs` — Standard navigation message types
  - `geometry_msgs` — Geometry message types
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

Or include it in a launch file:
```bash
ros2 launch sterilink_trajectory_planning trajectory_planning.launch.py
```

## Parameters
- `max_speed`: maximum allowed speed (default: 1.2).
- `max_accel`: maximum acceleration (default: 0.5).
- `obstacle_safety_margin`: Clearance from obstacles (default: 0.3).