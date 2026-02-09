"""Trajectory Planning Node for STERILINK Autonomous Mobile Robot."""
__author__ = "Hassan Abdelhadi"
__email__ = "has8385s@hs-coburg.de"
__version__ = "0.1.0"

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from ackermann_msgs.msg import AckermannDriveStamped
from derived_object_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry, Path
from sterilink_msgs.msg import TrajectoryStatus


class TrajectoryPlanning(Node):
    """
    Trajectory Planning Node.

    Subscribes to:
      - odometry: Current robot pose and velocity (nav_msgs/Odometry)
      - localized_objects: Detected dynamic obstacles (derived_object_msgs/ObjectArray)
      - path: Global path to follow (nav_msgs/Path)

    Publishes:
      - /motion_command: Drive commands (ackermann_msgs/AckermannDriveStamped)
      - /trajectory_status: trajectory planning status (sterilink_msgs/TrajectoryStatus)
    """

    def __init__(self) -> None:
        super().__init__("trajectory_planning_node")

        # Declare parameters
        self.declare_parameter("max_speed", 1.2)
        self.declare_parameter("obstacle_safety_margin", 0.3)

        # Cache parameter values
        self.max_speed = (
            self.get_parameter("max_speed")
            .get_parameter_value().double_value
        )
        self.obstacle_safety_margin = (
            self.get_parameter("obstacle_safety_margin")
            .get_parameter_value().double_value
        )

        # Allow runtime parameter updates
        self.add_on_set_parameters_callback(self._on_parameter_update)

        # Internal state
        self.odometry: Optional[Odometry] = None
        self.detected_objects: Optional[ObjectArray] = None
        self.path: Optional[Path] = None
        self.lowest_clearance: Optional[float] = None

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.sub_odometry = self.create_subscription(
            Odometry,
            "odometry",
            self.callback_odometry,
            qos_profile
        )

        self.sub_objects = self.create_subscription(
            ObjectArray,
            "localized_objects",
            self.callback_objects,
            qos_profile
        )

        self.sub_path = self.create_subscription(
            Path,
            "path",
            self.callback_path,
            qos_profile
        )

        # Publishers
        self.pub_motion_command = self.create_publisher(
            AckermannDriveStamped,
            "motion_command",
            10
        )

        self.pub_trajectory_status = self.create_publisher(
            TrajectoryStatus,
            "trajectory_status",
            10
        )

        self.get_logger().info("Trajectory Planning Node started")

    def callback_odometry(self, msg: Odometry) -> None:
        """Handle incoming robot odometry updates."""
        self.odometry = msg
        # Process the odometry data and compute drive commands
        self.publish_drive_commands()

    def callback_objects(self, msg: ObjectArray) -> None:
        """Handle incoming localized objects detections."""
        self.detected_objects = msg
        # Update obstacle information for trajectory planning
        self.update_obstacles_clearance()

    def callback_path(self, msg: Path) -> None:
        """Handle incoming global path."""
        self.path = msg
        # Update the trajectory based on the new path
        self.update_trajectory()

    def publish_drive_commands(self):
        """Compute and publish drive commands based on current state."""
        status_msg = TrajectoryStatus()
        # Check for missing odometry
        if self.odometry is None:
            status_msg.status = TrajectoryStatus.NO_AVAILABLE_ODOMETRY
            status_msg.message = "Waiting for localization odometry data"
            self.pub_trajectory_status.publish(status_msg)
            return

        # Check for missing path (no goal provided)
        if self.path is None:
            status_msg.status = TrajectoryStatus.NO_AVAILABLE_PATH
            status_msg.message = "Waiting for global path to follow"
            self.pub_trajectory_status.publish(status_msg)
            return

        if self.lowest_clearance is None:
            status_msg.status = TrajectoryStatus.SUCCESS
            status_msg.message = "No Obstacles"
        elif self.lowest_clearance <= self.obstacle_safety_margin:
            status_msg.status = TrajectoryStatus.LOW_CLEARANCE
            status_msg.message = (
                "Trajectory command aborted; clearance below safety margin"
            )
            status_msg.clearance = self.lowest_clearance
        else:
            status_msg.status = TrajectoryStatus.SUCCESS
            status_msg.message = "Trajectory published | Safe Clearance"
            status_msg.clearance = self.lowest_clearance

        # example for now: simple forward drive command with half max speed
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.drive.speed = self.max_speed / 2
        
        # publish the drive command and trajectory status
        self.pub_motion_command.publish(drive_cmd)
        self.pub_trajectory_status.publish(status_msg)

        self.get_logger().info(
            f"Publishing straight drive command with speed: {drive_cmd.drive.speed} m/s")

    def update_obstacles_clearance(self):
        """Update internal obstacle representation."""
        if self.detected_objects is None:
            return

        min_dist = None
        if self.odometry is not None:
            robot_x = self.odometry.pose.pose.position.x
            robot_y = self.odometry.pose.pose.position.y
            for obj in self.detected_objects.objects:
                dx = obj.position.x - robot_x
                dy = obj.position.y - robot_y
                dist = math.hypot(dx, dy)
                min_dist = dist if min_dist is None else min(min_dist, dist)

        self.lowest_clearance = min_dist

    def update_trajectory(self):
        """Update the trajectory based on the global path."""
        if self.path is None:
            return

        # TODO: Implement trajectory planning logic
        for index, pose in enumerate(self.path.poses):
            if index % 10 == 0:
                self.get_logger().debug(
                    "Path waypoint at %s, %s",
                    pose.pose.position.x,
                    pose.pose.position.y,
                )

    def _on_parameter_update(self, params):
        """Validate and apply parameter updates at runtime."""
        next_max_speed = self.max_speed
        next_margin = self.obstacle_safety_margin

        for param in params:
            if param.name == "max_speed":
                next_max_speed = float(param.value)
            elif param.name == "obstacle_safety_margin":
                next_margin = float(param.value)

        if next_max_speed <= 0.0 or next_margin < 0.0:
            return SetParametersResult(
                successful=False,
                reason="max_speed must be > 0 and obstacle_safety_margin >= 0",
            )

        self.max_speed = next_max_speed
        self.obstacle_safety_margin = next_margin
        return SetParametersResult(successful=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPlanning()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
