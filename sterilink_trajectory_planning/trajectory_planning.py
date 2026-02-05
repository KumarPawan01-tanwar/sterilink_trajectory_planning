#!/usr/bin/env python3
"""Trajectory Planning Node for STERILINK Autonomous Mobile Robot."""
__author__  = "Hassan Abdelhadi"
__email__   = "has8385s@hs-coburg.de"
__version__ = "0.1.0"

from typing import Optional
import math  # hypot (to calculate distance to obstacles)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from derived_object_msgs.msg import ObjectArray
from ackermann_msgs.msg import AckermannDriveStamped
from sterilink_msgs.msg import TrajectoryStatus


class TrajectoryPlanning(Node):
    """
    Trajectory Planning Node.

    Subscribes to:
      - /location: Current robot pose and velocity (nav_msgs/Odometry)
      - /localized_objects: Detected dynamic obstacles (derived_object_msgs/ObjectArray)
      - /path: Global path to follow (nav_msgs/Path)

    Publishes to:
      - /motion_command: Drive commands (ackermann_msgs/AckermannDriveStamped)
      - /live_location: Actual pose for remote display (nav_msgs/Odometry)
      - /trajectory_status: Local planning status (sterilink_msgs/TrajectoryStatus)
    """

    def __init__(self) -> None:
        super().__init__("trajectory_planning_node")

        # Declare parameters
        self.declare_parameter("max_speed", 1.2)
        self.declare_parameter("max_accel", 0.5)
        self.declare_parameter("obstacle_safety_margin", 0.3)

        # Internal state
        self.current_location: Optional[Odometry] = None
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
        self.sub_location = self.create_subscription(
            Odometry,
            "/location",
            self.callback_location,
            qos_profile
        )

        self.sub_objects = self.create_subscription(
            ObjectArray,
            "/localized_objects",
            self.callback_objects,
            qos_profile
        )

        self.sub_path = self.create_subscription(
            Path,
            "/path",
            self.callback_path,
            qos_profile
        )

        # Publishers
        self.pub_motion_command = self.create_publisher(
            AckermannDriveStamped,
            "/motion_command",
            10
        )

        self.pub_live_location = self.create_publisher(
            Odometry,
            "/live_location",
            10
        )

        self.pub_trajectory_status = self.create_publisher(
            TrajectoryStatus,
            "/trajectory_status",
            10
        )

        self.get_logger().info("Trajectory Planning Node started")

    def callback_location(self, msg: Odometry) -> None:
        """Handle incoming robot location updates."""
        self.current_location = msg
        # Process the location data and compute drive commands
        self.publish_drive_commands()

    def callback_objects(self, msg: ObjectArray) -> None:
        """Handle incoming dynamic obstacle detections."""
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
        if self.current_location is None or self.path is None:
            status_msg = TrajectoryStatus()
            status_msg.status = TrajectoryStatus.NO_AVAILABLE_PATH
            status_msg.message = "Waiting for valid location/path"
            status_msg.clearance = 0.0
            status_msg.speed_cmd = 0.0
            self.pub_trajectory_status.publish(status_msg)
            return

        # Example: Simple forward drive command
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        # Half max speed
        drive_cmd.drive.speed = self.get_parameter(
            "max_speed").get_parameter_value().double_value / 2
        # Straight ahead
        drive_cmd.drive.steering_angle = 0.0

        self.pub_motion_command.publish(drive_cmd)
        self.get_logger().info(
            f"Publishing drive command: {drive_cmd.drive.speed} m/s")

        status_msg = TrajectoryStatus()
        safety_margin = self.get_parameter(
            "obstacle_safety_margin").get_parameter_value().double_value

        if self.lowest_clearance is None:
            status_msg.status = TrajectoryStatus.SUCCESS
            status_msg.message = "Trajectory command published | no obstacle"
            status_msg.clearance = 0.0
        elif self.lowest_clearance <= safety_margin:
            status_msg.status = TrajectoryStatus.LOW_CLEARANCE
            status_msg.message = "Trajectory command aborted; clearance below safety margin"
            status_msg.clearance = self.lowest_clearance
        else:
            status_msg.status = TrajectoryStatus.SUCCESS
            status_msg.message = "Trajectory command published | safe clearance"
            status_msg.clearance = self.lowest_clearance
        status_msg.speed_cmd = float(drive_cmd.drive.speed)
        self.pub_trajectory_status.publish(status_msg)

    def update_obstacles_clearance(self):
        """Update internal obstacle representation."""
        if self.detected_objects is None:
            return

        min_dist = None
        if self.current_location is not None:
            robot_x = self.current_location.pose.pose.position.x
            robot_y = self.current_location.pose.pose.position.y
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
        for pose in self.path.poses:
            self.get_logger().info(
                f"Path waypoint at {pose.pose.position.x}, {pose.pose.position.y}")


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
