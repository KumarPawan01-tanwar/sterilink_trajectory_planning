#!/usr/bin/env python3
# two_in_two_out_node.py
 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
 
# Replace these with your real message types
from std_msgs.msg import String
 
 
class Trajectory_Planning(Node):
    """
    Example node with:
      - Two inputs: topic1, topic2
      - Two outputs: topic3, topic4
 
    Behaviour:
      - On receiving messages from topic1/topic2, process them
      - Publish results to topic3/topic4
    """
 
    def __init__(self) -> None:
        super().__init__("two_in_two_out_node")
 
        # Tunable parameters
        self.declare_parameter("param1", 1.0)
        self.declare_parameter("param2", 2.0)
 
        # Internal state (if needed)
        self.last_msg1: Optional[String] = None
        self.last_msg2: Optional[String] = None
 
        # QoS for subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
 
        # Subscribers
        self.sub1 = self.create_subscription(
            String,
            "topic1",
            self.callback1,
            qos
        )
        self.sub2 = self.create_subscription(
            String,
            "topic2",
            self.callback2,
            qos
        )
 
        # Publishers
        self.pub3 = self.create_publisher(String, "topic3", 10)
        self.pub4 = self.create_publisher(String, "topic4", 10)
 
        self.get_logger().info("TwoInTwoOutNode started, listening to topic1 and topic2")
 
    def callback1(self, msg: String) -> None:
        self.get_logger().info(f"Received from topic1: {msg.data}")
        self.last_msg1 = msg
 
        # Example processing
        out_msg = String()
        out_msg.data = f"Processed topic1: {msg.data}"
        self.pub3.publish(out_msg)
 
    def callback2(self, msg: String) -> None:
        self.get_logger().info(f"Received from topic2: {msg.data}")
        self.last_msg2 = msg
 
        # Example processing
        out_msg = String()
        out_msg.data = f"Processed topic2: {msg.data}"
        self.pub4.publish(out_msg)
 
 
def main(args=None) -> None:
    rclpy.init(args=args)
    node = Trajectory_Planning()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()