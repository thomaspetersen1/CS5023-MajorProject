"""Publishes the configured tour landmarks so clients like the web UI
can pick them up.

We don't use custom message types in this project, so the landmarks
go out as a latched JSON payload on /landmarks via std_msgs/String.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String

from tour_guide.landmark_manager import load_landmarks


class LandmarkPublisher(Node):
    def __init__(self) -> None:
        super().__init__("landmark_publisher")

        self.declare_parameter("landmarks_file", "")
        landmarks_path = (
            self.get_parameter("landmarks_file").get_parameter_value().string_value
        )
        if not landmarks_path:
            raise RuntimeError("landmark_publisher: landmarks_file parameter is required")

        self.landmarks = load_landmarks(landmarks_path)

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, "landmarks", latched_qos)
        self.publish_landmarks()

        self.get_logger().info(
            f"Published {len(self.landmarks)} landmarks from {landmarks_path}"
        )

    def publish_landmarks(self) -> None:
        payload = {
            "frame_id": "map",
            "landmarks": self.landmarks,
        }
        self.publisher.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
