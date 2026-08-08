#!/usr/bin/env python3
"""Publish a contract-correct 20x2 body-frame path for deployment smoke tests."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class PathPublisher(Node):
    def __init__(self, topic: str, rate: float, spacing: float, lateral: float) -> None:
        super().__init__("joint_graph_path_smoke")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Float32MultiArray, topic, qos)
        self.message = Float32MultiArray()
        self.message.layout.dim = [
            MultiArrayDimension(label="point", size=20, stride=40),
            MultiArrayDimension(label="xy", size=2, stride=2),
        ]
        self.message.layout.data_offset = 0
        self.message.data = [
            coordinate
            for index in range(20)
            for coordinate in ((index + 1) * spacing, lateral)
        ]
        self.timer = self.create_timer(1.0 / rate, self.publish)

    def publish(self) -> None:
        self.publisher.publish(self.message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/stepit/path_points_body")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--spacing", type=float, default=0.0,
                        help="Forward spacing in metres; keep 0 for the first hardware test")
    parser.add_argument("--lateral", type=float, default=0.0)
    args = parser.parse_args()
    if args.rate <= 0.0 or args.spacing < 0.0:
        parser.error("rate must be positive and spacing must be non-negative")

    rclpy.init()
    node = PathPublisher(args.topic, args.rate, args.spacing, args.lateral)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
