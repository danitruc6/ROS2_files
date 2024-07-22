#!/usr/bin/env python3
from example_interfaces.msg import String
import rclpy
from rclpy.node import Node


class SmarphoneNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("smartphone")  # MODIFY NAME
        self.subscriber = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10
        )
        self.get_logger().info("Smartphone has besen started")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SmarphoneNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
