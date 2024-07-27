#!/usr/bin/env python3
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounterNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("number_counter")  # MODIFY NAME
        self.number_ = 0
        self.counter_ = 0
        self.subscriber_ = self.create_subscription(
            Int64, "number", self.callback_number, 10
        )
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.timer_ = self.create_timer(0.5, self.publish_number_count)
        self.get_logger().info("Number counter has been started")

        # creating service server
        self.server_ = self.create_service(
            SetBool, "reset_counter", self.callback_reset_counter
        )

    def callback_reset_counter(self, request, response):
        if request.data:
            self.counter_ = 0
            response.success = True
        else:
            response.success = False
            response.message = "The counter has not been reset"
        self.get_logger().info(f"The input data is {request.data} and the response is {response.success}")
        return response

    def callback_number(self, msg):
        self.counter_ += msg.data
        self.get_logger().info(str(msg.data))

    def publish_number_count(self):
        msg = Int64()
        msg.data = self.counter_
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
