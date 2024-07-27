#!/usr/bin/env python3
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from functools import partial


class AddTwoIntsClientNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("add_two_ints_client")  # MODIFY NAME
        self.call_add_two_inst_server(6, 7)

    def call_add_two_inst_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        request = AddTwoInts.Request()
        request.a = 3
        request.b = 8

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_insts, a=a, b=b))

    def callback_call_add_two_insts(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")
        except Exception as e:
            self.get_logger().error("Service vall failure %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
