#!/usr/bin/env python3
import rclpy
from rclpy.node import Node, ReentrantCallbackGroup
from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("led_panel_state_node")  # MODIFY NAME
        self.declare_parameter("led_states", [0, 0, 0])
        self.led_states_ = self.get_parameter("led_states").value

        self.publisher_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.led_timer_ = self.create_timer(4, self.led_panel_state_publiser)
        self.led_service_ = self.create_service(
            SetLed, "set_led", self.callback_set_led
        )
        self.get_logger().info("Led Panel status publiser has been started")

    def led_panel_state_publiser(self):
        msg = LedPanelState()
        msg.led_states = self.led_states_
        self.publisher_.publish(msg)

    def callback_set_led(self, request, response):
        led_number = request.led_number
        state = request.state

        # we start by checking if the parameter in the request are valid
        if led_number > 0 and led_number <= len(self.led_states_) and state in [0, 1]:
            self.led_states_[led_number - 1] = state
            response.success = True
            return response
        else:
            response.success = False
            self.led_panel_state_publiser()
            return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
