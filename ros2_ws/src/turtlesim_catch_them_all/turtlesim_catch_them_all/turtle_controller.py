#!/usr/bin/env python3
import math
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleControllerNode(Node):  
    def __init__(self):
        super().__init__("turtle_controller")  

        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first = self.get_parameter(
            "catch_closest_turtle_first"
        ).value

        self.turtle_to_catch_ = None

        # Proportional controler constants
        self.kp_linear = 3
        self.kp_angular = 9.0

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self._pose_subscriber = self.create_subscription(
            Pose, "turtle1/pose", self.pose_callback, 10
        )

        # subcriber to /alive_turtles topic
        self_alive_turtles_sub = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10
        )

        # timer
        self.timer_ = self.create_timer(0.01, self.control_loop)

        # initialize the pose as None
        self.current_pose_ = None

    def pose_callback(self, msg):
        self.current_pose_ = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.current_pose_.x
                    dist_y = turtle.y - self.current_pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closest_turtle is None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle

            else:
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        # if there's no coordinate or no turtle to catch, exit
        if self.current_pose_ is None or self.turtle_to_catch_ is None:
            return

        # create twist message
        msg = Twist()
        error_x = self.turtle_to_catch_.x - self.current_pose_.x
        error_y = self.turtle_to_catch_.y - self.current_pose_.y

        target_distance = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)

        distance_tolerance = 0.1
        # angle_tolerance = 0.1

        angle_error = angle_to_target - self.current_pose_.theta

        # normalize angle error to be between [-pi, pi]
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        # position
        if target_distance > distance_tolerance:
            msg.linear.x = self.kp_linear * target_distance
            # orientation
            msg.angular.z = self.kp_angular * angle_error
        else:
            # target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        # Publish velocity command
        self.cmd_vel_publisher_.publish(msg)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /catch_turtle service...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_catch_turtle, turtle_name=turtle_name)
        )

    def callback_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f"Turtle named: {turtle_name} could not be caught"
                )

        except Exception as e:
            self.get_logger().error(f"Failed to catch turtle: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()  
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
