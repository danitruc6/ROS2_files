#!/usr/bin/env python3
import math
import rclpy
import random
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawnerNode(Node):  
    def __init__(self):
        super().__init__("turtle_spawner")  
        self.declare_parameter("spawn_frequency", 1.0)
        self.spawn_frequency = self.get_parameter("spawn_frequency").value

        self.spawn_turtle_timer_ = self.create_timer(
            1.0 / self.spawn_frequency, self.spawn_new_turtle
        )
        self.turtles_alive_publisher = self.create_publisher(
            TurtleArray, "alive_turtles", 10
        )
        self.alive_turtles_ = []

        # Initialize catch turttle server
        self.catch_turtle_service = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()

        msg.turtles = self.alive_turtles_
        self.turtles_alive_publisher.publish(msg)

    def spawn_new_turtle(self):
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.turtle_spawner_server(x, y, theta)

    def turtle_spawner_server(self, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for /spawn service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        # Generate a random floating-point number between 0.0 and 11.0
        request.name = ""  # The node wil handle the name since it's optional

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, x=request.x, y=request.y, theta=theta)
        )

    def callback_call_spawn(self, future, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(
                    f"Successfully spawned a turtle: {response.name} at {x}, {y}, {theta}  "
                )
                # Add the new created turtle to the list of alive_turtles_
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Failed to span turtle: {e}")

    def call_kill_server(self, turtle_name):
        client = self.create_client(Kill, "kill")

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /kill service...")

        request = Kill.Request()
        request.name = turtle_name

        self.get_logger().info(f"Killing turtle {turtle_name}")
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill, turtle_name=turtle_name)
        )

    def callback_call_kill(self, future, turtle_name):
        try:
            future.result
            self.get_logger().info("Successfully killed turtle")
            for i, turtle in enumerate(self.alive_turtles_):
                if turtle_name == turtle.name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break

        except Exception as e:
            self.get_logger().error(f"Failed to kill turtle: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()  
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
