from launch import LaunchDescription
from launch_ros.actions import Node

from my_py_pkg import smartphone


def generate_launch_description():
    ld = LaunchDescription()
    robot_names = ["glskard", "bb8", "daneel", "jander", "c3po"]

    for robot_name in robot_names:
        node = Node(
            package="my_py_pkg",
            executable="robot_news_station",
            # rename the node
            name=f"robot_news_station_{robot_name}",
            # remap the topic
            parameters=[{"robot_name": robot_name}],
        )
        ld.add_action(node)

    smartphone_node = Node(package="my_py_pkg", executable="smartphone")
    ld.add_action(smartphone_node)

    return ld
