from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    number_publiser_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        # rename the node
        name="my_number_publisher",
        # remap the topic
        remappings=[remap_number_topic],
        parameters=[{"number_to_publish": 4, "publish_frequency": 5.0}],
    )

    counter_node = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        # rename the node
        name="my_number_counter",
        # remap the topic
        remappings=[remap_number_topic, ("number_count", "my_number_count")],
    )

    ld.add_action(number_publiser_node)
    ld.add_action(counter_node)

    return ld
