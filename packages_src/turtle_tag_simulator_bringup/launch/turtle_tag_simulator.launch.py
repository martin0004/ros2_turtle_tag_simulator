from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    players_spawner_node = Node(
        package="turtle_tag_simulator",
        executable="players_spawner"
    )

    tagger_control_system_node = Node(
        package="turtle_tag_simulator",
        executable="tagger_control_system"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(players_spawner_node)
    ld.add_action(tagger_control_system_node)

    return ld


