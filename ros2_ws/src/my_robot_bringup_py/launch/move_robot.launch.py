from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    move_robot_server_a = LifecycleNode(
        package="lifecycle_py",
        executable="move_robot_server",
        name="move_robot_server_a",
        namespace="",
        parameters=[{"robot_name": "A"}]
    )

    move_robot_server_b = LifecycleNode(
        package="lifecycle_py",
        executable="move_robot_server",
        name="move_robot_server_b",
        namespace="",
        parameters=[{"robot_name": "B"}]
    )

    lifecycle_node_manager = Node(
        package="lifecycle_py",
        executable="move_robot_startup",
        parameters=[
            {"managed_node_name": [
                'move_robot_server_a', 'move_robot_server_b']}
        ]
    )

    ld.add_action(move_robot_server_a)
    ld.add_action(move_robot_server_b)
    ld.add_action(lifecycle_node_manager)
    return ld
