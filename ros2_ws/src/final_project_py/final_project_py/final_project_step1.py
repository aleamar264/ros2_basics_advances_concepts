import time
from rclpy.node import Node
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlesim.srv import Spawn, Kill


class TurtleController(Node):
    def __init__(self):
        super().__init__("trutle_controller")
        cb_group_ = ReentrantCallbackGroup()
        self.declare_parameter("turtle_name",
                               rclpy.Parameter.Type.STRING)
        self.turtle_name_ = self.get_parameter("turtle_name").value
        self.spawn_turtle_client_ = self.create_client(Spawn,
                                                       "/spawn",
                                                       callback_group=cb_group_)
        self.kill_turtle_client_ = self.create_client(Kill,
                                                      "/kill",
                                                      callback_group=cb_group_)

        self.spawn_turtle()

    def spawn_turtle(self) -> None:
        self.spawn_turtle_client_.wait_for_service()
        request = Spawn.Request()
        request.name = self.turtle_name_
        request.x = 5.0
        request.y = 5.0
        self.get_logger().info("Trying to spawn turtle")
        result = self.spawn_turtle_client_.call_async(request)
        rclpy.spin_until_future_complete(self, result)
        self.get_logger().info("New spawned turtle:" + result.result().name)
        time.sleep(3)
        self.kill_turtle()

    def kill_turtle(self):
        self.kill_turtle_client_.wait_for_service()
        request = Kill.Request()
        request.name = self.turtle_name_
        self.get_logger().info("Trying to remove turtle")
        self.kill_turtle_client_.call_async(request)
        self.get_logger().info("Turtle have been remove")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
