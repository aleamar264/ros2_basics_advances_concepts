import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from my_robot_interfaces.action import RobotPosition
from rclpy.callback_groups import ReentrantCallbackGroup

from threading import Lock


class RobotServer(Node):
    def __init__(self):
        super().__init__('robot_server')
        self.lock_group_ = Lock()
        self.action_server = ActionServer(self,
                                          RobotPosition,
                                          "robot_position",
                                          callback_group=ReentrantCallbackGroup(),
                                          cancel_callback=...,
                                          execute_callback=...,
                                          goal_callback=...
                                          )
        self.get_logger().info("Action server has been started")


def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotServer()
    rclpy.spin(robot_node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
