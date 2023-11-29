from datetime import datetime
import time

from rclpy.node import Node
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from rclpy.action.server import ServerGoalHandle
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import MoveTurtle
from threading import Lock


class TurtleController(Node):
    def __init__(self):
        super().__init__("trutle_controller")
        cb_group_ = ReentrantCallbackGroup()
        self.lock = Lock()
        self.goal_handle_: ServerGoalHandle | None = None
        self.declare_parameter("turtle_name", rclpy.Parameter.Type.STRING)
        self.turtle_name_ = self.get_parameter("turtle_name").value
        self.spawn_turtle_client_ = self.create_client(
            Spawn, "/spawn", callback_group=cb_group_
        )
        self.kill_turtle_client_ = self.create_client(
            Kill, "/kill", callback_group=cb_group_
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "".join(("/", self.turtle_name_, "/cmd_vel")), 10
        )

        self.spawn_turtle()

        self.move_turtle_server = ActionServer(
            self,
            MoveTurtle,
            "".join(("move_turtle_", self.turtle_name_)),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accept_callback,
            execute_callback=self.execute_callback,
            callback_group=cb_group_,
        )

        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal: MoveTurtle.Goal) -> GoalResponse:
        """
        The function `goal_callback` checks if a goal is valid and rejects it if it doesn't meet certain
        criteria.

        :param goal: The goal parameter is an instance of the MoveTurtle.Goal class. It contains the
        following attributes:
        :type goal: MoveTurtle.Goal
        :return: the value `GoalResponse.REJECT`
        """

        self.get_logger().info("Received a new goal")
        # Policy
        with self.lock:
            if self.goal_handle_ and self.goal_handle_.is_active:
                self.get_logger().warn("Current goal still active")
                return GoalResponse.REJECT

        # Valid goal
        if (
            abs(goal.linear_vel_x) > 3.0
            or abs(goal.angular_vel_z) > 2.0
            or goal.duration_sec <= 0.0
        ):
            self.get_logger().warn("Invalid goal")
            GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def handle_accept_callback(self, goal_hanlde: ServerGoalHandle) -> None:
        with self.lock:
            if self.goal_handle_ is None:
                goal_hanlde.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle) -> MoveTurtle.Result:
        with self.lock:
            self.goal_handle_ = goal_handle

        linear_x = goal_handle.request.linear_vel_x
        angular_z = goal_handle.request.angular_vel_z
        duration_sec = goal_handle.request.duration_sec

        result = MoveTurtle.Result()
        start = datetime.now()

        while rclpy.ok():
            if (datetime.now() - start).seconds > duration_sec:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(msg)
                result.message = "Success"
                result.success = True
                goal_handle.succeed()
                return result
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.cmd_vel_publisher.publish(msg)
            time.sleep(0.1)
        return result

    def spawn_turtle(self) -> None:
        """
        The function `spawn_turtle` spawns a turtle at coordinates (5.0, 5.0).
        """
        self.spawn_turtle_client_.wait_for_service()
        request = Spawn.Request()
        request.name = self.turtle_name_
        request.x = 5.0
        request.y = 5.0
        self.get_logger().info("Trying to spawn turtle")
        result = self.spawn_turtle_client_.call_async(request)
        rclpy.spin_until_future_complete(self, result)
        self.get_logger().info("New spawned turtle:" + result.result().name)

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
