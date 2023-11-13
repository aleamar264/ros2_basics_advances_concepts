import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []
        self.count_until_server_ = ActionServer(self,
                                                CountUntil,
                                                "count_until",
                                                goal_callback=self.goal_callback,
                                                handle_accepted_callback=self.handle_accepted_goal,
                                                execute_callback=self.execute_callback,
                                                cancel_callback=self.cancel_callback,
                                                callback_group=ReentrantCallbackGroup(),
                                                )
        self.get_logger().info("Action server has been started")

    # Accept o Reject request
    def goal_callback(self, goal_request: CountUntil.Goal):
        """
        The goal_callback function is a callback function that takes a goal_request parameter of type
        CountUntil.Goal.

        :param goal_request: The `goal_request` parameter is an instance of the `CountUntil.Goal` class.
        It represents the goal that the system is trying to achieve
        :type goal_request: CountUntil.Goal
        """
        self.get_logger().info("Received a goal")
        #  Policy: refuse a new goal  if current is active
        # with self.goal_lock:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().warn("A goal is alrady active, rejecting new goal")
        # return GoalResponse.REJECT
        # Validate goal request
        if goal_request.target_number <= 0:
            self.get_logger().error('Rejecting the goal')
            return GoalResponse.REJECT
        # Policy: preemt existing goal when receiving a new goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort current goal and accept a new goal")
        #         self.goal_handle_.abort()
        self.get_logger().info("Accpeting the goal")
        return GoalResponse.ACCEPT

    def handle_accepted_goal(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        The function executes an action by counting up to a target number with a specified period, while
        providing feedback and handling goal cancellation.

        :param goal_handle: The `goal_handle` parameter is an instance of the `ServerGoalHandle` class.
        It represents the handle to the current goal being executed by the action server. It is used to
        interact with the goal, such as checking for cancellation requests, publishing feedback, and
        setting the goal status (succeeded
        :type goal_handle: ServerGoalHandle
        :return: an instance of the `CountUntil.Result` class.
        """
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        # Get the request from the goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # execute the action
        self.get_logger().info("Executing the action")
        feedback = CountUntil.Feedback()
        counter = 0
        result = CountUntil.Result()
        for _ in range(target_number):
            # Policy: preemt existing goal when receiving a new goal
            if not goal_handle.is_active:
                result.reached_number = counter
                self.proccess_next_goal_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Canceling the goal')
                goal_handle.canceled()
                result.reached_number = counter
                self.proccess_next_goal_in_queue()
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        goal_handle.succeed()
        result.reached_number = counter
        self.proccess_next_goal_in_queue()
        return result

    def proccess_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        """
        The function `cancel_callback` accepts a cancel request and logs a message.

        :param goal_handle: The `goal_handle` parameter is an object that represents the current goal
        being processed by the action server. It contains information about the goal, such as its ID,
        status, and result. The `ServerGoalHandle` class is typically provided by the action server
        framework you are using
        :type goal_handle: ServerGoalHandle
        :return: the value `CancelResponse.ACCEPT`.
        """
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT  # or Reject


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
