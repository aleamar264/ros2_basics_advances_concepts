import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from challenge_actions_interfaces.action import RobotPosition
from rclpy.callback_groups import ReentrantCallbackGroup

from threading import Lock


class RobotServerNode(Node):
    def __init__(self):
        super().__init__('move_robot_server')
        self.lock_group_ = Lock()
        self.robot_position_ = 50
        self.goal_handle_: ServerGoalHandle = None
        self.action_server = ActionServer(self,
                                          RobotPosition,
                                          "move_robot",
                                          callback_group=ReentrantCallbackGroup(),
                                          cancel_callbacks=self.cancel_callback,
                                          execute_callback=self.execute_goalback,
                                          goal_callback=self.goal_callback
                                          )
        self.get_logger().info("Action server has been started")
        self.get_logger().info("Robot position: " + str(self.robot_position_))\


    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request: RobotPosition.Goal):
        """
        The goal_callback function checks if a goal request is within the specified boundaries, aborts
        any existing goals, and accepts the new goal.

        :param goal_request: The parameter `goal_request` is of type `RobotPosition.Goal`. It represents
        the goal position requested by the robot
        :type goal_request: RobotPosition.Goal
        :return: either GoalResponse.REJECT or GoalResponse.ACCEPT.
        """
        self.get_logger().info("Received a goal")
        if goal_request.position not in range(0, 100) or goal_request.velocity <= 0:
            self.get_logger().error('Rejecting goal, position/velocity out of the boundaries')
            return GoalResponse.REJECT
        # Policy: preemt existing goal
        with self.lock_group_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("Abort current goal and accepting a new goal")
                self.goal_handle_.abort()
        self.get_logger().info("Accepting a new goal")
        return GoalResponse.ACCEPT

    def execute_goalback(self, goal_handle: ServerGoalHandle):
        """
        The function `execute_goalback` moves a robot to a specified position with a specified velocity
        and returns a result indicating whether the move was successful.

        :param goal_handle: The `goal_handle` parameter is an instance of the `ServerGoalHandle` class.
        It represents the handle for the current goal being executed by the server. It is used to track
        the status and progress of the goal, as well as to provide feedback and result information to
        the client
        :type goal_handle: ServerGoalHandle
        :return: an instance of the `RobotPosition.Result` class.
        """
        with self.lock_group_:
            self.goal_handle_ = goal_handle
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity
        # execute the goal
        self.get_logger().info("Moving the robot to {} with velocity of {} m/s^2".format(
            goal_position, velocity))
        feedback = RobotPosition.Feedback()
        result = RobotPosition.Result()
        while rclpy.ok():
            if not goal_handle.is_active:
                result.position = goal_position
                result.message = "Preemt by another client"
                return result
            if goal_handle.is_cancel_requested:
                result.position = goal_position
                if goal_position == self.robot_position_:
                    result.message = "Succeess"
                    goal_handle.succeed()
                else:
                    result.message = "Canceled"
                    goal_handle.canceled()
                return result
            diff = goal_position - self.robot_position_
            if diff == 0:
                result.position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result
            elif diff > 0:
                if diff >= velocity:
                    self.robot_position_ += velocity
                else:
                    self.robot_position_ += diff
            else:
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ -= abs(diff)
            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("Robot position: " + str(self.robot_position_))
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotServerNode()
    rclpy.spin(robot_node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
