from rclpy.node import Node
import rclpy
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.action import ActionClient
from challenge_actions_interfaces.action import RobotPosition
from example_interfaces.msg import Empty


class RobotClientNode(Node):
    def __init__(self):
        super().__init__("robot_client_node")
        self.goal_handle_ = None
        self.client_ = ActionClient(self,
                                    RobotPosition,
                                    "move_robot")
        self.cancel_subscriber = self.create_subscription(
            Empty,
            "cancel_move",
            self.callback_cancel_move,
            10
        )

    def send_goal(self, position: int, velocity: int):
        self.client_.wait_for_server()
        goal = RobotPosition.Goal()
        goal.position = position
        goal.velocity = velocity

        self.client_.send_goal_async(goal, self.goal_feedback).\
            add_done_callback(
                self.goal_response_callback)

    def goal_response_callback(self,  future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().\
                add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Succeeded')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Canceled')
        self.get_logger().info('Position: ' + str(result.position))
        self.get_logger().info('Message: ' + str(result.message))

    def goal_feedback(self, feedback_message):
        position = feedback_message.feedback.current_position
        self.get_logger().info("Feedback_position: " + str(position))

    def callback_cancel_move(self, msg):
        self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.goal_handle_.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = RobotClientNode()
    node.send_goal(76, 7)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
