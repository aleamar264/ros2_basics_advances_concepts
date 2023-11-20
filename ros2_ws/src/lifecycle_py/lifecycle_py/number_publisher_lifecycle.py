import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn, LifecyclePublisher
from rclpy.timer import Timer
from example_interfaces.msg import Int64


class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("IN constructor")
        self.number_ = 1
        self.publish_freq_ = 1.0
        self.number_publisher_: LifecyclePublisher = None
        self.number_timer_: Timer = None

    #  Create ROS2 communication, connect to HW
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN configure")
        self.number_publisher_ = self.create_lifecycle_publisher(
            Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_freq_, self.publish_number
        )
        self.number_timer_.cancel()
        return TransitionCallbackReturn.SUCCESS

    # Enable hardware
    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_activate")
        self.number_timer_.reset()
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_deactivate")
        self.number_timer_.cancel()
        return super().on_deactivate(previous_state)

    #  Destroy ROS2 communication
    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN clean_up")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_error")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        #  do some checks, ok -> SUCCESS, if not FAILURE
        return TransitionCallbackReturn.SUCCESS

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
