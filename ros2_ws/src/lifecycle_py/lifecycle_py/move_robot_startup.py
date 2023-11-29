import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class LifeCycleManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.declare_parameter("managed_node_name",
                               rclpy.Parameter.Type.STRING_ARRAY)
        node_name_list = self.get_parameter("managed_node_name").value
        self.get_logger().info("Nodes" + str(node_name_list))
        self.client_list = []
        for node_name in node_name_list:
            service_change_state_name = f"/{node_name}/change_state"
            self.client_list.append(self.create_client(
                ChangeState, service_change_state_name))

    def change_state(self, transition: Transition):
        for client in self.client_list:
            client.wait_for_service()
            request = ChangeState.Request()
            request.transition = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):
        # Uncofigured to inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("configuring OK, now Inactive")

        time.sleep(3)

        # Inactive to Active
        self.get_logger().info("Trying to swtich to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = 'activate'
        self.change_state(transition)
        self.get_logger().info("Activating OK, now activate")


def main(args=None):
    rclpy.init(args=args)
    node = LifeCycleManager()
    node.initialization_sequence()
    rclpy.shutdown()
