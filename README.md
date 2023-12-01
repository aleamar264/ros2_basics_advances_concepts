# On building some feature ....

👷‍♂️👷‍♀️

# ROS2 Actions

## Why

Some features:

- Can cancel the actual executions.
- Can get feedback from the action server.
- Can handle Multiple request
- Longer actions

![[Pasted image 20231108164000.png]]

To use actions, we need first to create a package where all the actions/services/topics live. To do that, we need to create a C++ package.

When the C++ is created we proceed to create 3 directories:

- action
- srv
- msg

and modify the CMakeList.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

```

also the packages.xml

```xml
...
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
...
```

## Python

### Server

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil

import time


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_ = ActionServer(self,
                                                CountUntil,
                                                "count_until",
                                                execute_callback=self.execute_callback)
        self.get_logger().info("Action server has been started")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get the request from the goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # execute the action
        self.get_logger().info("Executing the action")
        counter = 0
        for _ in range(target_number):
            counter += 1
            self.get_logger().info(str(counter))
            time.sleep(period)

        goal_handle.succeed()
        result = CountUntil.Result()
        result.reached_number = counter
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

We can add some feature to this, like the capacity of reject or accept the goal

```python
...
class CountUntilServerNode(Node):

	...
	self.count_until_server_ = ActionServer(self,
											CountUntil,
											"count_until",
											goal_callback=self.goal_callback, #New Line
											execute_callback=self.execute_callback)

    # Accept o Reject request
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a goal")
        # Validate goal request
        if goal_request.target_number <= 0:
            self.get_logger().error('Rejecting the goal')
            return GoalResponse.REJECT
        self.get_logger().info("Accpeting the goal")
        return GoalResponse.ACCEPT
...
```

### Client

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from my_robot_interfaces.action import CountUntil


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(self,
                                                CountUntil,
                                                "count_until",
                                                )
        self.get_logger().info("Action server has been started")

    def send_goal(self, target_number: int, period: float):
        # wait the server
        self.count_until_client_.wait_for_server()
        # Create the goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        # send the goal
        self.get_logger().info("Sending goal")
        self.count_until_client_.\
            send_goal_async(goal).\
            add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().\
                add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: " + str(result.reached_number))


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(5, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

With the modification for accept/reject goal

```python
...
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")#New Line
            self.goal_handle_.get_result_async().\
                add_done_callback(self.goal_result_callback)
        else: #New Line
            self.get_logger().warn('Goal got rejected') #New Line
...
```

## State machine for the Goal

![[Pasted image 20231109110704.png]]
Full state machine
![[Pasted image 20231109110851.png]]

To reach the state of SUCCEED and ABORT, we need to modify the client:

```python
from rclpy.action.client import ClientGoalHandle, GoalStatus

...
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        self.get_logger().info("Result: " + str(result.reached_number))
...
```

We can also add a feedback on the client side, to take decision like cancel, abort o continue with the current actions

### Server side

```python
...
    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get the request from the goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # execute the action
        self.get_logger().info("Executing the action")
        #Create feedback object
        feedback = CountUntil.Feedback()
        counter = 0
        for _ in range(target_number):
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            # Publish feedback
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

...
```

### Cancel request

```python
...
 self.count_until_server_ = ActionServer(self,
								CountUntil,
								"count_until",
								goal_callback=self.goal_callback,
								cancel_callback=self.cancel_callback, # New line
								execute_callback=self.execute_callback)

def execute_callback(self, goal_handle: ServerGoalHandle):
	...
	for _ in range(target_number):
		if goal_handle.is_cancel_requested:
		self.get_logger().info('Canceling the goal')
		goal_handle.canceled()
		result.reached_number = counter
		return result
	counter += 1
	...

def cancel_callback(self, goal_handle: ServerGoalHandle):
	self.get_logger().info("Received cancel request")
	return CancelResponse.ACCEPT  # or Reject
```

### Client side

```python

    def send_goal(self, target_number: int, period: float):
	    ...
		self.count_until_client_.\
            send_goal_async(goal, feedback_callback=self.feedback_callback).\
            add_done_callback(self.goal_response_callback)
            #Add feedback callback
...
    def feedback_callback(self, feedback):
        number = feedback.feedback.current_number
        self.get_logger().info("Feedback got: " + str(number))
```

### Cancel request

This part of the code is hard coded, only test purpose

```python
...
	def send_goal(self, target_number: int, period: float):
		...
		# Only test purpose
        self.timer_ = self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self):
	    # Only test purpose
        self.get_logger().info("Send cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer = self.timer_.cancel()
```

One of the problems that have this code is that when we call the cancel request, we never reach this. This is because the spined node is stuck on the for loop. To fix this we need to introduce a new concept call executor (Multi Threading).

To implement the Multi threading we need to add on the server side the next changes

```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CountUntilServerNode(Node):
    def __init__(self):
    ...
    self.count_until_server_ = ActionServer(self,
								    ...,
								    callback_group=ReentrantCallbackGroup())

...
def main(args=None):
	...
    rclpy.spin(node, MultiThreadedExecutor())
    ...
```

![[Pasted image 20231112104115.png]]

## Command line

`ros2 action list`
`ros2 action info /<action_name>`

- example:
  `ros2 action info /count_until`

```shell
ros2 action info /count_until
	Action: /count_until
Action clients: 1
	/count_until_client
Action servers: 1
	/count_until_server
```

`ros2 action send_goal <action_name> <interface_name> <parameters>`
`ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}"`

```bash
ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}"
Waiting for an action server to become available...
Sending goal:
     target_number: 4
period: 1.3

Goal accepted with ID: 7a456e85d6e741eba8dc5662f1bd5b0d

Result:
    reached_number: 4

Goal finished with status: SUCCEEDED
```

## Multiple goals (Parallel [software])

This is already solve, using the Multithreading.

## Refuse a new goal when one is active

To reject a new goal, we need to look for the active goal and ask if is already running or active for this we add

```python
    def goal_callback(self, goal_request: CountUntil.Goal):
	    ...
	     self.goal_handle_.is_active:
			self.get_logger().warn("A goal is alrady active, rejecting new goal")
			return GoalResponse.REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
		self.goal_handle_ = goal_handle
		...


```

But his have a little flaw, is like we are looking and overwritten the same variable on different threads, for this basic case, we need to use a Lock from the Threading library

```python
...
import threading
...

class CountUntilServerNode(Node):
    def __init__(self):
		...
		self.goal_handle_: ServerGoalHandle = None
        self.goal_lock = threading.Lock()

	def goal_callback(self, goal_request: CountUntil.Goal):
        #  Policy: refuse a new goal  if current is active
        with self.goal_lock:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("A goal is alrady active, rejecting new goal")
        ...

	def execute_callback(self, goal_handle: ServerGoalHandle):
		with self.goal_lock:
			self.goal_handle_ = goal_handle
```

![[Pasted image 20231113120620.png]]

## Preempt new goal

With this policy, we abort the current goal, return the result and start the new goal.
Is very similar to [[Basics and fundamentals IV#Refuse a new goal when one is active|Refuse a new goal when one is active]], the only difference is that we abort the goal before accepting he new goal.

```python
	...
	def goal_callback(self, goal_request: CountUntil.Goal):
		if goal_request.target_number <= 0:
            self.get_logger().error('Rejecting the goal')
            return GoalResponse.REJECT
		 # Policy: preemt existing goal when receiving a new goal
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept a new goal")
                self.goal_handle_.abort()
        ...
    def execute_callback(self, goal_handle: ServerGoalHandle):
		for _ in range(target_number):
			# Policy: preemt existing goal when receiving a new goal
            if not goal_handle.is_active:
                result.reached_number = counter
                return result
        ...
```

![[Pasted image 20231113122113.png]]

## Lifercycle Nodes

### Camera example

![[Pasted image 20231119145024.png]]
Full state machine of how manage a `camera`
![[Pasted image 20231119145532.png]]

- Hardware comunication
- You want to initialize things in a specific order
- Easier reconfiguration
- Allocate resources and memory fist (C++)
- Synchronize the initialization between several nodes

-> New standard
-> Lifecycles Nodes are used on several stacks

### Example

The difference btw the Node and LifecycleNode are the intermediate states.
We change the Node -> LifecycleNode, and move the initialization to the `on_configure` and also create the destructors on `on_cleanup` and `on_shutdown`.

The publisher is changed for `create_lifecycle_publisher`, that means, when the node is not in the activate state, the publisher didn't send any data.

```python
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn, LifecyclePublisher
from example_interfaces.msg import Int64


class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("IN constructor")
        self.number_ = 1
        self.publish_freq_ = 1.0
        self.number_publisher_: LifecyclePublisher = None
        self.number_timer_ = None

    #  Create ROS2 communication, connect to HW
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN configure")
        self.number_publisher_ = self.create_lifecycle_publisher(
            Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_freq_, self.publish_number
        )
        return TransitionCallbackReturn.SUCCESS

    # Enable hardware
    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_activate")
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_deactivate")
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

```

### Pass between states

```shell
some@pc:~/$ ros2 lifecycle
get list nodes set

some@pc:~/$ ros2 lifecycle nodes
/number_publisher

some@pc:~/$ ros2 lifecycle get /number_publisher
uncofingured [1]

some@pc:~/$ ros2 lifecycle list /number_publisher
- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown

some@pc:~/$ ros2 lifecycle set /number_publisher configure
Transitioning successful
```

![[Pasted image 20231119164515.png]]

If we call a state that is not in the list

```shell
some@pc:~/$ ros2 lifecycle set /number_publisher cleanup
Unknown transition requested, available ones are:
- deactivate [4]
- shutdown [7]
```

### Errors

If you raise an Exception, always go to `on_error`. An advice, sometimes is better try-catch the error on the code that use the `on_error`.

### Lifecycle manager

```python
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class LifeCycleManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.declare_parameter("managed_node_name",
                               rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter("managed_node_name").value
        service_change_state_name = f"/{node_name}/change_state"
        self.client_ = self.create_client(
            ChangeState, service_change_state_name)

    def change_state(self, transition: Transition):
        self.client_.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):
        # Uncofigured to inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = 'configure'
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
```

### Create several nodes

In the console we need to rename the node and pass a parameter

```shell
ros2 run lifecycle_py move_robot_server --ros-args -r __node:=move_robot_server_a -p robot_name:=A
```

and in the code we need to declare this parameter

```python
	def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name_ = self.get_parameter("robot_name").value
        self.action_server = ActionServer(self,
                                          RobotPosition,
                                          "move_robot_" + self.robot_name_,
                                          callback_group=ReentrantCallbackGroup(),
                                          cancel_callback=self.cancel_callback,
                                          execute_callback=self.execute_goalback,
                                          goal_callback=self.goal_callback
                                          )

   def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.undeclare_parameter("robot_name")
        self.robot_name_ = ""
```

Using a launch file

- XML

  ```xml
  <?xml version="1.0"?>
  <launch>

      <node name="move_robot_server_a" pkg="lifecycle_py" exec="move_robot_server">
          <param name="robot_name" value="A" />
      </node>

      <node name="move_robot_server_b" pkg="lifecycle_py" exec="move_robot_server">
          <param name="robot_name" value="B" />
      </node>

  </launch>
  ```

- Python
  ```python
  from launch import LaunchDescription
  from launch_ros.actions import LifecycleNode
  from launch_ros.actions import Node
  ```

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
    ```

## ROS2 Executors

Main callbacks on ROS2:

- timers
- subscribers
- service servers
- action servers
- futures (clients)

### Single thread executors

When we use a single thread executors we block all the program meanwhile the callback is executing, this have some advantages:

- Accessing hardware (only one a time)
- Accessing a variables that can be change only one a time.

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time


class Node1(Node):
    def __init__(self):
        super().__init__("node1")
        self.timer1_ = self.create_timer(1.0, self.callback_timer1)
        self.timer2_ = self.create_timer(1.0, self.callback_timer2)
        self.timer3_ = self.create_timer(1.0, self.callback_timer3)

    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1")

    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2")

    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3")


def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    # rclpy.spin(node1)
    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.spin()
    rclpy.shutdown()

```

### Multithread executors

The `ReentrantCallbackGroup` launch all the callback registered on the same group in parallel.
The `MutuallyExclusiveCallbackGroup` is like a single thread

## Components

This for the moment is only for C++

### HPP file

```cpp
#include "example_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_namespace
{

class NumberPublisher : public rclcpp::Node
{
  public:
    NumberPublisher(const rclcpp::NodeOptions &options);

  private:
    void publishNumber();
    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
};
} // namespace my_namespace
```

### CPP file

```cpp
#include "components_cpp/number_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace my_namespace
{

NumberPublisher::NumberPublisher(const rclcpp::NodeOptions &options) : Node("number_publisher", options)
{
    number_ = 2;

    number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    number_timer_ = this->create_wall_timer(1000ms, std::bind(&NumberPublisher::publishNumber, this));
    RCLCPP_INFO(this->get_logger(), "Number publisher has been started");
}

void NumberPublisher::publishNumber()
{
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    number_publisher_->publish(msg);
    number_++;
}

}; // namespace my_namespace
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::NumberPublisher);
```

### CMake file

```cmake
cmake_minimum_required(VERSION 3.8)
project(components_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)

# hpp files
include_directories(include)
## Component section ##
add_library(number_pub_component SHARED src/number_publisher.cpp)
ament_target_dependencies(number_pub_component rclcpp rclcpp_components example_interfaces)
rclcpp_components_register_nodes(number_pub_component "my_namespace::NumberPublisher")
## End Component section ##

add_executable(manual_composition src/manual_composition.cpp src/node1.cpp src/node2.cpp)
ament_target_dependencies(manual_composition rclcpp)

## Component section ##
install(TARGETS
  number_pub_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  )
## End Component section ##

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}/)
ament_package()

```

### Package

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>components_cpp</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="alejandroamar66@gmail.com">arthemis</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>example_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Command lines

```shell
ros2_ws git:(main) ✗ ros2 run rclcpp_components component_container
### When we load the component the next text is loaded
[INFO] [1701026713.565105687] [ComponentManager]: Load Library: /home/arthemis/Documents/Robotics/ros2_lvl3/ros2_ws/install/components_cpp/lib/libnumber_pub_component.so
[INFO] [1701026713.566198422] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<my_namespace::NumberPublisher>
[INFO] [1701026713.566265022] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<my_namespace::NumberPublisher>
[INFO] [1701026713.572687567] [number_publisher]: Number publisher has been started
```

To load the component we need to follow `ros2 component load <topic_container> <pkg> <namespace>`

```shell
ros2_ws git:(main) ✗ ros2 component load /ComponentManager components_cpp my_namespace::NumberPublisher
```

We also can rename the node on the container

```shell
ros2_ws git:(main) ✗ ros2 component load /ComponentManager components_cpp my_namespace::NumberPublisher -r __node:=abc
```

```shell
➜  ros2_ws git:(main) ✗ ros2 component list
/ComponentManager
  1  /number_publisher
  2  /abc
```

# Final Project

Step 1 - 2

```shell
ros2 run turtlesim turtlesim_node
```

```shell
ros2 run final_project_py  final_project  --ros-args -p turtle_name:=abc
```

```shell
ros2 action send_goal /move_turtle_abc my_robot_interfaces/action/MoveTurtle "{linear_vel_x: 1.7, angular_vel_z: 1.1, duration_sec: 4}"
```
