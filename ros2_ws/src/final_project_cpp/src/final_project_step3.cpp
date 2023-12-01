#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

using Twist = geometry_msgs::msg::Twist;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::placeholders;
using namespace std::chrono_literals;

class TurtleController : public rclcpp_lifecycle::LifecycleNode {
public:
    TurtleController() : LifecycleNode("turtle_controller") {
        this->declare_parameter("turtle_name", rclcpp::PARAMETER_STRING);
        cb_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        server_activated_ = false;
    }

    LifecycleCallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>(
            "/spawn", rmw_qos_profile_services_default, cb_group_);
        kill_turtle_client_ = this->create_client<turtlesim::srv::Kill>(
            "/kill", rmw_qos_profile_services_default, cb_group_);
        cmd_vel_publisher =
            this->create_publisher<Twist>("/" + turtle_name_ + "/cmd_vel", 10);
        move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
            this, "move_turtle_" + turtle_name_,
            std::bind(&TurtleController::goal_callback, this, _1, _2),
            std::bind(&TurtleController::cancel_callback, this, _1),
            std::bind(&TurtleController::handle_accept_callback, this, _1),
            rcl_action_server_get_default_options(), cb_group_);
        RCLCPP_INFO(this->get_logger(), "Action server has been started");

        spawn_turtle();

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        kill_turtle();
        spawn_turtle_client_.reset();
        kill_turtle_client_.reset();
        cmd_vel_publisher.reset();
        move_turtle_server_.reset();

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        server_activated_ = true;
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        server_activated_ = false;
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        server_activated_ = false;
        kill_turtle();
        spawn_turtle_client_.reset();
        cmd_vel_publisher.reset();
        move_turtle_server_.reset();
        kill_turtle_client_.reset();

        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveTurtle::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "New goal");
        // Check if node is activate
        if (!server_activated_) {
            RCLCPP_WARN(this->get_logger(), "Action server not activate yet");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // goal policy
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_) {
                if (goal_handle_->is_active()) {
                    RCLCPP_WARN(this->get_logger(),
                                "Current goal still active");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
        }

        // Validate new goal
        if ((fabs(goal->linear_vel_x) > 3.0) ||
            (fabs(goal->angular_vel_z) > 2.0) || (goal->duration_sec <= 0.0)) {
            RCLCPP_WARN(this->get_logger(), "Invalid goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accept_callback(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }
        double linear_vel_x = goal_handle->get_goal()->linear_vel_x;
        double angular_vel_z = goal_handle->get_goal()->angular_vel_z;
        int duration_sec = goal_handle->get_goal()->duration_sec;

        auto result = std::make_shared<MoveTurtle::Result>();
        auto start = now();
        auto duration = rclcpp::Duration(duration_sec, 0);
        rclcpp::Rate rate(10);

        while (rclcpp::ok()) {
            if (now() - start > duration) {
                auto msg = Twist();
                msg.linear.x = 0;
                msg.angular.z = 0;
                cmd_vel_publisher->publish(msg);
                result->success = true;
                result->message = "Success";
                goal_handle->succeed(result);
                return;
            }
            if (!server_activated_) {
                auto msg = Twist();
                msg.linear.x = 0;
                msg.angular.z = 0;
                cmd_vel_publisher->publish(msg);
                result->success = false;
                result->message = "Aborted because server was deactivate";
                goal_handle->abort(result);
                return;
            }
            auto msg = Twist();
            msg.linear.x = linear_vel_x;
            msg.angular.z = angular_vel_z;
            cmd_vel_publisher->publish(msg);
            rate.sleep();
        }
    }

    void spawn_turtle() {
        spawn_turtle_client_->wait_for_service();
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = turtle_name_;
        request->x = 5.0;
        request->y = 5.0;
        RCLCPP_INFO(this->get_logger(), "Trying to spawn turtle");
        auto result = spawn_turtle_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "New spawned turtle: %s",
                    result.get()->name.c_str());
    }

    void kill_turtle() {
        kill_turtle_client_->wait_for_service();
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name_;
        RCLCPP_INFO(this->get_logger(), "Trying to remove turtle");
        kill_turtle_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Turtle have been remove");
    }

    std::string turtle_name_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_turtle_client_;
    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    std::mutex mutex_;
    bool server_activated_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::executors::MultiThreadedExecutor executors;
    executors.add_node(node->get_node_base_interface());
    executors.spin();
    rclcpp::shutdown();
    return 0;
}
