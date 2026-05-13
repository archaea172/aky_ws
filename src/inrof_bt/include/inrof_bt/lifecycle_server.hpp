#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "swerm_msgs/action/lifecycle.hpp"

using namespace std::placeholders;

class LifecycleServer
: public rclcpp::Node
{
public:
    LifecycleServer();

private:
    using Lifecycle = swerm_msgs::action::Lifecycle;
    using GoalHandleLifecycle = rclcpp_action::ServerGoalHandle<Lifecycle>;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Lifecycle::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLifecycle> goal_handle
    );
    void handle_accepted(const std::shared_ptr<GoalHandleLifecycle> goal_handle);
    void execute(const std::shared_ptr<GoalHandleLifecycle> goal_handle);

    rclcpp_action::Server<Lifecycle>::SharedPtr action_server_;
};
