#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "swerm_msgs/srv/leader_path.hpp"
#include "swerm_msgs/action/leader_pos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class LeaderPosServer
: public rclcpp::Node
{
public:
    LeaderPosServer();

private:
    using GoalHandleLeaderPos = rclcpp_action::ServerGoalHandle<swerm_msgs::action::LeaderPos>;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const swerm_msgs::action::LeaderPos::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLeaderPos> goal_handle
    );
    void handle_accepted(const std::shared_ptr<GoalHandleLeaderPos> goal_handle);
    void execute(const std::shared_ptr<GoalHandleLeaderPos> goal_handle);

    bool is_out_of_map(geometry_msgs::msg::PoseStamped start_pos, geometry_msgs::msg::PoseStamped goal_pos);

    rclcpp_action::Server<swerm_msgs::action::LeaderPos>::SharedPtr action_server_;
};
