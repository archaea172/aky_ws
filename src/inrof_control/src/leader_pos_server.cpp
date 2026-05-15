#include "leader_pos_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

LeaderPosServer::LeaderPosServer()
: rclcpp::Node("leader_pos_server")
{
    this->action_server_ = rclcpp_action::create_server<swerm_msgs::action::LeaderPos>(
        this,
        "leader_pos",
        std::bind(&LeaderPosServer::handle_goal, this, _1, _2),
        std::bind(&LeaderPosServer::handle_cancel,this, _1),
        std::bind(&LeaderPosServer::handle_accepted, this, _1)
    );
}

rclcpp_action::GoalResponse LeaderPosServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const swerm_msgs::action::LeaderPos::Goal> goal
)
{
    if (is_out_of_map(goal->start_pos, goal->goal_pos)) 
    {
        RCLCPP_ERROR(this->get_logger(), "start_pos or goal_pos is out of map");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

bool LeaderPosServer::is_out_of_map(geometry_msgs::msg::PoseStamped start_pos, geometry_msgs::msg::PoseStamped goal_pos)
{
    return false;
}
