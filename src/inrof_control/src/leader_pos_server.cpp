#include "leader_pos_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

LeaderPosServer::LeaderPosServer()
: rclcpp::Node("leader_pos_server")
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    this->leader_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "leader/odometry",
        device
    );

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

rclcpp_action::CancelResponse LeaderPosServer::handle_cancel(
    const std::shared_ptr<GoalHandleLeaderPos> goal_handle
)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LeaderPosServer::handle_accepted(const std::shared_ptr<GoalHandleLeaderPos> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    goal_handle->execute();
    std::thread{std::bind(&LeaderPosServer::execute, this, _1), goal_handle}.detach();
}

void LeaderPosServer::execute(const std::shared_ptr<GoalHandleLeaderPos> goal_handle)
{

}

bool LeaderPosServer::is_out_of_map(geometry_msgs::msg::PoseStamped start_pos, geometry_msgs::msg::PoseStamped goal_pos)
{
    static_cast<void>(start_pos);
    static_cast<void>(goal_pos);
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LeaderPosServer> node = std::make_shared<LeaderPosServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
