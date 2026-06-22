#include "leader_pos_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

LeaderPosServer::LeaderPosServer()
: rclcpp::Node("leader_pos_server"),
publish_rate_ms(50)
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
    const std::shared_ptr<const swerm_msgs::action::LeaderPos_Goal> goal = goal_handle->get_goal();
    swerm_msgs::action::LeaderPos::Result::SharedPtr result = std::make_shared<swerm_msgs::action::LeaderPos::Result>();

    double one_cycle_distance = goal->max_speed * publish_rate_ms / 1000.0;

    rclcpp::Client<swerm_msgs::srv::LeaderPath>::SharedPtr path_client = this->create_client<swerm_msgs::srv::LeaderPath>(
        "leader_path_service"
    );
    swerm_msgs::srv::LeaderPath::Request::SharedPtr request = std::make_shared<swerm_msgs::srv::LeaderPath::Request>();
    request->start_pos = goal->start_pos;
    request->goal_pos = goal->goal_pos;
    request->waypoints = goal->waypoints;
    request->resolution = one_cycle_distance;

    auto future = path_client->async_send_request(request);
    while (rclcpp::ok() && future.wait_for(10ms) != std::future_status::ready)
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->msg = "goal canceled";
            goal_handle->canceled(result);
            return;
        }
    }

    if (!rclcpp::ok())
    {
        result->success = false;
        result->msg = "rclcpp shutdown";
        goal_handle->abort(result);
        return;
    }

    const auto response = future.get();
    if (!response || response->route.poses.empty()) 
    {
            result->success = false;
            result->msg = "path plan failed";
            goal_handle->canceled(result);
            return;
    }

    const auto publish_period = rclcpp::Duration::from_seconds(publish_rate_ms / 1000.0);
    auto clock = this->get_clock();
    auto next_publish_time = clock->now();

    nav_msgs::msg::Odometry txdata;
    size_t i = 0;

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->msg = "goal canceled";
            goal_handle->canceled(result);
            return;
        }

        txdata.header = response->route.poses[i].header;
        txdata.header.stamp = clock->now();

        
        
        next_publish_time = next_publish_time + publish_period;
        if (!clock->sleep_until(
                next_publish_time,
                this->get_node_base_interface()->get_context()))
        {
            result->success = false;
            result->msg = "sleep interrupted";
            goal_handle->abort(result);
            return;
        }
    }
    
    result->success = true;
    result->msg = "goal succeeded";
    goal_handle->succeed(result);
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
