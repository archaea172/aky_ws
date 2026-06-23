#include "leader_pos_server_mppi.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

LeaderPosServer::LeaderPosServer()
: rclcpp::Node("leader_pos_server"),
publish_rate_ms(50)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .transient_local();

    this->leader_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "leader/odometry",
        device
    );
    this->map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        map_qos,
        std::bind(&LeaderPosServer::map_callback, this, std::placeholders::_1)
    );
    for (int i = 0; i < 5; ++i)
    {
        this->odom_subscribers_.push_back(
            this->create_subscription<nav_msgs::msg::Odometry>(
                "robot_" + std::to_string(i) + "/odometry",
                device,
                [this, i](nav_msgs::msg::Odometry::ConstSharedPtr rxdata) {
                    this->odom_callback(i, rxdata);
                }
            )
        );
    }

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

    if (!rclcpp::ok())
    {
        result->success = false;
        result->msg = "rclcpp shutdown";
        goal_handle->abort(result);
        return;
    }

    const auto publish_period = rclcpp::Duration::from_seconds(publish_rate_ms / 1000.0);
    auto clock = this->get_clock();
    auto next_publish_time = clock->now();

    nav_msgs::msg::Odometry txdata;
    Eigen::Vector2d leader_pos;
    leader_pos << goal->start_pos.pose.position.x, goal->start_pos.pose.position.y;
    Eigen::Vector2d goal_pos;
    goal_pos << goal->goal_pos.pose.position.x, goal->goal_pos.pose.position.y;
    size_t i = 0;

    MppiSwermParams mppi_parameter;
    this->mppi_controller_ = std::make_unique<MppiSwermController>(mppi_parameter, goal_pos);

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->msg = "goal canceled";
            goal_handle->canceled(result);
            return;
        }

        txdata.header = goal->goal_pos.header;
        txdata.header.stamp = clock->now();

        leader_pos = this->mppi_controller_->controlLoop(this->swerm_states, leader_pos);
        txdata.pose.pose.position.x = leader_pos(0);
        txdata.pose.pose.position.y = leader_pos(1);

        this->leader_odom_publisher_->publish(txdata);
        
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

void LeaderPosServer::odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    if (id < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "id is invalid. please check robot num");
        return;
    }

    this->swerm_states.pose.col(id) << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->swerm_states.vel.col(id) << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}

void LeaderPosServer::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr rxdata)
{
    if (this->subscribe_map_) return;
    this->subscribe_map_ = true;
    GridMap map;
    map.height = rxdata->info.height;
    map.width = rxdata->info.width;
    map.origin_x = rxdata->info.origin.position.x;
    map.origin_y = rxdata->info.origin.position.y;
    const auto &orientation = rxdata->info.origin.orientation;
    map.origin_yaw = std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    );
    map.resolution = rxdata->info.resolution;
    map.data = rxdata->data;

    DistanceFieldMap field = convertmap_grid_to_distance(map);

    this->field_ = field;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LeaderPosServer> node = std::make_shared<LeaderPosServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
