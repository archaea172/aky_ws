#include "leader_pos_server_mppi.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

LeaderPosServer::LeaderPosServer()
: rclcpp::Node("leader_pos_server"),
publish_rate_ms(50)
{
    this->declare_parameter<double>("mppi.control_frequency", 20.0);
    this->declare_parameter<double>("mppi.predict_resolution", 0.02);
    this->declare_parameter<int>("mppi.predict_horizon", 100);
    this->declare_parameter<int>("mppi.sample_num", 200);
    this->declare_parameter<std::vector<double>>("mppi.covariance", {1.0, 0.0, 0.0, 1.0});
    this->declare_parameter<double>("mppi.lambda", 5.0);
    this->declare_parameter<double>("mppi.gamma", 0.0);
    this->declare_parameter<double>("mppi.max_v", 1.5);
    this->declare_parameter<double>("mppi.weights.w_goal", 1.0);
    this->declare_parameter<double>("mppi.weights.w_leader_goal", 1.0);
    this->declare_parameter<double>("mppi.weights.w_linear", 100.0);
    this->declare_parameter<int>("mppi.boid.boid_num", 5);
    this->declare_parameter<double>("mppi.boid.max_vel", 0.05);
    this->declare_parameter<double>("mppi.boid.ir", 100.0);
    this->declare_parameter<double>("mppi.boid.ir_min", 0.01);
    this->declare_parameter<double>("mppi.boid.k_separation", 20.0);
    this->declare_parameter<double>("mppi.boid.k_alignment", 1.1);
    this->declare_parameter<double>("mppi.boid.k_gravity", 0.5);
    this->declare_parameter<double>("mppi.boid.k_wall", 0.5);
    this->declare_parameter<double>("mppi.k_follow", 0.5);

    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .transient_local();
    this->boid_num_ = static_cast<int>(this->get_parameter("mppi.boid.boid_num").as_int());
    this->swerm_states.pose = Eigen::Matrix<double, 2, Eigen::Dynamic>::Zero(2, this->boid_num_);
    this->swerm_states.vel = Eigen::Matrix<double, 2, Eigen::Dynamic>::Zero(2, this->boid_num_);

    this->leader_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "leader/odometry",
        device
    );
    this->map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        map_qos,
        std::bind(&LeaderPosServer::map_callback, this, std::placeholders::_1)
    );
    for (int i = 0; i < this->boid_num_; ++i)
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
    if (!this->subscribe_map_)
    {
        RCLCPP_ERROR(this->get_logger(), "map server is not availble.");
        return rclcpp_action::GoalResponse::REJECT;
    }
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

    MppiSwermParams mppi_parameter;
    mppi_parameter.control_frequency = this->get_parameter("mppi.control_frequency").as_double();
    mppi_parameter.predict_resolution = this->get_parameter("mppi.predict_resolution").as_double();
    mppi_parameter.predict_horizon = static_cast<int>(this->get_parameter("mppi.predict_horizon").as_int());
    mppi_parameter.sample_num = static_cast<int>(this->get_parameter("mppi.sample_num").as_int());

    const auto covariance = this->get_parameter("mppi.covariance").as_double_array();
    if (covariance.size() != 4)
    {
        result->success = false;
        result->msg = "mppi.covariance must have 4 elements";
        goal_handle->abort(result);
        return;
    }
    mppi_parameter.cov << covariance[0], covariance[1], covariance[2], covariance[3];

    mppi_parameter.lambda = this->get_parameter("mppi.lambda").as_double();
    mppi_parameter.gamma = this->get_parameter("mppi.gamma").as_double();
    mppi_parameter.max_v = this->get_parameter("mppi.max_v").as_double();
    mppi_parameter.weights.w_goal = this->get_parameter("mppi.weights.w_goal").as_double();
    mppi_parameter.weights.w_leader_goal = this->get_parameter("mppi.weights.w_leader_goal").as_double();
    mppi_parameter.weights.w_linear = this->get_parameter("mppi.weights.w_linear").as_double();
    mppi_parameter.boid_parameters.boid_num = this->boid_num_;
    mppi_parameter.boid_parameters.max_vel = this->get_parameter("mppi.boid.max_vel").as_double();
    mppi_parameter.boid_parameters.Ir = this->get_parameter("mppi.boid.ir").as_double();
    mppi_parameter.boid_parameters.Ir_min = this->get_parameter("mppi.boid.ir_min").as_double();
    mppi_parameter.boid_parameters.k_separation = this->get_parameter("mppi.boid.k_separation").as_double();
    mppi_parameter.boid_parameters.k_alignment = this->get_parameter("mppi.boid.k_alignment").as_double();
    mppi_parameter.boid_parameters.k_gravity = this->get_parameter("mppi.boid.k_gravity").as_double();
    mppi_parameter.boid_parameters.k_wall = this->get_parameter("mppi.boid.k_wall").as_double();
    mppi_parameter.boid_parameters.field = this->field_;
    mppi_parameter.k_follow = this->get_parameter("mppi.k_follow").as_double();

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
    if (id < 0 || id >= this->boid_num_)
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
