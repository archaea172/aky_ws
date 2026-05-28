#include "ros/follower_node.hpp"

#include <cmath>

using namespace std::chrono_literals;

follower_node::follower_node()
: rclcpp_lifecycle::LifecycleNode("follower_node")
{
    this->declare_parameter<int>("boid_num", 20);
    this->declare_parameter<double>("max_vel", 0.05);
    this->declare_parameter<double>("Ir", 100);
    this->declare_parameter<double>("Ir_min", 0.01);
    this->declare_parameter<double>("k_separation", 20.0);
    this->declare_parameter<double>("k_alignment", 1.1);
    this->declare_parameter<double>("k_gravity", 0.5);
    this->declare_parameter<double>("k_follow", 0.5);
    this->declare_parameter<double>("k_wall", 0.5);
}

follower_node::CallbackReturn follower_node::on_configure(const rclcpp_lifecycle::State &state)
{
    this->boid_params_.boid_num = this->get_parameter("boid_num").as_int();
    this->boid_params_.max_vel = this->get_parameter("max_vel").as_double();
    this->boid_params_.Ir = this->get_parameter("Ir").as_double();
    this->boid_params_.Ir_min = this->get_parameter("Ir_min").as_double();
    this->boid_params_.k_separation = this->get_parameter("k_separation").as_double();
    this->boid_params_.k_alignment = this->get_parameter("k_alignment").as_double();
    this->boid_params_.k_gravity = this->get_parameter("k_gravity").as_double();
    this->boid_params_.k_wall = this->get_parameter("k_wall").as_double();
    this->k_follow_ = this->get_parameter("k_follow").as_double();
    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&follower_node::parameters_callback, this, std::placeholders::_1)
    );

    this->pos_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_params_.boid_num);
    this->vel_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_params_.boid_num);
    this->leader_pos_ = Eigen::Vector2d::Zero();

    rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .transient_local();

    this->subscribe_map_ = false;
    this->map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        map_qos,
        std::bind(&follower_node::map_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
        get_logger(),
        "on_configure() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

follower_node::CallbackReturn follower_node::on_activate(const rclcpp_lifecycle::State &state)
{
    if (!subscribe_map_)
    {
        RCLCPP_WARN(this->get_logger(), "map server is not available!");
        return CallbackReturn::FAILURE;
    }
    this->follower_core_ = std::make_unique<FollowerCore>(boid_params_, k_follow_);
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    this->cmd_vel_publishers_.reserve(this->boid_params_.boid_num);
    for (int i = 0; i < this->boid_params_.boid_num; ++i)
    {
        this->cmd_vel_publishers_.push_back(
            this->create_publisher<geometry_msgs::msg::Twist>(
                "robot_" + std::to_string(i) + "/cmd_vel",
                device
            )
        );

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
    this->leader_pos_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "leader/odometry",
        device,
        std::bind(&follower_node::leader_odom_callback, this, std::placeholders::_1)
    );
    this->control_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        50ms,
        std::bind(&follower_node::control_callback, this)
    );
    
    RCLCPP_INFO(
        get_logger(),
        "on_activate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

follower_node::CallbackReturn follower_node::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->control_stop();
    this->control_timer_.reset();
    this->cmd_vel_publishers_.clear();
    this->odom_subscribers_.clear();
    this->leader_pos_subscriber_.reset();
    this->follower_core_.reset();
    this->control_stop();
    
    RCLCPP_INFO(
        get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

follower_node::CallbackReturn follower_node::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->map_subscriber_.reset();
    this->control_stop();
    this->pos_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_params_.boid_num);
    this->vel_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_params_.boid_num);
    this->leader_pos_ = Eigen::Vector2d::Zero();
    
    RCLCPP_INFO(
        get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

follower_node::CallbackReturn follower_node::on_error(const rclcpp_lifecycle::State &state)
{
    this->control_stop();
    
    RCLCPP_INFO(
        get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

follower_node::CallbackReturn follower_node::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->control_stop();
    
    RCLCPP_INFO(
        get_logger(),
        "on_shutdown() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult follower_node::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    BoidPrams next_boid_params = this->boid_params_;
    double next_k_follow = this->k_follow_;

    for (const auto &parameter : parameters)
    {
        const std::string &name = parameter.get_name();

        if (name == "boid_num")
        {
            result.successful = false;
            result.reason = "boid_num cannot be changed after startup";
            return result;
        }
        if (name == "max_vel")
        {
            next_boid_params.max_vel = parameter.as_double();
        }
        else if (name == "Ir")
        {
            next_boid_params.Ir = parameter.as_double();
        }
        else if (name == "Ir_min")
        {
            next_boid_params.Ir_min = parameter.as_double();
        }
        else if (name == "k_separation")
        {
            next_boid_params.k_separation = parameter.as_double();
        }
        else if (name == "k_alignment")
        {
            next_boid_params.k_alignment = parameter.as_double();
        }
        else if (name == "k_gravity")
        {
            next_boid_params.k_gravity = parameter.as_double();
        }
        else if (name == "k_follow")
        {
            next_k_follow = parameter.as_double();
        }
    }

    if (!std::isfinite(next_boid_params.max_vel) || next_boid_params.max_vel < 0.0 ||
        !std::isfinite(next_boid_params.Ir) || next_boid_params.Ir <= 0.0 ||
        !std::isfinite(next_boid_params.Ir_min) || next_boid_params.Ir_min <= 0.0 ||
        next_boid_params.Ir <= next_boid_params.Ir_min ||
        !std::isfinite(next_boid_params.k_separation) ||
        !std::isfinite(next_boid_params.k_alignment) ||
        !std::isfinite(next_boid_params.k_gravity) ||
        !std::isfinite(next_k_follow))
    {
        result.successful = false;
        result.reason = "invalid follower parameter";
        return result;
    }

    this->boid_params_ = next_boid_params;
    this->k_follow_ = next_k_follow;

    return result;
}

void follower_node::leader_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    this->leader_pos_ << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
}

void follower_node::odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    if (id < 0 || id >= this->boid_params_.boid_num)
    {
        RCLCPP_ERROR(this->get_logger(), "id is invalid. please check robot num");
        return;
    }

    this->pos_matrix_.col(id) << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->vel_matrix_.col(id) << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}

void follower_node::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr rxdata)
{
    if (subscribe_map_) return;
    subscribe_map_ = true;
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

    this->boid_params_.field = field;
}

void follower_node::control_callback()
{
    Eigen::MatrixXd cmd_vels = this->follower_core_->update_vels(pos_matrix_, vel_matrix_, this->leader_pos_);
    
    for (int i = 0; i < this->boid_params_.boid_num; ++i)
    {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = cmd_vels(0, i);
        txdata.linear.y = cmd_vels(1, i);

        this->cmd_vel_publishers_[i]->publish(txdata);
    }
}

void follower_node::control_stop()
{
    if (!rclcpp::ok(this->get_node_base_interface()->get_context())) return;

    geometry_msgs::msg::Twist txdata;
    txdata.linear.x = 0.0;
    txdata.linear.y = 0.0;
    txdata.angular.z = 0.0;
    for (auto& publisher : this->cmd_vel_publishers_)
    {
        publisher->publish(txdata);
    }
}
