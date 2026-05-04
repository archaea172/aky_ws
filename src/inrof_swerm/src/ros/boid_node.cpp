#include "ros/boid_node.hpp"

using namespace std::chrono_literals;

boid_node::boid_node()
: rclcpp::Node("boid_node")
{
    this->declare_parameter<int>("boid_num", 20);
    this->boid_params_.boid_num = this->get_parameter("boid_num").as_int();
    this->declare_parameter<double>("max_vel", 0.05);
    this->boid_params_.max_vel = this->get_parameter("max_vel").as_double();
    this->declare_parameter<double>("Ir", 100);
    this->boid_params_.Ir = this->get_parameter("Ir").as_double();
    this->declare_parameter<double>("Ir_min", 0.01);
    this->boid_params_.Ir_min = this->get_parameter("Ir_min").as_double();
    this->declare_parameter<double>("k_separation", 20.0);
    this->declare_parameter<double>("k_alignment", 1.1);
    this->declare_parameter<double>("k_gravity", 0.5);
    this->boid_params_.k_separation = this->get_parameter("k_separation").as_double();
    this->boid_params_.k_alignment = this->get_parameter("k_alignment").as_double();
    this->boid_params_.k_gravity = this->get_parameter("k_gravity").as_double();

    this->boid_core_ = std::make_unique<BoidCore>(boid_params_);
    
    this->pos_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_params_.boid_num);
    this->vel_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_params_.boid_num);

    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);this->cmd_vel_publishers_.reserve(this->boid_params_.boid_num);
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
    this->control_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        50ms,
        std::bind(&boid_node::control_callback, this)
    );
}

void boid_node::odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    if (id < 0 || id >= this->boid_params_.boid_num)
    {
        RCLCPP_ERROR(this->get_logger(), "id is invalid. please check robot num");
        return;
    }

    this->pos_matrix_.col(id) << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->vel_matrix_.col(id) << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}

void boid_node::control_callback()
{
    Eigen::MatrixXd cmd_vels = this->boid_core_->update_vels(pos_matrix_, vel_matrix_);
    
    for (int i = 0; i < this->boid_params_.boid_num; ++i)
    {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = cmd_vels(0, i);
        txdata.linear.y = cmd_vels(1, i);

        this->cmd_vel_publishers_[i]->publish(txdata);
    }
}
