#include "boid_node.hpp"

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
