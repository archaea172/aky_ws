#include "boid.hpp"

using namespace std::chrono_literals;

boid_node::boid_node()
: rclcpp::Node("boid")
{
    this->declare_parameter<double>("boid_num");
    this->boid_num_ = this->get_parameter("boid_num").as_double();

    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->cmd_vel_publishers_.reserve(this->boid_num_);
    for (int i = 0; i < this->boid_num_; ++i)
    this->cmd_vel_publishers_.push_back(
        this->create_publisher<geometry_msgs::msg::Twist>(
            "robot_" + std::to_string(i) + "/cmd_vel",
            device
        )
    );

    this->control_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        10ms,
        std::bind(&boid_node::control_callback, this)
    );
}
