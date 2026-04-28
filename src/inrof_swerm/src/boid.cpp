#include "boid.hpp"

using namespace std::chrono_literals;

boid_node::boid_node()
: rclcpp::Node("boid")
{
    this->declare_parameter<int>("boid_num");
    this->boid_num_ = this->get_parameter("boid_num").as_int();

    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->cmd_vel_publishers_.reserve(this->boid_num_);
    for (int i = 0; i < this->boid_num_; ++i)
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

    this->odoms_.resize(this->boid_num_);
    this->pos_matrix_.resize(2, this->boid_num_);
    this->vel_matrix_.resize(2, this->boid_num_);

    this->control_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        10ms,
        std::bind(&boid_node::control_callback, this)
    );
}

void boid_node::odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    if (id > this->boid_num_)
    {
        RCLCPP_ERROR(this->get_logger(), "id is invalid. please check robot num");
        return;
    }

    this->odoms_[id] = *rxdata;

    this->pos_matrix_.col(id) << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->vel_matrix_.col(id) << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}
