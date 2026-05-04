#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "core/sheep_core.hpp"

class sheep_node
: public rclcpp::Node
{
public:
    sheep_node();

private:
    void odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    void dog_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    void control_callback();

    std::unique_ptr<SheepCore> sheep_core_;
    BoidPrams boid_params_;
    double k_run_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_publishers_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dog_pos_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    Eigen::MatrixXd pos_matrix_;
    Eigen::MatrixXd vel_matrix_;
    Eigen::Vector2d dog_pos_;
};
