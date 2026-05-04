#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "core/boid_core.hpp"

class boid_node
: public rclcpp::Node
{
public:
    boid_node();

private:
    void odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    void control_callback();

    std::unique_ptr<BoidCore> boid_core_;
    BoidPrams boid_params_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_publishers_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    Eigen::MatrixXd pos_matrix_;
    Eigen::MatrixXd vel_matrix_;
};
