#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "boid.hpp"

class Follower
: public boid_node
{
public:
    Follower();

private:
    void leader_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    Eigen::Vector2d make_follow_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_l);
    Eigen::MatrixXd update_vels() override;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_pos_subscriber_;
    nav_msgs::msg::Odometry leader_odom_;
    Eigen::Vector2d leader_pos_;
    Eigen::Vector2d leader_vel_;
    double k_follow_;
};
