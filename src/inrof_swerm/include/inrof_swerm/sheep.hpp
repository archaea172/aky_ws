#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "boid.hpp"

class Sheep
: public boid_node
{
public:
    Sheep();

private:
    void dog_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    Eigen::Vector2d make_run_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_d);
    Eigen::MatrixXd update_vels() override;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dog_pos_subscriber_;
    nav_msgs::msg::Odometry dog_odom_;
    Eigen::Vector2d dog_pos_;
    Eigen::Vector2d dog_vel_;
    double k_run_;
};
