#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <Eigen/Dense>

class boid_node
: public rclcpp::Node
{
public:
    boid_node();
private:
    int boid_num_;
    std::vector<geometry_msgs::msg::Pose> pose_;
};
