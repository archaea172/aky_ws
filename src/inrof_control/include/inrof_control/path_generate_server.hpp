#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "swerm_msgs/srv/leader_path.hpp"
#include "nav_msgs/msg/path.hpp"

class PathGenerateServer
: public rclcpp::Node
{
public:
    PathGenerateServer();

private:
    void server_callback(
        const std::shared_ptr<swerm_msgs::srv::LeaderPath::Request> request,
        std::shared_ptr<swerm_msgs::srv::LeaderPath::Response> response
    );
    nav_msgs::msg::Path gen_path(swerm_msgs::srv::LeaderPath::Request request);

    rclcpp::Service<swerm_msgs::srv::LeaderPath>::SharedPtr path_server_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};
