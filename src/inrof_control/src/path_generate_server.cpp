#include "path_generate_server.hpp"

PathGenerateServer::PathGenerateServer()
: rclcpp::Node("path_generate_server")
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "leader_path",
        device
    );

    this->path_server_ = this->create_service<swerm_msgs::srv::LeaderPath>(
        "leader_path_service",
        std::bind(&PathGenerateServer::server_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void PathGenerateServer::server_callback(
    const std::shared_ptr<swerm_msgs::srv::LeaderPath::Request> request,
    std::shared_ptr<swerm_msgs::srv::LeaderPath::Response> response
)
{

}
