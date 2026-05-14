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
    nav_msgs::msg::Path path = this->gen_path(*request);
    response->route = path;
    this->path_publisher_->publish(path);
}

nav_msgs::msg::Path PathGenerateServer::gen_path(swerm_msgs::srv::LeaderPath::Request request)
{
    nav_msgs::msg::Path path;

    double start_x = request.start_pos.pose.position.x;
    double start_y = request.start_pos.pose.position.y;
    double goal_x = request.goal_pos.pose.position.x;
    double goal_y = request.goal_pos.pose.position.y;

    return path;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PathGenerateServer> node = std::make_shared<PathGenerateServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
