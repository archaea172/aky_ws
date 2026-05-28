#include "path_generate_server.hpp"

PathGenerateServer::PathGenerateServer()
: rclcpp::Node("path_generate_server")
{
    rclcpp::QoS path = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "leader_path",
        path
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
    const auto stamp = this->now();
    path.header.stamp = stamp;
    path.header.frame_id = request.start_pos.header.frame_id;

    Eigen::Vector2d start_xy;
    start_xy << request.start_pos.pose.position.x, request.start_pos.pose.position.y;
    Eigen::Vector2d goal_xy;
    goal_xy << request.goal_pos.pose.position.x, request.goal_pos.pose.position.y;
    std::vector<Eigen::Vector2d> waypoints_vector;
    for (const auto &waypoint : request.waypoints)
    {
        Eigen::Vector2d waypoint_xy;
        waypoint_xy << waypoint.pose.position.x, waypoint.pose.position.y;
        waypoints_vector.push_back(waypoint_xy);
    }
    waypoints_vector.push_back(goal_xy);

    Eigen::Vector2d diff = waypoints_vector[0] - start_xy;
    double theta = std::atan2(diff.y(), diff.x());
    double distance = diff.norm();
    int point_num = distance / request.resolution;

    request.start_pos.header.stamp = stamp;
    path.poses.push_back(request.start_pos);
    geometry_msgs::msg::PoseStamped add_point = request.start_pos;
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    for (int i = 0; i < point_num; i++)
    {
        add_point.pose.position.x += request.resolution * cos_t;
        add_point.pose.position.y += request.resolution * sin_t;
        add_point.header.stamp = stamp;
        path.poses.push_back(add_point);
    }

    for (int i = 0; i < (int)request.waypoints.size(); i++)
    {
        diff = waypoints_vector[i+1] - waypoints_vector[i];
        theta = std::atan2(diff.y(), diff.x());
        distance = diff.norm();
        point_num = distance / request.resolution;
        cos_t = std::cos(theta);
        sin_t = std::sin(theta);
        for (int j = 0; j < point_num; j++)
        {
            add_point.pose.position.x += request.resolution * cos_t;
            add_point.pose.position.y += request.resolution * sin_t;
            add_point.header.stamp = stamp;
            path.poses.push_back(add_point);
        }
    }

    request.start_pos.header.stamp = stamp;
    request.goal_pos.header.stamp = stamp;
    path.poses.push_back(request.goal_pos);

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
