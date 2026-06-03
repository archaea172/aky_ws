#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "core/boid_core.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("make_distancefield_file");

    nav_msgs::msg::OccupancyGrid grid;
    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();

    bool success = rclcpp::wait_for_message(
        grid,
        node,
        "map",
        std::chrono::seconds(4),
        map_qos
    );

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Received occupancy grid, writing to file...");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to receive occupancy grid within timeout.");
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}