#pragma once

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "swerm_msgs/action/leader_pos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "mppi_swerm.hpp"
#include "swerm/follower_core.hpp"

class LeaderPosServer
: public rclcpp::Node
{
public:
    LeaderPosServer();

private:
    using GoalHandleLeaderPos = rclcpp_action::ServerGoalHandle<swerm_msgs::action::LeaderPos>;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const swerm_msgs::action::LeaderPos::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLeaderPos> goal_handle
    );
    void handle_accepted(const std::shared_ptr<GoalHandleLeaderPos> goal_handle);
    void execute(const std::shared_ptr<GoalHandleLeaderPos> goal_handle);

    void odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    void map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr rxdata);

    bool is_out_of_map(geometry_msgs::msg::PoseStamped start_pos, geometry_msgs::msg::PoseStamped goal_pos);

    rclcpp_action::Server<swerm_msgs::action::LeaderPos>::SharedPtr action_server_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

    std::unique_ptr<MppiSwermController> mppi_controller_;
    SwermState swerm_states;
    DistanceFieldMap field_;
    bool subscribe_map_{false};

    double publish_rate_ms;
};
