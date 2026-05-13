#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "core/follower_core.hpp"

#include <atomic>

class follower_node
: public rclcpp_lifecycle::LifecycleNode
{
public:
    follower_node();
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

private:
    void control_stop();

    void odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    void leader_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    void control_callback();
    
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );

    std::unique_ptr<FollowerCore> follower_core_;
    BoidPrams boid_params_;
    double k_follow_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_publishers_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_pos_subscriber_;
    nav_msgs::msg::Odometry leader_odom_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    Eigen::MatrixXd pos_matrix_;
    Eigen::MatrixXd vel_matrix_;
    Eigen::Vector2d leader_pos_;
};
