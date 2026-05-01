#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <Eigen/Dense>
#include <vector>

class boid_node
: public rclcpp::Node
{
public:
    boid_node(Eigen::MatrixXd wall_matrix = Eigen::MatrixXd{});

protected:
    virtual Eigen::MatrixXd update_vels();
    Eigen::Vector2d make_base_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j);
    Eigen::MatrixXd remove_col(const Eigen::MatrixXd& A, int k);
    int boid_num_;
    double max_vel_;
    Eigen::MatrixXd pos_matrix_;
    Eigen::MatrixXd vel_matrix_;

    Eigen::MatrixXd wall_matrix_;

    double k_separation;
    double k_alignment;
    double k_gravity;
    double k_wall;
    double Ir;
    double Ir_2;
    double Ir_min;
    double Ir_min_2;
    
    std::vector<nav_msgs::msg::Odometry> odoms_;

private:
    void odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata);
    void control_callback();
    Eigen::Vector2d make_separation_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j);
    Eigen::Vector2d make_alignment_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j);
    Eigen::Vector2d make_gravity_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j);
    Eigen::Vector2d make_wall_power(const Eigen::Vector2d& x_i);

    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_publishers_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<char> odom_received_;
    int received_odom_count_{0};
    bool is_ready{false};
};
