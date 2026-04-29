#include "follower.hpp"

Follower::Follower()
: boid_node()
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->leader_pos_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "leader/odometry",
        device,
        std::bind(&Follower::leader_odom_callback, this, std::placeholders::_1)
    );
}

void Follower::leader_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    this->leader_odom_ = *rxdata;
    this->leader_pos_ << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->leader_vel_ << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}

Eigen::Vector2d Follower::make_follow_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_l)
{
    Eigen::Vector2d l_i_diff = x_l - x_i;

    return l_i_diff / l_i_diff.norm();
}

Eigen::MatrixXd Follower::update_vels()
{
    Eigen::MatrixXd cmd_vels(2, this->boid_num_);
    for (int i = 0; i < this->boid_num_; ++i)
    {
        Eigen::Vector2d x_i = this->pos_matrix_.col(i);
        Eigen::MatrixXd x_j = this->remove_col(this->pos_matrix_, i);
        Eigen::MatrixXd v_j = this->remove_col(this->vel_matrix_, i);
        Eigen::Vector2d cmd_vel_i = this->make_base_power(x_i, x_j, v_j) + this->k_follow_ * this->make_follow_power(x_i, leader_pos_);

        double cmd_vel_norm = cmd_vel_i.norm();
        if (cmd_vel_norm > this->max_vel_)
        {
            cmd_vel_i *= this->max_vel_ / cmd_vel_norm;
        }

        cmd_vels.col(i) = cmd_vel_i;
    }

    return cmd_vels;
}
