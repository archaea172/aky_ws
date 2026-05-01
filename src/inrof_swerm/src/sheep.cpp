#include "sheep.hpp"

Sheep::Sheep(Eigen::MatrixXd wall_matrix)
: boid_node(wall_matrix)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->dog_pos_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "leader/odometry",
        device,
        std::bind(&Sheep::dog_odom_callback, this, std::placeholders::_1)
    );
    this->declare_parameter<double>("k_run", 0.5);
    this->k_run_ = this->get_parameter("k_run").as_double();
}

void Sheep::dog_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    this->dog_odom_ = *rxdata;
    this->dog_pos_ << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->dog_vel_ << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}

Eigen::Vector2d Sheep::make_run_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_d)
{
    Eigen::Vector2d l_i_diff = x_d - x_i;

    return -l_i_diff / std::max(l_i_diff.norm(), 0.01);
}

Eigen::MatrixXd Sheep::update_vels()
{
    Eigen::MatrixXd cmd_vels(2, this->boid_num_);
    for (int i = 0; i < this->boid_num_; ++i)
    {
        Eigen::Vector2d x_i = this->pos_matrix_.col(i);
        Eigen::MatrixXd x_j = this->remove_col(this->pos_matrix_, i);
        Eigen::MatrixXd v_j = this->remove_col(this->vel_matrix_, i);
        Eigen::Vector2d cmd_vel_i = this->make_base_power(x_i, x_j, v_j) + this->k_run_ * this->make_run_power(x_i, dog_pos_);

        double cmd_vel_norm = cmd_vel_i.norm();
        if (cmd_vel_norm > this->max_vel_)
        {
            cmd_vel_i *= this->max_vel_ / cmd_vel_norm;
        }

        cmd_vels.col(i) = cmd_vel_i;
    }

    return cmd_vels;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Sheep> node = std::make_shared<Sheep>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
