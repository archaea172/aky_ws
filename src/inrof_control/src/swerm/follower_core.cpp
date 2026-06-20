#include "swerm/follower_core.hpp"

FollowerCore::FollowerCore(const BoidPrams& params, double k_follow)
: BoidCore(params), k_follow_(k_follow)
{

}

Eigen::Vector2d FollowerCore::make_follow_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_l)
{
    Eigen::Vector2d l_i_diff = x_l - x_i;

    return l_i_diff / std::max(l_i_diff.norm(), 0.01);
}

Eigen::MatrixXd FollowerCore::update_vels(const Eigen::MatrixXd& pos_matrix, const Eigen::MatrixXd& vel_matrix, const Eigen::Vector2d& leader_pos)
{
    Eigen::MatrixXd cmd_vels(2, this->boid_params_.boid_num);
    for (int i = 0; i < this->boid_params_.boid_num; ++i)
    {
        Eigen::Vector2d x_i = pos_matrix.col(i);
        Eigen::MatrixXd x_j = remove_col(pos_matrix, i);
        Eigen::MatrixXd v_j = remove_col(vel_matrix, i);
        Eigen::Vector2d cmd_vel_i = this->make_base_power(x_i, x_j, v_j) + this->k_follow_ * this->make_follow_power(x_i, leader_pos);

        double cmd_vel_norm = cmd_vel_i.norm();
        if (cmd_vel_norm > this->boid_params_.max_vel)
        {
            cmd_vel_i *= this->boid_params_.max_vel / cmd_vel_norm;
        }

        cmd_vels.col(i) = cmd_vel_i;
    }

    return cmd_vels;
}
