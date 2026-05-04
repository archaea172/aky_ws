#include "core/sheep_core.hpp"

SheepCore::SheepCore(const BoidPrams& params, double k_run)
: BoidCore(params), k_run_(k_run)
{
}

Eigen::Vector2d SheepCore::make_run_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_d)
{
    Eigen::Vector2d l_i_diff = x_d - x_i;

    return -l_i_diff / std::max(l_i_diff.norm(), 0.01);
}

Eigen::MatrixXd SheepCore::update_vels(const Eigen::MatrixXd& pos_matrix, const Eigen::MatrixXd& vel_matrix, const Eigen::Vector2d& dog_pos)
{
    Eigen::MatrixXd cmd_vels(2, this->boid_params_.boid_num);
    for (int i = 0; i < this->boid_params_.boid_num; ++i)
    {
        Eigen::Vector2d x_i = pos_matrix.col(i);
        Eigen::MatrixXd x_j = remove_col(pos_matrix, i);
        Eigen::MatrixXd v_j = remove_col(vel_matrix, i);
        Eigen::Vector2d cmd_vel_i = this->make_base_power(x_i, x_j, v_j) + this->k_run_ * this->make_run_power(x_i, dog_pos);

        double cmd_vel_norm = cmd_vel_i.norm();
        if (cmd_vel_norm > this->boid_params_.max_vel)
        {
            cmd_vel_i *= this->boid_params_.max_vel / cmd_vel_norm;
        }

        cmd_vels.col(i) = cmd_vel_i;
    }

    return cmd_vels;
}
