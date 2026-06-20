#pragma once

#include "swerm/boid_core.hpp"

class FollowerCore
: public BoidCore
{
public:
    FollowerCore(const BoidPrams& params, double k_follow);
    Eigen::MatrixXd update_vels(const Eigen::MatrixXd& pos_matrix, const Eigen::MatrixXd& vel_matrix, const Eigen::Vector2d& leader_pos);

private:
    double k_follow_;
    Eigen::Vector2d make_follow_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_l);
};
