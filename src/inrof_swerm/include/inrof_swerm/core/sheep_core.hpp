#pragma once

#include "core/boid_core.hpp"

class SheepCore
: public BoidCore
{
public:
    SheepCore(const BoidPrams& params, double k_run);
    Eigen::MatrixXd update_vels(const Eigen::MatrixXd& pos_matrix, const Eigen::MatrixXd& vel_matrix, const Eigen::Vector2d& dog_pos);

private:
    double k_run_;
    Eigen::Vector2d make_run_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_d);
};
