#pragma once

#include <Eigen/Dense>
#include <random>
#include "swerm/boid_core.hpp"
#include "swerm/follower_core.hpp"

struct MppiSwermParams
{
    double predict_resolution;
    int predict_horizon;
    int sample_num;
    Eigen::Matrix2d cov;
    BoidPrams boid_parameters;
    double k_follow;
};

class MppiSwermController
{
public:
    MppiSwermController(const MppiSwermParams& parameters);
    ~MppiSwermController();

// private:
    Eigen::Matrix<double, 2, Eigen::Dynamic> samplingLeaderVelArray(const Eigen::Vector2d& pre_control_input);
    Eigen::Matrix<double, 2, Eigen::Dynamic> calcLeaderPos(
        const Eigen::Vector2d& now_leader_pos,
        const Eigen::Matrix<double, 2, Eigen::Dynamic> leader_vel_array
    );

    Eigen::Matrix2d L;
    const int control_dim_ = 2;
    const MppiSwermParams parameters_;
    const FollowerCore follower_core_;
    std::mt19937 rng_;
};
