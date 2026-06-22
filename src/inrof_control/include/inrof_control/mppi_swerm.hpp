#pragma once

#include <Eigen/Dense>
#include <random>
#include <vector>
#include <iostream>
#include "swerm/boid_core.hpp"
#include "swerm/follower_core.hpp"

struct MppiSwermParams
{
    double control_frequency; // hz
    double predict_resolution;
    int predict_horizon;
    int sample_num;
    Eigen::Matrix2d cov;
    double lambda;
    double gamma;
    double max_v;

    BoidPrams boid_parameters;
    double k_follow;
};

struct SwermState
{
    Eigen::Matrix<double, 2, Eigen::Dynamic> pose;
    Eigen::Matrix<double, 2, Eigen::Dynamic> vel;
};

class MppiSwermController
{
public:
    MppiSwermController(const MppiSwermParams& parameters, const Eigen::Vector2d& goal_pos);
    ~MppiSwermController();
    Eigen::Vector2d controlLoop(
        const SwermState& x0,
        const Eigen::Vector2d& now_leader_pos
    );

private:

    Eigen::Matrix<double, 2, Eigen::Dynamic> samplingLeaderVelArray(const Eigen::Vector2d& pre_control_input);
    Eigen::Matrix<double, 2, Eigen::Dynamic> calcLeaderPos(
        const Eigen::Vector2d& now_leader_pos,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& leader_vel_array
    );
    std::vector<SwermState> calcSwermPos(
        const SwermState& x0,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& leader_pos_array
    );
    double calcCost(
        const std::vector<SwermState>& swerm_state,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& leader_pos_array
    );
    Eigen::VectorXd calcWeights(const Eigen::VectorXd& costs);

    Eigen::Matrix2d L;
    const int control_dim_ = 2;
    const MppiSwermParams parameters_;
    FollowerCore follower_core_;
    std::mt19937 rng_;

    const Eigen::Vector2d goal_pos_;
    Eigen::Vector2d pre_leader_vel_;
    bool is_start_{false};
};
