#pragma once

#include <Eigen/Dense>
#include <random>

struct MppiSwermParams
{
    int control_dim; // 制御入力の次元
    double predict_resolution;
    int predict_horizon;
    Eigen::MatrixXd cov;
    int robot_num;
};

class MppiSwermController
{
public:
    MppiSwermController(const MppiSwermParams& parameters);
    ~MppiSwermController();

// private:
    Eigen::VectorXd sampleMultivariateNormal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& L);
    Eigen::MatrixXd samplingControlArray(const Eigen::VectorXd& pre_control_input);
    Eigen::MatrixXd PredictState(
        const Eigen::MatrixXd input_array,
        const Eigen::MatrixXd state_array
    );
    MppiSwermParams parameters_;
    std::mt19937 rng_;
};
