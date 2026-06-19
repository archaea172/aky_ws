#pragma once

#include <Eigen/Dense>
#include <random>

struct MppiSwermParams
{
    int control_dim_; // 制御入力の次元
    double predict_resolution_;
    int predict_horizon_;
    Eigen::MatrixXd cov;

};

class MppiSwermController
{
public:
    MppiSwermController();
    ~MppiSwermController();

private:
    Eigen::VectorXd sampleMultivariateNormal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& L);
    Eigen::MatrixXd samplingControlArray(const Eigen::VectorXd& pre_control_input);
    Eigen::MatrixXd PredictState(
        const Eigen::MatrixXd input_array,
        const Eigen::MatrixXd state_array
    );
    MppiSwermParams parameters_;
    std::mt19937 rng_;
};
