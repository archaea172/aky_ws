#pragma once

#include <Eigen/Dense>
#include <random>

struct MppiParams
{
    int control_dim_; // 制御入力の次元
    double predict_resolution_;
    int predict_horizon_;
    Eigen::MatrixXd cov;

};

class MppiController
{
public:
    MppiController();
    ~MppiController();

private:
    Eigen::VectorXd sampleMultivariateNormal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& L);
    Eigen::MatrixXd samplingControlArray(const Eigen::VectorXd& pre_control_input);
    Eigen::MatrixXd calcState(
        const Eigen::MatrixXd input_array
    );
    MppiParams parameters_;
    std::mt19937 rng_;
};
