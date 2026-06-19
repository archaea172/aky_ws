#pragma once

#include <Eigen/Dense>
#include <random>

struct MppiParams
{
    int control_dim_; // 制御入力の次元
    double predict_resolution_;
    double predict_horizon_;
    Eigen::MatrixXd cov;
};

class MppiController
{
public:
    MppiController();
    ~MppiController();

private:
    Eigen::VectorXd sampleMultivariateNormal(const Eigen::VectorXd& mean);

    MppiParams parameters;
    std::mt19937 rng;
};
