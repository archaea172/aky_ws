#pragma once

#include <Eigen/Dense>
#include <random>

struct MppiSwermParams
{
    double predict_resolution;
    int predict_horizon;
    int sample_num;
    Eigen::Matrix2d cov;
    int robot_num;
};

class MppiSwermController
{
public:
    MppiSwermController(const MppiSwermParams& parameters);
    ~MppiSwermController();

// private:
    Eigen::Matrix<double, 2, Eigen::Dynamic> samplingControlArray(const Eigen::Vector2d& pre_control_input);
    std::vector<Eigen::MatrixXd> PredictState(
        const Eigen::MatrixXd& input_array,
        const std::vector<Eigen::MatrixXd>& state_array
    );

    Eigen::Matrix2d L;
    const int control_dim_ = 2;
    const MppiSwermParams parameters_;
    std::mt19937 rng_;
};
