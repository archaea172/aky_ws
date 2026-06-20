#include "mppi_swerm.hpp"

MppiSwermController::MppiSwermController(const MppiSwermParams& parameters)
: parameters_(parameters), follower_core_(parameters.boid_parameters, parameters.k_follow)
{
    Eigen::LLT<Eigen::Matrix2d> llt(this->parameters_.cov);
    this->L = llt.matrixL();
}

MppiSwermController::~MppiSwermController()
{
}

Eigen::Matrix<double, 2, Eigen::Dynamic>
MppiSwermController::samplingLeaderVelArray(const Eigen::Vector2d& pre_control_input)
{
    const int horizon = this->parameters_.predict_horizon;
    std::normal_distribution<double> normal(0.0, 1.0);

    Eigen::Matrix<double, 2, Eigen::Dynamic> z(2, horizon);

    z = Eigen::Matrix<double, 2, Eigen::Dynamic>::NullaryExpr(
        2, horizon,
        [&]() {
            return normal(rng_);
        }
    );

    Eigen::Matrix<double, 2, Eigen::Dynamic> control_array(2, horizon);
    control_array.noalias() = this->L * z;
    control_array.colwise() += pre_control_input;

    return control_array;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> MppiSwermController::calcLeaderPos(
    const Eigen::Vector2d& now_leader_pos,
    const Eigen::Matrix<double, 2, Eigen::Dynamic> leader_vel_array
)
{
    Eigen::Matrix<double, 2, Eigen::Dynamic> leader_pos_array(2, this->parameters_.predict_horizon + 1);
    leader_pos_array.col(0) = now_leader_pos;
    for (int i = 0; i < this->parameters_.predict_horizon; ++i)
    {
        Eigen::Vector2d pos = leader_pos_array.col(i) + leader_vel_array.col(i) * this->parameters_.predict_resolution;
        leader_pos_array.col(i + 1) = pos;
    }

    return leader_pos_array;
}
