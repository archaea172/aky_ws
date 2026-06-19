#include "mppi_swerm.hpp"

MppiSwermController::MppiSwermController(const MppiSwermParams& parameters)
: parameters_(parameters)
{
    Eigen::LLT<Eigen::Matrix2d> llt(this->parameters_.cov);
    this->L = llt.matrixL();
}

MppiSwermController::~MppiSwermController()
{
}

Eigen::Matrix<double, 2, Eigen::Dynamic>
MppiSwermController::samplingControlArray(const Eigen::Vector2d& pre_control_input)
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

Eigen::MatrixXd MppiSwermController::PredictState(
    const Eigen::MatrixXd input_array,
    const Eigen::MatrixXd state_array
)
{

}
