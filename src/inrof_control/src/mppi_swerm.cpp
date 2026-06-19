#include "mppi_swerm.hpp"

MppiSwermController::MppiSwermController(const MppiSwermParams& parameters)
: parameters_(parameters)
{
}

MppiSwermController::~MppiSwermController()
{
}

Eigen::VectorXd MppiSwermController::sampleMultivariateNormal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& L)
{
    std::normal_distribution<double> normal(0.0, 1.0);
    Eigen::VectorXd z = Eigen::VectorXd::NullaryExpr(this->parameters_.control_dim, [&]() {
        return normal(this->rng_);
    });

    return mean + L * z;
}

Eigen::MatrixXd MppiSwermController::samplingControlArray(const Eigen::VectorXd& pre_control_input)
{
    Eigen::MatrixXd control_array(this->parameters_.control_dim, this->parameters_.predict_horizon);
    Eigen::LLT<Eigen::MatrixXd> llt(this->parameters_.cov);
    Eigen::MatrixXd L = llt.matrixL();

    for (int i = 0; i < this->parameters_.predict_horizon; ++i)
    {
        Eigen::VectorXd control_sample = this->sampleMultivariateNormal(pre_control_input, L);
        control_array.col(i) = control_sample;
    }

    return control_array;
}

Eigen::MatrixXd MppiSwermController::PredictState(
    const Eigen::MatrixXd input_array,
    const Eigen::MatrixXd state_array
)
{

}
