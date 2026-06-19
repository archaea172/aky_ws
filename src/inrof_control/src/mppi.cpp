#include "mppi.hpp"

MppiController::MppiController()
{
}

MppiController::~MppiController()
{
}

Eigen::VectorXd MppiController::sampleMultivariateNormal(const Eigen::VectorXd& mean)
{
    std::normal_distribution<double> normal(0.0, 1.0);
    Eigen::VectorXd z = Eigen::VectorXd::NullaryExpr(this->parameters.control_dim_, [&]() {
        return normal(this->rng);
    });

    Eigen::LLT<Eigen::MatrixXd> llt(this->parameters.cov);
    Eigen::MatrixXd L = llt.matrixL();

    return mean + L * z;
}
