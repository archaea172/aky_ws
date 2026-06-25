#include "mppi_swerm.hpp"

MppiSwermController::MppiSwermController(const MppiSwermParams& parameters, const Eigen::Vector2d& goal_pos)
: parameters_(parameters), follower_core_(parameters.boid_parameters, parameters.k_follow), goal_pos_(goal_pos)
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

    for (int i = 0; i < horizon; ++i)
    {
        const double norm = control_array.col(i).norm();
        if (norm > this->parameters_.max_v)
        {
            control_array.col(i) *= this->parameters_.max_v / norm;
        }
    }

    return control_array;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> MppiSwermController::calcLeaderPos(
    const Eigen::Vector2d& now_leader_pos,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& leader_vel_array
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

std::vector<SwermState> MppiSwermController::calcSwermPos(
    const SwermState& x0,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& leader_pos_array
)
{
    std::vector<SwermState> predict_states;
    predict_states.reserve(this->parameters_.predict_horizon + 1);
    predict_states.push_back(x0);

    for (int i = 0; i < this->parameters_.predict_horizon; ++i)
    {
        SwermState i_swerm_state;
        Eigen::MatrixXd cmd_vels =
            this->follower_core_.update_vels(
                predict_states[i].pose.topRows(2),
                predict_states[i].vel,
                leader_pos_array.col(i)
        );
        i_swerm_state.vel = cmd_vels;
        i_swerm_state.pose = predict_states[i].pose + cmd_vels * this->parameters_.predict_resolution;

        predict_states.push_back(i_swerm_state);
    }

    return predict_states;
}

double MppiSwermController::calcCost(
    const std::vector<SwermState>& swerm_state,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& leader_pos_array
)
{
    static_cast<void>(leader_pos_array);

    double cost = 0.0;

    for (const SwermState& i_swerm_state : swerm_state)
    {
        // 各ロボットがゴールまで近づいているか
        cost += this->parameters_.weights.w_goal * (i_swerm_state.pose.colwise() - this->goal_pos_).colwise().squaredNorm().sum();

        // 直線状にしたい
        const Eigen::MatrixXd pose = i_swerm_state.pose;
        const int n = pose.cols();
        if (n >= 2)
        {
            double sum_x = 0.0;
            double sum_y = 0.0;
            double sum_xx = 0.0;
            double sum_yy = 0.0;
            double sum_xy = 0.0;

            for (int i = 0; i < n; ++i)
            {
                const double x = pose(0, i);
                const double y = pose(1, i);

                sum_x += x;
                sum_y += y;
                sum_xx += x * x;
                sum_yy += y * y;
                sum_xy += x * y;
            }

            const double inv_n = 1.0 / static_cast<double>(n);
            const double sxx = sum_xx - sum_x * sum_x * inv_n;
            const double syy = sum_yy - sum_y * sum_y * inv_n;
            const double sxy = sum_xy - sum_x * sum_y * inv_n;

            const double trace = sxx + syy;

            if (trace > 1e-9)
            {
                const double diff = sxx - syy;
                const double discriminant = std::sqrt(diff * diff + 4.0 * sxy * sxy);
                const double line_score = (trace + discriminant) / (2.0 * trace);

                cost += this->parameters_.weights.w_linear * (1.0 - line_score);
                // 既存と同じ -log 系にしたいなら:
                // cost += -std::log(line_score + 1e-9);
            }
            else
            {
                cost += 1.0;
            }
        }
    }
    // リーダーがゴールまで近づいているか
    cost += this->parameters_.weights.w_leader_goal * (leader_pos_array.col(leader_pos_array.cols() - 1) - this->goal_pos_).squaredNorm();
    

    return cost;
}

Eigen::VectorXd MppiSwermController::calcWeights(const Eigen::VectorXd& costs)
{
    double rho = costs.minCoeff();
    Eigen::VectorXd weights = (-(costs.array() - rho) / this->parameters_.lambda).exp().matrix();
    weights /= weights.sum();

    return weights;
}

Eigen::Vector2d MppiSwermController::controlLoop(
    const SwermState& x0,
    const Eigen::Vector2d& now_leader_pos
)
{
    if (!is_start_)
    {
        Eigen::Vector2d dir = this->goal_pos_ - now_leader_pos;

        if (dir.norm() > 1e-6) {
            this->pre_leader_vel_ = 4 * dir.normalized();
        }
        is_start_ = true;
        std::cout << "starting!!"  << std::endl;
    }
    Eigen::VectorXd costs(this->parameters_.sample_num);
    Eigen::Matrix<double, 2, Eigen::Dynamic> initial_leader_vel(2, this->parameters_.sample_num);

    for (int i = 0; i < this->parameters_.sample_num; ++i)
    {
        Eigen::Matrix<double, 2, Eigen::Dynamic> leader_vels = this->samplingLeaderVelArray(this->pre_leader_vel_);
        Eigen::Matrix<double, 2, Eigen::Dynamic> leader_poses = this->calcLeaderPos(now_leader_pos, leader_vels);
        std::vector<SwermState> swerm_pos = this->calcSwermPos(x0, leader_poses);
        costs(i) = this->calcCost(swerm_pos, leader_poses);
        initial_leader_vel.col(i) = leader_vels.col(0);
    }
    Eigen::VectorXd weights = this->calcWeights(costs);
    
    Eigen::Vector2d leader_vel = initial_leader_vel * weights;
    Eigen::Vector2d input = now_leader_pos + leader_vel * 1 / this->parameters_.control_frequency;
    this->pre_leader_vel_ = leader_vel;

    return input;
}
