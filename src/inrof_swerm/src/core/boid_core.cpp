#include "core/boid_core.hpp"

Eigen::MatrixXd remove_col(const Eigen::MatrixXd& A, int k)
{
    Eigen::MatrixXd B(A.rows(), A.cols() - 1);
    B.leftCols(k) = A.leftCols(k);
    B.rightCols(A.cols() - k - 1) = A.rightCols(A.cols() - k - 1);
    return B;
}

DistanceFieldMap convertmap_grid_to_distance(const GridMap& map)
{
    DistanceFieldMap field;
    const int w = map.width;
    const int h = map.height;
    const float inf = std::numeric_limits<float>::infinity();

    field.resolution = map.resolution;
    field.origin_x = map.origin_x;
    field.origin_y = map.origin_y;
    field.origin_yaw = map.origin_yaw;
    field.width = w;
    field.height = h;

    field.distance = Eigen::ArrayXXf::Constant(h, w, inf);

    auto idx = [w](int x, int y) {
        return y * w + x;
    };

    using Item = std::pair<float, Eigen::Array2i>;
    struct CompareItem {
        bool operator()(const Item& a, const Item& b) const {
            return a.first > b.first;
        }
    };
    std::priority_queue<Item, std::vector<Item>, CompareItem> queue;

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            int i = idx(x, y);

            if (map.data[i] >= 65)
            {
                field.distance(y, x) = 0.0f;
                Eigen::Array2i xy = {x, y};
                queue.push({0.0f, xy});
            }
        }
    }

    const std::array<std::pair<int, int>, 8> dirs = {{
        {-1,  0}, {1,  0}, {0, -1}, {0, 1},
        {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
    }};

    while (!queue.empty())
    {
        auto [d, xy] = queue.top();
        queue.pop();

        if (d > field.distance(xy.y(), xy.x())) continue;

        for (auto [dx, dy] : dirs)
        {
            int nx = xy.x() + dx;
            int ny = xy.y() + dy;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

            float step = (dx != 0 && dy != 0)
                ? static_cast<float>(map.resolution * std::sqrt(2.0))
                : static_cast<float>(map.resolution);
            
            float nd = d + step;

            if (nd < field.distance(ny, nx))
            {
                field.distance(ny, nx) = nd;
                Eigen::Array2i nxy = {nx, ny};
                queue.push({nd, nxy});
            }
        }
    }
    return field;
}

BoidCore::BoidCore(const BoidPrams& params)
: boid_params_(params)
{
    Ir_2_ = std::pow(boid_params_.Ir, 2);
    Ir_min_2_ = std::pow(boid_params_.Ir_min, 2);
}

Eigen::Vector2d BoidCore::make_base_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j)
{
    return
        this->boid_params_.k_separation * this->make_separation_power(x_i, x_j) +
        this->boid_params_.k_alignment * this->make_alignment_power(x_i, x_j, v_j) +
        this->boid_params_.k_gravity * this->make_gravity_power(x_i, x_j) +
        this->boid_params_.k_wall * this->make_wall_power(x_i);
}

Eigen::MatrixXd BoidCore::update_vels(const Eigen::MatrixXd& pos_matrix, const Eigen::MatrixXd& vel_matrix)
{
    Eigen::MatrixXd cmd_vels(2, this->boid_params_.boid_num);
    for (int i = 0; i < this->boid_params_.boid_num; ++i)
    {
        Eigen::Vector2d x_i = pos_matrix.col(i);
        Eigen::MatrixXd x_j = remove_col(pos_matrix, i);
        Eigen::MatrixXd v_j = remove_col(vel_matrix, i);
        Eigen::Vector2d cmd_vel_i = this->make_base_power(x_i, x_j, v_j);

        double cmd_vel_norm = cmd_vel_i.norm();
        if (cmd_vel_norm > this->boid_params_.max_vel)
        {
            cmd_vel_i *= this->boid_params_.max_vel / cmd_vel_norm;
        }

        cmd_vels.col(i) = cmd_vel_i;
    }

    return cmd_vels;
}

Eigen::Vector2d BoidCore::make_separation_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < (size_t)x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2_ > D_square && D_square > Ir_min_2_) 
        {
            Eigen::Vector2d posVD_i = i_j_diff / pow(D_square, 1.5);
            sum += posVD_i;
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return vel;
}

Eigen::Vector2d BoidCore::make_alignment_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < (size_t)x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2_ > D_square && D_square > Ir_min_2_) 
        {
            double velVN = v_j.col(i).norm();
            Eigen::Vector2d velD = v_j.col(i) / std::max(velVN, 1.0);
            sum += velD;
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return vel;
}

Eigen::Vector2d BoidCore::make_gravity_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < (size_t)x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2_ > D_square && D_square > Ir_min_2_) 
        {
            double use_d = std::max(sqrt(D_square), 1.0);
            sum += i_j_diff / use_d;
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return -vel;
}

Eigen::Vector2d BoidCore::make_wall_power(const Eigen::Vector2d& x_i)
{
    const DistanceFieldMap& field = this->boid_params_.field;
    if (field.resolution <= 0.0 || field.width <= 2 || field.height <= 2 || field.distance.size() == 0)
    {
        return Eigen::Vector2d::Zero();
    }
    
    double dx = x_i.x() - field.origin_x;
    double dy = x_i.y() - field.origin_y;
    double local_x =  cos(field.origin_yaw) * dx + sin(field.origin_yaw) * dy;
    double local_y = -sin(field.origin_yaw) * dx + cos(field.origin_yaw) * dy;

    const int mx = static_cast<int>(
        std::floor(local_x / field.resolution)
    );
    const int my = static_cast<int>(
        std::floor(local_y / field.resolution)
    );

    if (mx <= 0 || mx >= field.width - 1 || my <= 0 || my >= field.height - 1)
    {
        return Eigen::Vector2d::Zero();
    }

    const double d = field.distance(my, mx);
    if (!std::isfinite(d)) return Eigen::Vector2d::Zero();

    const double wall_range = boid_params_.Ir;
    if (d >= wall_range) {
        return Eigen::Vector2d::Zero();
    }

    const double r = field.resolution;

    const double grad_x =
        (field.distance(my, mx + 1) - field.distance(my, mx - 1)) / (2.0 * r);

    const double grad_y =
        (field.distance(my + 1, mx) - field.distance(my - 1, mx)) / (2.0 * r);

    Eigen::Vector2d grad(grad_x, grad_y);

    if (grad.norm() < 1e-6) return Eigen::Vector2d::Zero();

    Eigen::Rotation2Dd map_to_world(field.origin_yaw);
    const Eigen::Vector2d dir = (map_to_world * grad).normalized() / std::max(d, boid_params_.Ir_min);

    return dir;
}
