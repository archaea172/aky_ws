#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <vector>
#include <queue>

struct GridMap
{
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    int width = 0;
    int height = 0;
    std::vector<int8_t> data;
};

struct DistanceFieldMap {
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    int width = 0;
    int height = 0;
    Eigen::ArrayXXf distance;
};

struct BoidPrams
{
    int boid_num;
    double max_vel;
    double k_separation;
    double k_alignment;
    double k_gravity;
    double k_wall;
    DistanceFieldMap field;
    double Ir;
    double Ir_min;
};

Eigen::MatrixXd remove_col(const Eigen::MatrixXd& A, int k);
DistanceFieldMap convertmap_grid_to_distance(const GridMap& map);

class BoidCore
{
public:
    BoidCore(const BoidPrams& params);
    virtual Eigen::MatrixXd update_vels(const Eigen::MatrixXd& pos_matrix, const Eigen::MatrixXd& vel_matrix);
    
protected:
    Eigen::Vector2d make_base_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j);

    BoidPrams boid_params_;
    double Ir_2_;
    double Ir_min_2_;

private:
    Eigen::Vector2d make_separation_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j);
    Eigen::Vector2d make_alignment_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j);
    Eigen::Vector2d make_gravity_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j);
    Eigen::Vector2d make_wall_power(const Eigen::Vector2d& x_i);
};
