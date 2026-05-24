#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <vector>

struct GridMap
{
    double resolution;
    double origin_x;
    double origin_y;
    int width;
    int height;
    std::vector<int8_t> data;
};

struct DistanceFieldMap {
    double resolution;
    double origin_x;
    double origin_y;
    int width;
    int height;
    Eigen::ArrayXXf distance;
};

struct BoidPrams
{
    int boid_num;
    double max_vel;
    double k_separation;
    double k_alignment;
    double k_gravity;
    double Ir;
    double Ir_min;
};

Eigen::MatrixXd remove_col(const Eigen::MatrixXd& A, int k);

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
};
