#include "mppi_swerm.hpp"
#include <chrono>

int main(int argc, char *argv[])
{
    MppiSwermParams params;
    params.predict_resolution = 0.01;
    params.predict_horizon = 200;
    Eigen::MatrixXd cov(2, 2);
    cov << 1, 0, 0, 1;
    params.cov = cov;
    params.sample_num = 200;
    params.boid_parameters.boid_num = 5;
    params.boid_parameters.max_vel = 0.05;
    params.boid_parameters.Ir = 100.0;
    params.boid_parameters.Ir_min = 0.01;
    params.boid_parameters.k_separation = 20.0;
    params.boid_parameters.k_alignment = 1.1;
    params.boid_parameters.k_gravity = 0.5;
    params.boid_parameters.k_wall = 0.5;
    params.k_follow = 0.5;

    MppiSwermController test_controller(params);

    SwermState state;
    Eigen::MatrixXd pose = Eigen::MatrixXd::Zero(2, 5);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 5);
    state.pose = pose;
    state.vel = vel;
    Eigen::Vector2d leader_pos;
    leader_pos << 2.0, 2.0;
    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間
    Eigen::Matrix<double, 2, Eigen::Dynamic> leader_vels = test_controller.samplingLeaderVelArray(leader_pos);
    Eigen::Matrix<double, 2, Eigen::Dynamic> leader_poses = test_controller.calcLeaderPos(leader_pos, leader_vels);
    std::vector<SwermState> swerm_pos = test_controller.calcSwermPos(state, leader_poses);
    end = std::chrono::system_clock::now();  // 計測終了時間
    double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
    
    printf("%f\r\n", elapsed);
    return 0;
}
