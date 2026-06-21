#include "mppi_swerm.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iostream>

DistanceFieldMap loadDistanceField(const std::filesystem::path& yaml_path)
{
    YAML::Node meta = YAML::LoadFile(yaml_path.string());

    const int width = meta["width"].as<int>();
    const int height = meta["height"].as<int>();
    const auto origin = meta["origin"];

    const auto bin_path = yaml_path.parent_path() / meta["data"].as<std::string>();

    std::vector<float> buffer(width * height);
    std::ifstream bin(bin_path, std::ios::binary);
    if (!bin) {
        throw std::runtime_error("failed to open distance field bin: " + bin_path.string());
    }

    bin.read(
        reinterpret_cast<char*>(buffer.data()),
        static_cast<std::streamsize>(buffer.size() * sizeof(float))
    );

    if (bin.gcount() != static_cast<std::streamsize>(buffer.size() * sizeof(float))) {
        throw std::runtime_error("invalid distance field bin size: " + bin_path.string());
    }

    DistanceFieldMap field;
    field.width = width;
    field.height = height;
    field.resolution = meta["resolution"].as<double>();
    field.origin_x = origin[0].as<double>();
    field.origin_y = origin[1].as<double>();
    field.origin_yaw = origin[2].as<double>();

    field.distance = Eigen::ArrayXXf(height, width);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            field.distance(y, x) = buffer[y * width + x];
        }
    }

    return field;
}

int main(int argc, char *argv[])
{
    MppiSwermParams params;
    params.predict_resolution = 0.02;
    params.predict_horizon = 100;
    Eigen::MatrixXd cov(2, 2);
    cov << 1, 0, 0, 1;
    params.cov = cov;
    params.sample_num = 200;
    params.lambda = 5.0;
    params.boid_parameters.boid_num = 5;
    params.boid_parameters.max_vel = 0.05;
    params.boid_parameters.Ir = 100.0;
    params.boid_parameters.Ir_min = 0.01;
    params.boid_parameters.k_separation = 20.0;
    params.boid_parameters.k_alignment = 1.1;
    params.boid_parameters.k_gravity = 0.5;
    params.boid_parameters.k_wall = 0.5;
    params.boid_parameters.field = loadDistanceField("/home/aky/aky_ws/src/inrof_control/maps/irc_distance_field.yaml");
    params.k_follow = 0.5;

    MppiSwermController test_controller(params);

    SwermState state;
    Eigen::MatrixXd pose = Eigen::MatrixXd::Zero(2, 5);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 5);
    state.pose = pose;
    state.vel = vel;
    Eigen::Vector2d leader_pos;
    leader_pos << 1.0, 1.0;
    Eigen::VectorXd costs(params.sample_num);
    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間
    Eigen::Vector2d input = test_controller.controlLoop(state, leader_pos, leader_pos);
    end = std::chrono::system_clock::now();  // 計測終了時間
    double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
    
    printf("time:%f ms\r\n", elapsed);
    std::cout << input << std::endl;
    
    return 0;
}
