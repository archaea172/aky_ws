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
    params.robot_num = 5;

    MppiSwermController test_controller(params);

    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間
    Eigen::Vector2d input(2);
    input << 2.0, 2.0;
    test_controller.samplingControlArray(input);
    end = std::chrono::system_clock::now();  // 計測終了時間
    double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
    printf("%f\r\n", elapsed);
    return 0;
}
