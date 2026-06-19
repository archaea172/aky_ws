#pragma once

#include <Eigen/Dense>

struct MppiParams
{
    int control_dim_; // 制御入力の次元
    float predict_resolution_;
    float predict_horizon_;
    
};

class MppiController
{
public:
    MppiController();
    ~MppiController();

private:
    MppiParams parameters;
};
