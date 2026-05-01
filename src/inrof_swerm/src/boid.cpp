#include "boid.hpp"

using namespace std::chrono_literals;

boid_node::boid_node(Eigen::MatrixXd wall_matrix)
: rclcpp::Node("boid"), wall_matrix_(wall_matrix)
{
    this->declare_parameter<int>("boid_num", 20);
    this->boid_num_ = this->get_parameter("boid_num").as_int();
    this->declare_parameter<double>("max_vel", 0.05);
    this->max_vel_ = this->get_parameter("max_vel").as_double();
    this->declare_parameter<double>("Ir", 100);
    this->Ir = this->get_parameter("Ir").as_double();
    this->declare_parameter<double>("Ir_min", 0.01);
    this->Ir_min = this->get_parameter("Ir_min").as_double();
    this->declare_parameter<double>("k_separation", 20.0);
    this->declare_parameter<double>("k_alignment", 1.1);
    this->declare_parameter<double>("k_gravity", 0.5);
    this->declare_parameter<double>("k_wall", 20.0);
    this->k_separation = this->get_parameter("k_separation").as_double();
    this->k_alignment = this->get_parameter("k_alignment").as_double();
    this->k_gravity = this->get_parameter("k_gravity").as_double();
    this->k_wall = this->get_parameter("k_wall").as_double();
    
    this->Ir_2 = std::pow(this->Ir, 2);
    this->Ir_min_2 = std::pow(this->Ir_min, 2);

    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->cmd_vel_publishers_.reserve(this->boid_num_);
    for (int i = 0; i < this->boid_num_; ++i)
    {
        this->cmd_vel_publishers_.push_back(
            this->create_publisher<geometry_msgs::msg::Twist>(
                "robot_" + std::to_string(i) + "/cmd_vel",
                device
            )
        );

        this->odom_subscribers_.push_back(
            this->create_subscription<nav_msgs::msg::Odometry>(
                "robot_" + std::to_string(i) + "/odometry",
                device,
                [this, i](nav_msgs::msg::Odometry::ConstSharedPtr rxdata) {
                    this->odom_callback(i, rxdata);
                }
            )
        );
    }

    this->odoms_.resize(this->boid_num_);
    this->odom_received_.assign(this->boid_num_, false);
    this->pos_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_num_);
    this->vel_matrix_ = Eigen::MatrixXd::Zero(2, this->boid_num_);

    this->control_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        50ms,
        std::bind(&boid_node::control_callback, this)
    );
}

void boid_node::odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    if (id < 0 || id >= this->boid_num_)
    {
        RCLCPP_ERROR(this->get_logger(), "id is invalid. please check robot num");
        return;
    }

    if (!this->odom_received_[id])
    {
        this->odom_received_[id] = true;
        ++this->received_odom_count_;
        this->is_ready = this->received_odom_count_ == this->boid_num_;
    }

    this->odoms_[id] = *rxdata;

    this->pos_matrix_.col(id) << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->vel_matrix_.col(id) << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}

void boid_node::control_callback()
{
    if (!this->is_ready) return;

    Eigen::MatrixXd cmd_vels = this->update_vels();

    for (int i = 0; i < this->boid_num_; ++i)
    {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = cmd_vels(0, i);
        txdata.linear.y = cmd_vels(1, i);

        this->cmd_vel_publishers_[i]->publish(txdata);
    }
}

Eigen::Vector2d boid_node::make_separation_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < (size_t)x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2 > D_square && D_square > Ir_min_2) 
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

Eigen::Vector2d boid_node::make_alignment_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < (size_t)x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2 > D_square && D_square > Ir_min_2) 
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

Eigen::Vector2d boid_node::make_gravity_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < (size_t)x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2 > D_square && D_square > Ir_min_2) 
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

Eigen::Vector2d boid_node::make_wall_power(const Eigen::Vector2d& x_i)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < (size_t)this->wall_matrix_.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - this->wall_matrix_.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2 > D_square && D_square > Ir_min_2) 
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

Eigen::Vector2d boid_node::make_base_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j)
{
    return
        this->k_separation * this->make_separation_power(x_i, x_j) +
        this->k_alignment * this->make_alignment_power(x_i, x_j, v_j) +
        this->k_gravity * this->make_gravity_power(x_i, x_j) +
        this->k_wall * this->make_wall_power(x_i);
}

Eigen::MatrixXd boid_node::update_vels()
{
    if (!this->is_ready) return Eigen::MatrixXd::Zero(2, this->boid_num_);
    Eigen::MatrixXd cmd_vels(2, this->boid_num_);
    for (int i = 0; i < this->boid_num_; ++i)
    {
        Eigen::Vector2d x_i = this->pos_matrix_.col(i);
        Eigen::MatrixXd x_j = this->remove_col(this->pos_matrix_, i);
        Eigen::MatrixXd v_j = this->remove_col(this->vel_matrix_, i);
        Eigen::Vector2d cmd_vel_i = this->make_base_power(x_i, x_j, v_j);

        double cmd_vel_norm = cmd_vel_i.norm();
        if (cmd_vel_norm > this->max_vel_)
        {
            cmd_vel_i *= this->max_vel_ / cmd_vel_norm;
        }

        cmd_vels.col(i) = cmd_vel_i;
    }

    return cmd_vels;
}

Eigen::MatrixXd boid_node::remove_col(const Eigen::MatrixXd& A, int k)
{
    Eigen::MatrixXd B(A.rows(), A.cols() - 1);
    B.leftCols(k) = A.leftCols(k);
    B.rightCols(A.cols() - k - 1) = A.rightCols(A.cols() - k - 1);
    return B;
}

#ifndef INROF_SWERM_BOID_NODE_LIBRARY_ONLY
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<boid_node> node = std::make_shared<boid_node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
#endif
