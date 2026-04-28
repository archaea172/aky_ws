#include "boid.hpp"

using namespace std::chrono_literals;

boid_node::boid_node()
: rclcpp::Node("boid")
{
    this->declare_parameter<int>("boid_num");
    this->boid_num_ = this->get_parameter("boid_num").as_int();

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
    this->pos_matrix_.resize(2, this->boid_num_);
    this->vel_matrix_.resize(2, this->boid_num_);

    this->control_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        50ms,
        std::bind(&boid_node::control_callback, this)
    );
}

void boid_node::odom_callback(int id, nav_msgs::msg::Odometry::ConstSharedPtr rxdata)
{
    if (id > this->boid_num_)
    {
        RCLCPP_ERROR(this->get_logger(), "id is invalid. please check robot num");
        return;
    }

    this->odoms_[id] = *rxdata;

    this->pos_matrix_.col(id) << rxdata->pose.pose.position.x, rxdata->pose.pose.position.y;
    this->vel_matrix_.col(id) << rxdata->twist.twist.linear.x, rxdata->twist.twist.linear.y;
}

void boid_node::control_callback()
{
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
    # pragma omp parallel for
    for (size_t i = 0; i < x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2 > D_square) 
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
    # pragma omp parallel for
    for (size_t i = 0; i < x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2 > D_square) 
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

    return -vel;
}

Eigen::Vector2d boid_node::make_gravity_power(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j)
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    # pragma omp parallel for
    for (size_t i = 0; i < x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (Ir_2 > D_square) 
        {
            double use_d = std::max(sqrt(D_square), 1.0);
            sum += i_j_diff / use_d;
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return vel;
}

Eigen::MatrixXd boid_node::update_vels()
{
    Eigen::MatrixXd cmd_vels(2, this->boid_num_);
    # pragma omp parallel for
    for (int i = 0; i < this->boid_num_; ++i)
    {
        Eigen::Vector2d x_i = this->pos_matrix_.col(i);
        Eigen::MatrixXd x_j = this->remove_col(this->pos_matrix_, i);
        Eigen::MatrixXd v_j = this->remove_col(this->vel_matrix_, i);
        cmd_vels.col(i) = 
            this->k_separation * this->make_separation_power(x_i, x_j) +
            this->k_alignment * this->make_alignment_power(x_i, x_j, v_j) +
            this->k_gravity * this->make_gravity_power(x_i, x_j);
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<boid_node> node = std::make_shared<boid_node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
