#include "ros/follower_node.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<follower_node> node = std::make_shared<follower_node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
