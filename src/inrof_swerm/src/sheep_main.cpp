#include "ros/sheep_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<sheep_node> node = std::make_shared<sheep_node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
