#include <memory>
#include <string>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "bt/lifecycle_bt.hpp"
#include "bt/leader_pos_bt.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("inrof_bt_node");
    const std::string default_xml =
        ament_index_cpp::get_package_share_directory("inrof_bt") + "/config/main_bt.xml";
    node->declare_parameter<std::string>("bt_xml_file", default_xml);
    node->declare_parameter<int>("wait_for_server_timeout_ms", 5000);

    std::string bt_xml_file;
    int wait_for_server_timeout_ms;
    node->get_parameter("bt_xml_file", bt_xml_file);
    node->get_parameter("wait_for_server_timeout_ms", wait_for_server_timeout_ms);
    RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", bt_xml_file.c_str());

    BT::BehaviorTreeFactory factory;
    
    BT::RosNodeParams lifecycle_params;
    lifecycle_params.nh = node;
    lifecycle_params.default_port_value = "lifecycle_server";
    lifecycle_params.server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    lifecycle_params.wait_for_server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    
    BT::RosNodeParams leader_pos_params;
    leader_pos_params.nh = node;
    leader_pos_params.default_port_value = "leader_pos";
    leader_pos_params.server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    leader_pos_params.wait_for_server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    
    
    factory.registerNodeType<LifecycleAction>("lifecycle_action", lifecycle_params);
    factory.registerNodeType<LeaderPosAction>("leader_pos_action", leader_pos_params);
    BT::Tree tree = factory.createTreeFromFile(bt_xml_file);
    BT::StdCoutLogger logger_cout(tree);
    tree.tickWhileRunning();
    RCLCPP_INFO(
        node->get_logger(),
        "BT finished with status: %s",
        BT::toStr(tree.rootNode()->status(), false).c_str());

    rclcpp::shutdown();
    return 0;
}
