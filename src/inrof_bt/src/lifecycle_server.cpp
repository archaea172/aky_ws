#include "lifecycle_server.hpp"

LifecycleServer::LifecycleServer()
: rclcpp::Node("lifecycle_server")
{
    this->action_server_ = rclcpp_action::create_server<Lifecycle>(
        this,
        "lifecycle_server",
        std::bind(&LifecycleServer::handle_goal, this, _1, _2),
        std::bind(&LifecycleServer::handle_cancel,this, _1),
        std::bind(&LifecycleServer::handle_accepted, this, _1)
    );
}

rclcpp_action::GoalResponse LifecycleServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Lifecycle::Goal> goal
)
{
    // todo そのノードが存在するかと状態遷移が正しいかの確認

    RCLCPP_INFO(this->get_logger(), "arm start to move!");
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse LifecycleServer::handle_cancel(
    const std::shared_ptr<GoalHandleLifecycle> goal_handle
)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LifecycleServer::handle_accepted(const std::shared_ptr<GoalHandleLifecycle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    goal_handle->execute();
    std::thread{std::bind(&LifecycleServer::execute, this, _1), goal_handle}.detach();
}

void LifecycleServer::execute(const std::shared_ptr<GoalHandleLifecycle> goal_handle)
{
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LifecycleServer> node = std::make_shared<LifecycleServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
