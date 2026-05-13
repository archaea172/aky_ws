#include "lifecycle_server.hpp"

using namespace std::chrono_literals;

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
    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::Client<lifecycle_msgs::srv::GetAvailableTransitions>::SharedPtr get_available_transition_client = this->create_client<lifecycle_msgs::srv::GetAvailableTransitions>(
        goal->node_name + "/get_available_transitions",
        rmw_qos_profile_services_default,
        callback_group
    );
    if (!get_available_transition_client->wait_for_service(100ms))
    {
        RCLCPP_WARN(this->get_logger(), "Lifecycle node '%s' does not exist.", goal->node_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    lifecycle_msgs::srv::GetAvailableTransitions::Request::SharedPtr request = std::make_shared<lifecycle_msgs::srv::GetAvailableTransitions::Request>();
    auto future = get_available_transition_client->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Check available transition...");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group, this->get_node_base_interface());
    if (executor.spin_until_future_complete(future, 100ms) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to get available transitions from '%s'.", goal->node_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    const auto response = future.get();

    const auto transition_reaches_goal_state = [](const uint8_t transition_id, const uint8_t state_id) {
        using State = lifecycle_msgs::msg::State;
        using Transition = lifecycle_msgs::msg::Transition;

        switch (transition_id)
        {
            case Transition::TRANSITION_CONFIGURE:
            case Transition::TRANSITION_DEACTIVATE:
                return state_id == State::PRIMARY_STATE_INACTIVE;
            case Transition::TRANSITION_CLEANUP:
                return state_id == State::PRIMARY_STATE_UNCONFIGURED;
            case Transition::TRANSITION_ACTIVATE:
                return state_id == State::PRIMARY_STATE_ACTIVE;
            case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
            case Transition::TRANSITION_INACTIVE_SHUTDOWN:
            case Transition::TRANSITION_ACTIVE_SHUTDOWN:
                return state_id == State::PRIMARY_STATE_FINALIZED;
            default:
                return false;
        }
    };

    for (const auto & transition : response->available_transitions)
    {
        if (transition_reaches_goal_state(transition.transition.id, goal->state.id))
        {
            RCLCPP_INFO(this->get_logger(), "start to change state!");
            return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
        }
    }

    RCLCPP_WARN(
        this->get_logger(),
        "Transition to state %u is not available on '%s'.",
        static_cast<unsigned int>(goal->state.id),
        goal->node_name.c_str()
    );

    return rclcpp_action::GoalResponse::REJECT;
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
