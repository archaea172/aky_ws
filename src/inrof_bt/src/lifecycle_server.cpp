#include "lifecycle_server.hpp"

using namespace std::chrono_literals;

namespace
{
bool transition_reaches_goal_state(const uint8_t transition_id, const uint8_t state_id)
{
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
}
}

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
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Lifecycle::Result>();

    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group, this->get_node_base_interface());

    auto fail = [&](const std::string & msg) {
        result->success = false;
        result->msg = msg;
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        goal_handle->abort(result);
    };

    if (goal_handle->is_canceling())
    {
        result->success = false;
        result->msg = "Goal canceled.";
        goal_handle->canceled(result);
        return;
    }

    rclcpp::Client<lifecycle_msgs::srv::GetAvailableTransitions>::SharedPtr get_available_transition_client = this->create_client<lifecycle_msgs::srv::GetAvailableTransitions>(
        goal->node_name + "/get_available_transitions",
        rmw_qos_profile_services_default,
        callback_group
    );
    if (!get_available_transition_client->wait_for_service(100ms))
    {
        fail("Lifecycle node '" + goal->node_name + "' does not exist.");
        return;
    }

    auto get_transitions_request = std::make_shared<lifecycle_msgs::srv::GetAvailableTransitions::Request>();
    auto get_transitions_future = get_available_transition_client->async_send_request(get_transitions_request);
    if (executor.spin_until_future_complete(get_transitions_future, 100ms) != rclcpp::FutureReturnCode::SUCCESS)
    {
        fail("Failed to get available transitions from '" + goal->node_name + "'.");
        return;
    }

    const auto transitions_response = get_transitions_future.get();
    lifecycle_msgs::msg::Transition target_transition;
    bool found_transition = false;
    for (const auto & transition : transitions_response->available_transitions)
    {
        if (transition_reaches_goal_state(transition.transition.id, goal->state.id))
        {
            target_transition = transition.transition;
            found_transition = true;
            break;
        }
    }
    if (!found_transition)
    {
        fail("Transition to requested state is not available on '" + goal->node_name + "'.");
        return;
    }

    if (goal_handle->is_canceling())
    {
        result->success = false;
        result->msg = "Goal canceled.";
        goal_handle->canceled(result);
        return;
    }

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(
        goal->node_name + "/change_state",
        rmw_qos_profile_services_default,
        callback_group
    );
    if (!change_state_client->wait_for_service(100ms))
    {
        fail("Lifecycle node '" + goal->node_name + "' does not exist.");
        return;
    }

    auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_request->transition = target_transition;
    auto change_state_future = change_state_client->async_send_request(change_state_request);
    if (executor.spin_until_future_complete(change_state_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        fail("Failed to change state on '" + goal->node_name + "'.");
        return;
    }

    result->success = change_state_future.get()->success;
    if (!result->success)
    {
        result->msg = "Failed to change state on '" + goal->node_name + "'.";
        RCLCPP_WARN(this->get_logger(), "%s", result->msg.c_str());
        goal_handle->abort(result);
        return;
    }

    result->msg = "Changed state on '" + goal->node_name + "'.";
    RCLCPP_INFO(this->get_logger(), "%s", result->msg.c_str());
    goal_handle->succeed(result);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LifecycleServer> node = std::make_shared<LifecycleServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
