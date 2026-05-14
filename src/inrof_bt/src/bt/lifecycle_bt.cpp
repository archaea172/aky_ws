#include "bt/lifecycle_bt.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <limits>
#include <string>

#include <behaviortree_ros2/plugins.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace
{
using State = lifecycle_msgs::msg::State;

std::string normalize_state_name(std::string state)
{
    const auto first = state.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
        return {};
    }

    const auto last = state.find_last_not_of(" \t\n\r");
    state = state.substr(first, last - first + 1);

    std::transform(state.begin(), state.end(), state.begin(), [](const unsigned char ch) {
        if (ch == '-' || ch == ' ') {
            return '_';
        }
        return static_cast<char>(std::tolower(ch));
    });

    const std::string primary_state_prefix = "primary_state_";
    if (state.rfind(primary_state_prefix, 0) == 0) {
        state.erase(0, primary_state_prefix.size());
    }

    return state;
}

const char * lifecycle_state_label(const uint8_t id)
{
    switch (id) {
        case State::PRIMARY_STATE_UNCONFIGURED:
            return "unconfigured";
        case State::PRIMARY_STATE_INACTIVE:
            return "inactive";
        case State::PRIMARY_STATE_ACTIVE:
            return "active";
        case State::PRIMARY_STATE_FINALIZED:
            return "finalized";
        default:
            return "";
    }
}

bool assign_lifecycle_state(const uint8_t id, State & state)
{
    const char * label = lifecycle_state_label(id);
    if (std::string(label).empty()) {
        return false;
    }

    state.id = id;
    state.label = label;
    return true;
}

bool parse_lifecycle_state(const std::string & input, State & state)
{
    const auto normalized = normalize_state_name(input);
    if (normalized.empty()) {
        return false;
    }

    if (std::all_of(normalized.begin(), normalized.end(), [](const unsigned char ch) {
            return std::isdigit(ch) != 0;
        }))
    {
        unsigned long value = 0;
        try {
            value = std::stoul(normalized);
        } catch (const std::exception &) {
            return false;
        }

        if (value > std::numeric_limits<uint8_t>::max()) {
            return false;
        }
        return assign_lifecycle_state(static_cast<uint8_t>(value), state);
    }

    if (normalized == "unconfigured") {
        return assign_lifecycle_state(State::PRIMARY_STATE_UNCONFIGURED, state);
    }
    if (normalized == "inactive") {
        return assign_lifecycle_state(State::PRIMARY_STATE_INACTIVE, state);
    }
    if (normalized == "active") {
        return assign_lifecycle_state(State::PRIMARY_STATE_ACTIVE, state);
    }
    if (normalized == "finalized" || normalized == "shutdown") {
        return assign_lifecycle_state(State::PRIMARY_STATE_FINALIZED, state);
    }

    return false;
}
}  // namespace

BT::PortsList LifecycleAction::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("node_name"),
        BT::InputPort<std::string>("state"),
    });
}

bool LifecycleAction::setGoal(Goal & goal)
{
    const auto node_name = getInput<std::string>("node_name");
    if (!node_name) {
        RCLCPP_ERROR(logger(), "Missing required input [node_name]: %s", node_name.error().c_str());
        return false;
    }

    const auto state = getInput<std::string>("state");
    if (!state) {
        RCLCPP_ERROR(logger(), "Missing required input [state]: %s", state.error().c_str());
        return false;
    }

    goal.node_name = node_name.value();
    if (goal.node_name.empty()) {
        RCLCPP_ERROR(logger(), "Input [node_name] must not be empty");
        return false;
    }

    if (!parse_lifecycle_state(state.value(), goal.state)) {
        RCLCPP_ERROR(
            logger(),
            "Invalid lifecycle state '%s'. Use unconfigured, inactive, active, finalized, shutdown, or a primary state id.",
            state.value().c_str());
        return false;
    }

    RCLCPP_INFO(
        logger(),
        "Sending lifecycle goal: node='%s', state='%s' (%u)",
        goal.node_name.c_str(),
        goal.state.label.c_str(),
        static_cast<unsigned int>(goal.state.id));
    return true;
}

BT::NodeStatus LifecycleAction::onResultReceived(const WrappedResult & result)
{
    if (result.result && result.result->success) {
        RCLCPP_INFO(logger(), "Lifecycle goal succeeded: %s", result.result->msg.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
        logger(),
        "Lifecycle goal failed: %s",
        result.result ? result.result->msg.c_str() : "empty result");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LifecycleAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    (void) feedback;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LifecycleAction::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "Lifecycle goal failed with error code: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}

void LifecycleAction::onHalt()
{
    RCLCPP_INFO(logger(), "Halting lifecycle goal");
}

CreateRosNodePlugin(LifecycleAction, "LifecycleAction");
