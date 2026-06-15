#include "bt/leader_pos_bt.hpp"

BT::PortsList LeaderPosAction::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("start_x"),
        BT::InputPort<double>("start_y"),
        BT::InputPort<double>("goal_x"),
        BT::InputPort<double>("goal_y"),
        BT::InputPort<std::string>("waypoints", ""),
        BT::InputPort<double>("max_speed"),
    });
}

bool LeaderPosAction::setGoal(Goal & goal)
{
    const auto start_x = getInput<double>("start_x");
    if (!start_x) {
        RCLCPP_ERROR(logger(), "Missing required input [start_x]: %s", start_x.error().c_str());
        return false;
    }

    const auto start_y = getInput<double>("start_y");
    if (!start_y) {
        RCLCPP_ERROR(logger(), "Missing required input [start_y]: %s", start_y.error().c_str());
        return false;
    }

    const auto goal_x = getInput<double>("goal_x");
    if (!goal_x) {
        RCLCPP_ERROR(logger(), "Missing required input [goal_x]: %s", goal_x.error().c_str());
        return false;
    }

    const auto goal_y = getInput<double>("goal_y");
    if (!goal_y) {
        RCLCPP_ERROR(logger(), "Missing required input [goal_y]: %s", goal_y.error().c_str());
        return false;
    }

    const auto waypoints_str = getInput<std::string>("waypoints");
    if (!waypoints_str) {
        RCLCPP_ERROR(logger(), "Missing required input [waypoints]: %s", waypoints_str.error().c_str());
        return false;
    }

    const auto max_speed = getInput<double>("max_speed");
    if (!max_speed) {
        RCLCPP_ERROR(logger(), "Missing required input [max_speed]: %s", max_speed.error().c_str());
        return false;
    }

    goal.start_pos.pose.position.x = start_x.value();
    goal.start_pos.pose.position.y = start_y.value();
    goal.goal_pos.pose.position.x = goal_x.value();
    goal.goal_pos.pose.position.y = goal_y.value();
    goal.max_speed = static_cast<float>(max_speed.value());

    goal.waypoints.clear();
    if (!waypoints_str.value().empty()) {
        for (const auto waypoint_text : BT::splitString(waypoints_str.value(), ';')) {
            const auto xy = BT::splitString(waypoint_text, ',');

            if (xy.size() != 2) {
                RCLCPP_ERROR(logger(), "Invalid waypoint format. Use x,y;x,y;...");
                return false;
            }

            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.pose.position.x = BT::convertFromString<double>(xy[0]);
            waypoint.pose.position.y = BT::convertFromString<double>(xy[1]);

            goal.waypoints.push_back(waypoint);
        }
    }

    return true;
}

BT::NodeStatus LeaderPosAction::onResultReceived(const WrappedResult & result)
{
    if (result.result && result.result->success) {
        RCLCPP_INFO(logger(), "leader pos succeeded: %s", result.result->msg.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
        logger(),
        "leader pos failed: %s",
        result.result ? result.result->msg.c_str() : "empty result");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LeaderPosAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    (void) feedback;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LeaderPosAction::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "leader pos failed with error code: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}

void LeaderPosAction::onHalt()
{
    RCLCPP_INFO(logger(), "Halting lifecycle goal");
}
