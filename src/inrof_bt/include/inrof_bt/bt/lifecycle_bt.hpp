#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <swerm_msgs/action/lifecycle.hpp>

class LifecycleAction : public BT::RosActionNode<swerm_msgs::action::Lifecycle>
{
public:
  LifecycleAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<swerm_msgs::action::Lifecycle>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult & result) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;
};
