// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_behavior_tree_core/node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#define INPUT_KEY_NODE_NAME "node"
#define INPUT_KEY_TRANSITION "transition"
#define OUTPUT_KEY_STATE_ID "state_id"
#define OUTPUT_KEY_STATE_LABEL "state_label"

namespace auto_apms_behavior_tree
{

/**
 * @brief Retrieves the current lifecycle state of a ROS 2 managed node.
 *
 * The targeted node's service name is constructed from the 'node' input port as
 * '<node>/get_state'. The numeric state ID and its human-readable label are written
 * to the respective output ports.
 *
 * State IDs are defined in lifecycle_msgs/msg/State:
 *   UNKNOWN=0, UNCONFIGURED=1, INACTIVE=2, ACTIVE=3, FINALIZED=4
 */
class LifecycleGetState : public core::RosServiceNode<lifecycle_msgs::srv::GetState>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    // We do not use the default 'topic' port — the service name is derived from the 'node' port
    // via the NodeRegistrationOptions::topic pattern "(input:node)/get_state".
    return {
      BT::InputPort<std::string>(INPUT_KEY_NODE_NAME, "Name of the targeted lifecycle node."),
      BT::OutputPort<uint8_t>(OUTPUT_KEY_STATE_ID, "Numeric ID of the current lifecycle state."),
      BT::OutputPort<std::string>(OUTPUT_KEY_STATE_LABEL, "Human-readable label of the current lifecycle state."),
    };
  }

  bool setRequest(Request::SharedPtr & /*request*/) override final
  {
    // Initialize output ports with defaults so downstream nodes always read valid values
    setOutput(OUTPUT_KEY_STATE_ID, uint8_t{lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN});
    setOutput(OUTPUT_KEY_STATE_LABEL, std::string{});
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (const BT::Result res = setOutput(OUTPUT_KEY_STATE_ID, response->current_state.id); !res) {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), res.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (const BT::Result res = setOutput(OUTPUT_KEY_STATE_LABEL, response->current_state.label); !res) {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), res.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/**
 * @brief Template for ChangeState nodes.
 *
 * The targeted node's service name is constructed from the 'node' input port as
 * '<node>/change_state'. When TRANSITION_ID < 0 (the generic case), the transition ID is read
 * from the 'transition' input port at tick time, allowing full runtime configurability.
 * For non-negative TRANSITION_ID values the transition is baked into the class at compile time
 * and no 'transition' port is exposed — only 'node' is required.
 *
 * Transition IDs are defined in lifecycle_msgs/msg/Transition:
 *   configure=1, cleanup=2, activate=3, deactivate=4,
 *   unconfigured_shutdown=5, inactive_shutdown=6, active_shutdown=7
 *
 * @tparam TRANSITION_ID Compile-time transition ID. Use -1 (default) for the generic version.
 */
template <int TRANSITION_ID = -1>
class ChangeStateTemplate : public core::RosServiceNode<lifecycle_msgs::srv::ChangeState>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    // We do not use the default 'topic' port — the service name is derived from the 'node' port
    // via the NodeRegistrationOptions::topic pattern "(input:node)/change_state".
    BT::PortsList ports = {
      BT::InputPort<std::string>(INPUT_KEY_NODE_NAME, "Name of the targeted lifecycle node."),
    };
    if constexpr (TRANSITION_ID < 0) {
      ports.insert(
        BT::InputPort<int>(
          INPUT_KEY_TRANSITION,
          "ID of the lifecycle state machine transition to execute. Refer to lifecycle_msgs/msg/Transition for "
          "available values (configure=1, cleanup=2, activate=3, deactivate=4, "
          "unconfigured_shutdown=5, inactive_shutdown=6, active_shutdown=7)."));
    }
    return ports;
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    if constexpr (TRANSITION_ID >= 0) {
      requested_transition_id_ = static_cast<uint8_t>(TRANSITION_ID);
    } else {
      const BT::Expected<int> expected = getInput<int>(INPUT_KEY_TRANSITION);
      if (!expected) {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return false;
      }
      if (expected.value() < 0 || expected.value() > 255) {
        RCLCPP_ERROR(
          logger_, "%s - Transition ID %d is out of the valid range [0, 255].",
          context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.value());
        return false;
      }
      requested_transition_id_ = static_cast<uint8_t>(expected.value());
    }
    request->transition.id = requested_transition_id_;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (!response->success) {
      RCLCPP_ERROR(
        logger_, "%s - Failed to execute lifecycle transition (id: %u) via service '%s'.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), static_cast<unsigned>(requested_transition_id_),
        getServiceName().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  uint8_t requested_transition_id_{0};
};

// Generic version — transition ID is read from the 'transition' input port at tick time
class LifecycleChangeState : public ChangeStateTemplate<-1>
{
public:
  using ChangeStateTemplate::ChangeStateTemplate;
};

// Convenience specializations for the primary lifecycle transitions

/// @brief Sends the 'configure' transition (id=1) to a lifecycle node.
class LifecycleConfigure : public ChangeStateTemplate<lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE>
{
public:
  using ChangeStateTemplate::ChangeStateTemplate;
};

/// @brief Sends the 'cleanup' transition (id=2) to a lifecycle node.
class LifecycleCleanup : public ChangeStateTemplate<lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP>
{
public:
  using ChangeStateTemplate::ChangeStateTemplate;
};

/// @brief Sends the 'activate' transition (id=3) to a lifecycle node.
class LifecycleActivate : public ChangeStateTemplate<lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE>
{
public:
  using ChangeStateTemplate::ChangeStateTemplate;
};

/// @brief Sends the 'deactivate' transition (id=4) to a lifecycle node.
class LifecycleDeactivate : public ChangeStateTemplate<lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE>
{
public:
  using ChangeStateTemplate::ChangeStateTemplate;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LifecycleGetState)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LifecycleChangeState)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LifecycleConfigure)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LifecycleCleanup)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LifecycleActivate)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LifecycleDeactivate)
