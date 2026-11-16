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

#include <optional>
#include <type_traits>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_util/container.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"

#define INPUT_KEY_PARAM_NAME "parameter"
#define INPUT_KEY_NODE_NAME "node"

namespace auto_apms_behavior_tree
{

class HasParameter : public core::RosServiceNode<rcl_interfaces::srv::ListParameters>
{
public:
  HasParameter(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
    // A node manifest may name the service instead, with (input:port) substitutions (registration option 'topic', e.g.
    // "(input:robot)/my_node/list_parameters" to target a node of the robot named by the input port 'robot'). Then the
    // base class has created the client, or creates it when the node is ticked first if the name refers to a
    // blackboard entry that is not set yet.
    if (context_.getRegistrationOptions().topic != core::NodeRegistrationOptions().topic) return;

    // If input port is empty or not provided, refer to this ROS 2 node as the target. Otherwise, the port's value is
    // resolved like the (input:port) pattern of a node manifest's topic: a static name, or a blackboard entry that is
    // set before the tree is created (e.g. by the entry point), as the client is created now
    std::string node_name = context_.getFullyQualifiedRosNodeName();
    if (
      config.input_ports.find(INPUT_KEY_NODE_NAME) != config.input_ports.end() &&
      !config.input_ports.at(INPUT_KEY_NODE_NAME).empty()) {
      const BT::Expected<std::string> expected_node_name = getInput<std::string>(INPUT_KEY_NODE_NAME);
      if (!expected_node_name || expected_node_name.value().empty()) {
        throw exceptions::RosNodeError(
          context_.getFullyQualifiedTreeNodeName(this) +
          " - Cannot determine the targeted ROS 2 node from input port '" + INPUT_KEY_NODE_NAME + "' (" +
          config.input_ports.at(INPUT_KEY_NODE_NAME) +
          "): " + (expected_node_name ? std::string("value is empty") : expected_node_name.error()) +
          ". A blackboard entry must be set before the tree is created.");
      }
      node_name = expected_node_name.value();
    }
    createClient(node_name + "/list_parameters");
  }

  static BT::PortsList providedPorts()
  {
    // We do not use the default port for the service name
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_NODE_NAME,
        "Name of the targeted ROS 2 node (a blackboard entry must be set before the tree is created). Leave empty to "
        "target this executor's node."),
      BT::InputPort<std::string>(INPUT_KEY_PARAM_NAME, "Name of the parameter.")};
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<std::string> expected_name = getInput<std::string>(INPUT_KEY_PARAM_NAME);
    if (!expected_name || expected_name.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - Parameter name must not be empty.", context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_name, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_name.error().c_str());
      return false;
    }
    has_parameter_name_ = expected_name.value();
    request->depth = Request::DEPTH_RECURSIVE;
    request->prefixes = {};
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (auto_apms_util::contains(response->result.names, has_parameter_name_)) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string has_parameter_name_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::HasParameter)
