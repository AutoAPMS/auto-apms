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

#include <sstream>
#include <string>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

#define INPUT_KEY_CONTAINER "container"
#define INPUT_KEY_PACKAGE "package"
#define INPUT_KEY_PLUGIN "plugin"
#define INPUT_KEY_NODE_NAME "node_name"
#define INPUT_KEY_NODE_NAMESPACE "node_namespace"
#define INPUT_KEY_LOG_LEVEL "log_level"
#define INPUT_KEY_REMAP_RULES "remap_rules"
#define PORT_KEY_UNIQUE_ID "unique_id"
#define OUTPUT_KEY_FULL_NODE_NAME "full_node_name"

namespace auto_apms_behavior_tree
{

/**
 * @brief Loads a composable node plugin into a running ROS 2 component container.
 *
 * The targeted container's service name is constructed from the 'container' input port as
 * '<container>/_container/load_node'. Both 'package' and 'plugin' are required inputs.
 * Optional inputs allow fine-grained control over the loaded node's name, namespace, log level
 * and static remapping rules.
 *
 * On success, the container-local unique ID and the full node name are written to the output ports,
 * allowing subsequent behavior tree nodes (e.g. CompositionUnloadNode) to reference the loaded instance.
 */
class CompositionLoadNode : public core::RosServiceNode<composition_interfaces::srv::LoadNode>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    // We do not use the default 'topic' port — the service name is derived from the 'container' port
    // via the NodeRegistrationOptions::topic pattern "(input:container)/_container/load_node".
    return {
      BT::InputPort<std::string>(INPUT_KEY_CONTAINER, "Name of the component container to load the node into."),
      BT::InputPort<std::string>(INPUT_KEY_PACKAGE, "Name of the ROS 2 package containing the composable node plugin."),
      BT::InputPort<std::string>(INPUT_KEY_PLUGIN, "Name of the composable node plugin to load."),
      BT::InputPort<std::string>(
        INPUT_KEY_NODE_NAME, "", "Assigned name of the loaded node. Leave empty to use the plugin's default name."),
      BT::InputPort<std::string>(
        INPUT_KEY_NODE_NAMESPACE, "",
        "Assigned namespace of the loaded node. Leave empty to use the plugin's default namespace."),
      BT::InputPort<std::string>(
        INPUT_KEY_LOG_LEVEL, "UNSET",
        "Log level for the loaded node. Must be one of [UNSET, DEBUG, INFO, WARN, ERROR, FATAL] "
        "(case-insensitive). UNSET (default) means the container's log level is inherited."),
      BT::InputPort<std::string>(
        INPUT_KEY_REMAP_RULES, "",
        "Semicolon-separated list of static remapping rules applied to the loaded node "
        "(e.g. 'old_topic:=new_topic;foo:=bar')."),
      BT::OutputPort<uint64_t>(
        PORT_KEY_UNIQUE_ID, "Container-local unique ID assigned to the loaded node (0 if loading failed)."),
      BT::OutputPort<std::string>(
        OUTPUT_KEY_FULL_NODE_NAME, "Full node name (namespace/name) of the loaded node (empty if loading failed)."),
    };
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<std::string> expected_package = getInput<std::string>(INPUT_KEY_PACKAGE);
    if (!expected_package || expected_package.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - Package name must not be empty.", context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_package, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_package.error().c_str());
      return false;
    }
    request->package_name = expected_package.value();

    const BT::Expected<std::string> expected_plugin = getInput<std::string>(INPUT_KEY_PLUGIN);
    if (!expected_plugin || expected_plugin.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - Plugin name must not be empty.", context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_plugin, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_plugin.error().c_str());
      return false;
    }
    request->plugin_name = expected_plugin.value();
    requested_plugin_name_ = expected_plugin.value();

    request->node_name = getInput<std::string>(INPUT_KEY_NODE_NAME).value_or("");
    request->node_namespace = getInput<std::string>(INPUT_KEY_NODE_NAMESPACE).value_or("");

    // Initialize output ports with defaults so downstream nodes always read valid values
    setOutput(PORT_KEY_UNIQUE_ID, uint64_t{0});
    setOutput(OUTPUT_KEY_FULL_NODE_NAME, std::string{});

    // Interpret log level string the same way as the Logger node
    const std::string level_str = getInput<std::string>(INPUT_KEY_LOG_LEVEL).value_or("UNSET");
    int log_level = 0;
    if (
      rcutils_logging_severity_level_from_string(level_str.c_str(), rcutils_get_default_allocator(), &log_level) !=
      RCUTILS_RET_OK) {
      const std::string error = rcutils_get_error_string().str;
      rcutils_reset_error();
      RCLCPP_ERROR(
        logger_, "%s - Cannot convert log level '%s' to a valid severity: %s",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), level_str.c_str(), error.c_str());
      return false;
    }
    request->log_level = static_cast<uint8_t>(log_level);

    // Parse semicolon-separated remap rules and trim surrounding whitespace from each rule
    const std::string remap_str = getInput<std::string>(INPUT_KEY_REMAP_RULES).value_or("");
    if (!remap_str.empty()) {
      std::istringstream ss(remap_str);
      std::string rule;
      while (std::getline(ss, rule, ';')) {
        const auto first = rule.find_first_not_of(" \t");
        const auto last = rule.find_last_not_of(" \t");
        if (first != std::string::npos) {
          request->remap_rules.push_back(rule.substr(first, last - first + 1));
        }
      }
    }

    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (!response->success) {
      RCLCPP_ERROR(
        logger_, "%s - Failed to load composable node '%s' via service '%s': %s",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), requested_plugin_name_.c_str(), getServiceName().c_str(),
        response->error_message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput(PORT_KEY_UNIQUE_ID, response->unique_id);
    setOutput(OUTPUT_KEY_FULL_NODE_NAME, response->full_node_name);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::string requested_plugin_name_;
};

/**
 * @brief Unloads a composable node from a running ROS 2 component container using its unique ID.
 *
 * The targeted container's service name is constructed from the 'container' input port as
 * '<container>/_container/unload_node'. The unique ID is typically obtained from a preceding
 * CompositionLoadNode node via its 'unique_id' output port.
 */
class CompositionUnloadNode : public core::RosServiceNode<composition_interfaces::srv::UnloadNode>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    // We do not use the default 'topic' port — the service name is derived from the 'container' port
    // via the NodeRegistrationOptions::topic pattern "(input:container)/_container/unload_node".
    return {
      BT::InputPort<std::string>(INPUT_KEY_CONTAINER, "Name of the component container to unload the node from."),
      BT::InputPort<uint64_t>(PORT_KEY_UNIQUE_ID, "Container-local unique ID of the composable node to unload."),
    };
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<uint64_t> expected_id = getInput<uint64_t>(PORT_KEY_UNIQUE_ID);
    if (!expected_id) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_id.error().c_str());
      return false;
    }
    request->unique_id = expected_id.value();
    requested_unique_id_ = expected_id.value();
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (!response->success) {
      RCLCPP_ERROR(
        logger_, "%s - Failed to unload composable node (id: %s) via service '%s': %s",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), std::to_string(requested_unique_id_).c_str(),
        getServiceName().c_str(), response->error_message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  uint64_t requested_unique_id_{0};
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::CompositionLoadNode)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::CompositionUnloadNode)
