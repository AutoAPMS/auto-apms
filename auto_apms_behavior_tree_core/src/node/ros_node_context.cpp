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

#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"

#include <iterator>
#include <regex>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_util/logging.hpp"

namespace auto_apms_behavior_tree::core
{
RosNodeContext::RosNodeContext(
  rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group,
  rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
  const NodeRegistrationOptions & options)
: ros_node_name_(ros_node ? ros_node->get_name() : ""),
  fully_qualified_ros_node_name_(ros_node ? ros_node->get_fully_qualified_name() : ""),
  base_logger_(ros_node ? ros_node->get_logger() : rclcpp::get_logger("")),
  nh_(ros_node),
  cb_group_(tree_node_waitables_callback_group),
  executor_(tree_node_waitables_executor),
  registration_options_(options)
{
}

std::string RosNodeContext::getROSNodeName() const { return ros_node_name_; }

std::string RosNodeContext::getFullyQualifiedRosNodeName() const { return fully_qualified_ros_node_name_; }

rclcpp::Logger RosNodeContext::getBaseLogger() const { return base_logger_; }

rclcpp::Logger RosNodeContext::getChildLogger(const std::string & name)
{
  const rclcpp::Logger child_logger = base_logger_.get_child(name);
  if (!registration_options_.logger_level.empty()) {
    try {
      auto_apms_util::setLoggingSeverity(child_logger, registration_options_.logger_level);
    } catch (const auto_apms_util::exceptions::SetLoggingSeverityError & e) {
      RCLCPP_ERROR(
        base_logger_,
        "Failed to set the logging severity for the child logger using the node's registration options: %s", e.what());
    }
  }
  return child_logger;
}

rclcpp::Time RosNodeContext::getCurrentTime() const
{
  if (const rclcpp::Node::SharedPtr node = nh_.lock()) {
    return node->now();
  }
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

std::string RosNodeContext::getFullyQualifiedTreeNodeName(const BT::TreeNode * node, bool with_class_name) const
{
  // NOTE: registrationName() is empty during construction as this member is first set after the factory constructed the
  // object
  const std::string instance_name = node->name();
  const std::string registration_name = node->registrationName();
  if (!registration_name.empty() && instance_name != registration_name) {
    if (with_class_name) {
      return instance_name + " (" + registration_name + " : " + registration_options_.class_name + ")";
    }
    return instance_name + " (" + registration_name + ")";
  }
  return with_class_name ? (instance_name + " (" + registration_options_.class_name + ")") : instance_name;
}

YAML::Node RosNodeContext::getExtraOptions() const { return registration_options_.extra; }

rclcpp::Node::SharedPtr RosNodeContext::getRosNode() const
{
  const rclcpp::Node::SharedPtr node = nh_.lock();
  if (!node) {
    throw exceptions::RosNodeError(
      "Cannot access the associated ROS 2 node from the RosNodeContext because the weak pointer expired. The tree "
      "node doesn't take ownership of it.");
  }
  return node;
}

rclcpp::CallbackGroup::SharedPtr RosNodeContext::getWaitablesCallbackGroup() const { return cb_group_.lock(); }

rclcpp::executors::SingleThreadedExecutor::SharedPtr RosNodeContext::getWaitablesExecutor() const
{
  return executor_.lock();
}

const NodeRegistrationOptions & RosNodeContext::getRegistrationOptions() const { return registration_options_; }

BT::Expected<std::string> RosNodeContext::getTopicName(const BT::TreeNode * node) const
{
  std::string res = registration_options_.topic;
  if (res.empty()) {
    return nonstd::make_unexpected(
      getFullyQualifiedTreeNodeName(node) +
      " - Cannot get the name of the node's associated ROS 2 topic: Registration option '" +
      NodeRegistrationOptions::PARAM_NAME_ROS2TOPIC + "' is empty.");
  }
  BT::PortsRemapping input_ports = node->config().input_ports;

  // Parameter registration_options_.topic may contain substrings, that that are to be replaced with values retrieved
  // from a specific node input port. Must be something like (input:my_port) where 'my_port' is the key/name of the
  // BT::InputPort to use. Anything before or after the expression is kept and used as a prefix respectively suffix.
  const std::regex pattern(R"(\(input:([^)\s]+)\))");
  const std::sregex_iterator replace_begin(res.begin(), res.end(), pattern);
  const std::sregex_iterator replace_end = std::sregex_iterator();

  // We iterate over each substitution expression. If there are none, this for loop has no effect, and we simply return
  // the parameters value.
  for (std::sregex_iterator it = replace_begin; it != replace_end; ++it) {
    const std::smatch match = *it;
    const std::string input_port_key = match[1].str();

    // Search for the specified input port key in the list of input ports passed at construction time
    if (input_ports.find(input_port_key) != input_ports.end()) {
      // The input port has been found using input_port_key

      // Make sure its value is either a blackboard pointer or a static string (Must not be empty)
      if (input_ports.at(input_port_key).empty()) {
        return nonstd::make_unexpected(
          getFullyQualifiedTreeNodeName(node) +
          " - Cannot get the name of the node's associated ROS 2 topic: Input port '" + input_port_key +
          "' required by substring '" + match.str() + "' must not be empty.");
      }

      // We try to get the value from the node input port. If the value is a string pointing at a blackboard entry, this
      // may not work during construction time. In case the expected value contains an error, we forward it to indicate
      // we weren't successful.
      const BT::Expected<std::string> expected = node->getInput<std::string>(input_port_key);
      if (expected) {
        // Replace the respective substring with the value returned from getInput()
        res.replace(match.position(), match.length(), expected.value());
      } else {
        // Return expected (contains error) if value couldn't be retrieved from input ports
        return expected;
      }
    } else {
      return nonstd::make_unexpected(
        getFullyQualifiedTreeNodeName(node) +
        " - Cannot get the name of the node's associated ROS 2 topic: Input port '" + input_port_key +
        "' required by substring '" + match.str() + "' doesn't exist.");
    }
  }
  return res;
}

/**
 * @brief Parse alias port name and optional description from format "alias_name" or "alias_name (description)"
 *
 * The description is delimited by the outermost pair of round brackets, so it may itself contain further round
 * brackets, e.g. "alias_name (a description (with nested) brackets)" yields the description
 * "a description (with nested) brackets".
 * @param str Input string to parse.
 * @return Pair of alias name and description (empty if not provided).
 */
std::pair<std::string, std::string> parseAliasPortName(const std::string & str)
{
  // The description group is greedy ((.*) rather than ([^)]*)) so it extends to the last ')' in the string. This lets
  // the description contain round brackets of its own while the outermost pair still delimits it.
  static const std::regex alias_regex(R"(^\s*([^\s(]+)\s*(?:\((.*)\))?\s*$)");
  std::smatch match;
  if (std::regex_match(str, match, alias_regex)) {
    return {match[1].str(), match[2].str()};  // {alias_name, description}
  }
  return {str, ""};  // Fallback if regex doesn't match
}

void RosNodeContext::modifyProvidedPortsListForRegistration(BT::PortsList & ports_list) const
{
  // Add port aliases if specified in the node manifest (original port is kept for compatibility with the
  // implementation, but hidden in the node model)
  std::map<std::string, std::string> original_alias_name_map;
  for (const auto & [original_port_name, aliased_port_name] : registration_options_.port_alias) {
    if (ports_list.find(original_port_name) == ports_list.end()) {
      throw exceptions::NodeRegistrationError(
        "Cannot alias port '" + original_port_name + "' which is not provided by class '" +
        registration_options_.class_name + "'. The keys under " + NodeRegistrationOptions::PARAM_NAME_PORT_ALIAS +
        " must refer to a port implemented by the node.");
    }
    BT::PortInfo port_info = ports_list.at(original_port_name);
    const auto [aliased_name_cleaned, aliased_description] = parseAliasPortName(aliased_port_name);
    original_alias_name_map[original_port_name] = aliased_name_cleaned;

    // Update description if provided within round brackets
    if (!aliased_description.empty()) {
      port_info.setDescription(aliased_description);
    }

    // Insert additional port
    ports_list.insert({aliased_name_cleaned, port_info});
  }

  // Modify the default value of the ports if specified in the node manifest
  for (const auto & [port_name, new_default] : registration_options_.port_default) {
    if (ports_list.find(port_name) == ports_list.end()) {
      throw exceptions::NodeRegistrationError(
        "Cannot set default value for port '" + port_name + "' which is not provided by class '" +
        registration_options_.class_name + "'. The keys under " + NodeRegistrationOptions::PARAM_NAME_PORT_DEFAULT +
        " must refer to a port implemented by the node.");
    }
    // We're passing the new default value as string. Conversion is done when getting the port value during
    // execution (also allows blackboard pointers)
    ports_list.at(port_name).setDefaultValue(new_default);

    // If it's an aliased port, we also need to update the default of the original port and vice versa
    if (original_alias_name_map.find(port_name) != original_alias_name_map.end()) {
      // User provided the original port name for setting the default value, we update the alias port
      const std::string aliased_port_name = original_alias_name_map.at(port_name);
      ports_list.at(aliased_port_name).setDefaultValue(new_default);
    } else {
      for (const auto & [original_port_name, aliased_port_name] : original_alias_name_map) {
        if (aliased_port_name == port_name) {
          // User provided the aliased port name for setting the default value, we update the original port
          ports_list.at(original_port_name).setDefaultValue(new_default);
          break;
        }
      }
    }
  }
}

void RosNodeContext::copyAliasedPortValuesToOriginalPorts(const BT::TreeNode * node) const
{
  // The node owns its NodeConfig; config() only hands back a const reference. We copy the aliased port values directly
  // onto the original ports here rather than going through BT::TreeNode::modifyPortsRemapping, because that only
  // *updates* ports already present in the config. The original port is typically absent from the config when it has
  // no default value and the user only ever sets the aliased port, so it must be *inserted* - otherwise the node reads
  // an unset original port (e.g. an empty list for a std::vector<std::string> port, or an outright getInput() failure).
  BT::NodeConfig & config = const_cast<BT::NodeConfig &>(node->config());

  // Iterate the configured aliases (original -> aliased) rather than the node's populated ports: the value the user
  // provides lives on the *aliased* port and must be copied onto the *original* port the node implementation reads via
  // getInput()/getOutput().
  for (const auto & [original_port_name, aliased_port_spec] : registration_options_.port_alias) {
    const auto [aliased_port_name, _] = parseAliasPortName(aliased_port_spec);

    // The aliased port may be an input or an output port; copy from whichever map holds its value. If the aliased port
    // carries no value at all (neither set in the XML nor via a default), there is nothing to copy and the original
    // port is left unset - exactly as if the port had not been provided.
    if (const auto it = config.input_ports.find(aliased_port_name); it != config.input_ports.end()) {
      config.input_ports[original_port_name] = it->second;
    } else if (const auto out_it = config.output_ports.find(aliased_port_name); out_it != config.output_ports.end()) {
      config.output_ports[original_port_name] = out_it->second;
    }
  }
}

}  // namespace auto_apms_behavior_tree::core