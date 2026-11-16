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

#include <type_traits>

#include "auto_apms_behavior_tree/util/parameter.hpp"
#include "auto_apms_behavior_tree_core/node.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

#define INPUT_KEY_PARAM_NAME "parameter"
#define INPUT_KEY_PARAM_VALUE "value"
#define INPUT_KEY_NODE_NAME "node"

namespace auto_apms_behavior_tree
{

template <typename T>
class SetParameterTemplate : public core::RosServiceNode<rcl_interfaces::srv::SetParameters>
{
public:
  SetParameterTemplate(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
    // A node manifest may name the service instead, with (input:port) substitutions (registration option 'topic', e.g.
    // "(input:robot)/my_node/set_parameters" to target a node of the robot named by the input port 'robot'). Then the
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
    createClient(node_name + "/set_parameters");
  }

  static BT::PortsList providedPorts()
  {
    // We do not use the default port for the service name

    // There is no string conversion function for variables that are type initialized using the value port if the
    // BT::Any version is used. To prevent errors when using these variables in e.g. the scripting language we have to
    // set the type to BT::AnyTypeAllowed to truly indicate that the type is not set by this port
    using AnyType = typename std::conditional_t<std::is_same_v<BT::Any, T>, BT::AnyTypeAllowed, T>;
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_NODE_NAME,
        "Name of the targeted ROS 2 node (a blackboard entry must be set before the tree is created). Leave empty to "
        "target this executor's node."),
      BT::InputPort<AnyType>(INPUT_KEY_PARAM_VALUE, "Value of the parameter to be set."),
      BT::InputPort<std::string>(INPUT_KEY_PARAM_NAME, "Name of the parameter to be set."),
    };
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    rcl_interfaces::msg::Parameter parameter;
    const BT::Expected<std::string> expected_name = getInput<std::string>(INPUT_KEY_PARAM_NAME);
    if (!expected_name || expected_name.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - Parameter name must not be empty.", context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_name, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_name.error().c_str());
      return false;
    }
    parameter.name = expected_name.value();

    rclcpp::ParameterType inferred_param_type = rclcpp::PARAMETER_NOT_SET;
    if constexpr (!std::is_same_v<BT::Any, T>) {
      inferred_param_type = rclcpp::ParameterValue(T()).get_type();
    }

    rcl_interfaces::msg::ParameterValue param_val;
    const BT::PortsRemapping::iterator it = config().input_ports.find(INPUT_KEY_PARAM_VALUE);
    if (it == config().input_ports.end() || it->second.empty()) {
      if constexpr (std::is_same_v<BT::Any, T>) {
        throw exceptions::RosNodeError(
          context_.getFullyQualifiedTreeNodeName(this) + " - Parameter value must not be empty.");
      } else {
        // Use default value of rcl_interfaces::msg::Parameter if input is not specified (Don't set param_val.value)
        param_val.type = inferred_param_type;
      }
    } else {
      // If input port is a literal, the user must use one of the statically typed versions of SetParameter, since
      // it's impossible to know what type the parameter to be set should have
      if (!isBlackboardPointer(it->second) && std::is_same_v<BT::Any, T>) {
        throw exceptions::RosNodeError(
          context_.getFullyQualifiedTreeNodeName(this) +
          " - Cannot infer type from literal string. Use one of the type specialized versions of SetParameter "
          "instead.");
      }

      const BT::Expected<T> expected_entry = getInput<T>(INPUT_KEY_PARAM_VALUE);
      if (!expected_entry) {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_entry.error().c_str());
        return false;
      }
      const BT::Expected<rclcpp::ParameterValue> expected_param_val =
        createParameterValueFromAny(BT::Any(expected_entry.value()), inferred_param_type);
      if (!expected_param_val) {
        // Conversion might not be possible. In this case, log error message and reject to set parameter.
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_param_val.error().c_str());
        return false;
      }
      param_val = expected_param_val.value().to_value_msg();
    }

    parameter.value = param_val;
    requested_parameter_ = rclcpp::Parameter(parameter.name, parameter.value);
    request->parameters.push_back(parameter);
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    const rcl_interfaces::msg::SetParametersResult & result = response->results[0];
    if (!result.successful) {
      RCLCPP_ERROR(
        logger_, "Failed to set parameter %s = %s (Type: %s) via service '%s': %s",
        requested_parameter_.get_name().c_str(), requested_parameter_.value_to_string().c_str(),
        requested_parameter_.get_type_name().c_str(), getServiceName().c_str(), result.reason.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Parameter requested_parameter_;
};

// Automatically infer the parameter type from BT::Any
class SetParameter : public SetParameterTemplate<BT::Any>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterBool : public SetParameterTemplate<bool>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterInt : public SetParameterTemplate<int>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterDouble : public SetParameterTemplate<double>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterString : public SetParameterTemplate<std::string>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterByteVec : public SetParameterTemplate<std::vector<uint8_t>>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterBoolVec : public SetParameterTemplate<std::vector<bool>>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterIntVec : public SetParameterTemplate<std::vector<int>>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterDoubleVec : public SetParameterTemplate<std::vector<double>>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterStringVec : public SetParameterTemplate<std::vector<std::string>>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameter)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterBool)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterInt)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterDouble)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterString)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterByteVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterBoolVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterIntVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterDoubleVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterStringVec)