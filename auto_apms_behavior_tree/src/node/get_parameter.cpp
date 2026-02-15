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
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/parameter_value.hpp"

#define INPUT_KEY_PARAM_NAME "parameter"
#define OUTPUT_KEY_PARAM_VALUE "value"
#define INPUT_KEY_NODE_NAME "node"

namespace auto_apms_behavior_tree
{

template <typename T>
class GetParameterTemplate : public core::RosServiceNode<rcl_interfaces::srv::GetParameters>
{
public:
  GetParameterTemplate(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
    // If input port is empty or not provided, refer to this ROS 2 node as the target
    std::string node_name = context_.getFullyQualifiedRosNodeName();
    if (
      config.input_ports.find(INPUT_KEY_NODE_NAME) != config.input_ports.end() &&
      !config.input_ports.at(INPUT_KEY_NODE_NAME).empty()) {
      node_name = config.input_ports.at(INPUT_KEY_NODE_NAME);
    }
    createClient(node_name + "/get_parameters");
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
        INPUT_KEY_NODE_NAME, "Name of the targeted ROS 2 node. Leave empty to target this executor's node."),
      BT::OutputPort<AnyType>(OUTPUT_KEY_PARAM_VALUE, "Output port for the parameter's value."),
      BT::InputPort<std::string>(INPUT_KEY_PARAM_NAME, "Name of the parameter to get.")};
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
    requested_parameter_name_ = expected_name.value();
    request->names.push_back(requested_parameter_name_);
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (response->values.empty()) {
      RCLCPP_DEBUG(
        logger_, "%s - Parameter '%s' not found on the targeted node.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), requested_parameter_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    rclcpp::ParameterValue val(response->values[0]);
    if (val.get_type() == rclcpp::PARAMETER_NOT_SET) {
      RCLCPP_DEBUG(
        logger_, "%s - Tried to get undeclared parameter '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        requested_parameter_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    BT::Result set_ouput_result;
    if constexpr (std::is_same_v<BT::Any, T>) {
      // If ouput port type is BT::Any, we must try to infer the underlying type of the blackboard entry from received
      // the parameter message
      const BT::Expected<BT::Any> expected = createAnyFromParameterValue(val);
      if (!expected) {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return BT::NodeStatus::FAILURE;
      }
      set_ouput_result = setOutput(OUTPUT_KEY_PARAM_VALUE, expected.value());
    } else {
      set_ouput_result = setOutput(OUTPUT_KEY_PARAM_VALUE, val.get<T>());
    }

    if (!set_ouput_result) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), set_ouput_result.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::string requested_parameter_name_;
};

// This version allows to infer the blackboard entry type from the type of the parameter. Behavior trees trying to read
// the entry must know the current type and cast correspondingly.
class GetParameter : public GetParameterTemplate<BT::Any>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterBool : public GetParameterTemplate<bool>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterInt : public GetParameterTemplate<int>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterDouble : public GetParameterTemplate<double>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterString : public GetParameterTemplate<std::string>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterByteVec : public GetParameterTemplate<std::vector<uint8_t>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterBoolVec : public GetParameterTemplate<std::vector<bool>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterIntVec : public GetParameterTemplate<std::vector<int>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterDoubleVec : public GetParameterTemplate<std::vector<double>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterStringVec : public GetParameterTemplate<std::vector<std::string>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameter)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterBool)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterInt)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterDouble)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterString)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterByteVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterBoolVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterIntVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterDoubleVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterStringVec)