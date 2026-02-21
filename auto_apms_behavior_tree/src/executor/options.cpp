// Copyright 2026 Robin Müller
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

#include "auto_apms_behavior_tree/executor/options.hpp"

namespace auto_apms_behavior_tree
{

// --- TreeExecutorNodeOptions ---

TreeExecutorNodeOptions::TreeExecutorNodeOptions(const rclcpp::NodeOptions & ros_node_options)
: ros_node_options_(ros_node_options)
{
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::enableCommandAction(bool enable)
{
  enable_command_action_ = enable;
  return *this;
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::enableClearBlackboardService(bool enable)
{
  enable_clear_blackboard_service_ = enable;
  return *this;
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::enableScriptingEnumParameters(bool from_overrides, bool dynamic)
{
  scripting_enum_parameters_from_overrides_ = from_overrides;
  scripting_enum_parameters_dynamic_ = dynamic;
  return *this;
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::enableGlobalBlackboardParameters(bool from_overrides, bool dynamic)
{
  blackboard_parameters_from_overrides_ = from_overrides;
  blackboard_parameters_dynamic_ = dynamic;
  return *this;
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::setDefaultBuildHandler(const std::string & name)
{
  custom_default_parameters_[_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER] = rclcpp::ParameterValue(name);
  return *this;
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::setCommandActionName(const std::string & name)
{
  command_action_name_ = name;
  return *this;
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::setClearBlackboardServiceName(const std::string & name)
{
  clear_blackboard_service_name_ = name;
  return *this;
}

rclcpp::NodeOptions TreeExecutorNodeOptions::getROSNodeOptions() const
{
  rclcpp::NodeOptions opt(ros_node_options_);
  opt.automatically_declare_parameters_from_overrides(
    scripting_enum_parameters_from_overrides_ || blackboard_parameters_from_overrides_);
  opt.allow_undeclared_parameters(scripting_enum_parameters_dynamic_ || blackboard_parameters_dynamic_);

  // Default configuration
  opt.enable_logger_service(true);

  return opt;
}

}  // namespace auto_apms_behavior_tree