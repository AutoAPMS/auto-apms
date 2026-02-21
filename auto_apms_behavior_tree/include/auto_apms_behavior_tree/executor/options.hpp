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

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Configuration options for GenericEventBasedTreeExecutor.
 *
 * This allows configuring which features of the behavior tree executor are enabled, such as the command action
 * interface, parameter/blackboard synchronization, and scripting enum support.
 */
class TreeExecutorNodeOptions
{
public:
  /**
   * @brief Constructor.
   * @param ros_node_options ROS 2 node options.
   */
  TreeExecutorNodeOptions(const rclcpp::NodeOptions & ros_node_options = rclcpp::NodeOptions());

  /**
   * @brief Enable or disable the CommandTreeExecutor action interface.
   * @param enable `true` to enable, `false` to disable.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableCommandAction(bool enable);

  /**
   * @brief Enable or disable the clear blackboard service.
   * @param enable `true` to enable, `false` to disable.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableClearBlackboardService(bool enable);

  /**
   * @brief Configure whether the executor accepts scripting enum parameters.
   * @param from_overrides `true` allows to set scripting enums from parameter overrides.
   * @param dynamic `true` allows to dynamically set scripting enums at runtime.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableScriptingEnumParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Configure whether the executor accepts global blackboard parameters.
   * @param from_overrides `true` allows to set global blackboard entries from parameter overrides.
   * @param dynamic `true` allows to dynamically set global blackboard entries at runtime.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableGlobalBlackboardParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Specify a default behavior tree build handler.
   * @param name Fully qualified class name of the behavior tree build handler plugin.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & setDefaultBuildHandler(const std::string & name);

  /**
   * @brief Set a custom name for the command tree executor action server.
   *
   * If not set, the default name `<node_name>/cmd` is used.
   * @param name Full action name.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & setCommandActionName(const std::string & name);

  /**
   * @brief Set a custom name for the clear blackboard service.
   *
   * If not set, the default name `<node_name>/clear_blackboard` is used.
   * @param name Full service name.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & setClearBlackboardServiceName(const std::string & name);

  /**
   * @brief Get the ROS 2 node options that comply with the given options.
   * @return Corresponding `rclcpp::NodeOptions` object.
   */
  rclcpp::NodeOptions getROSNodeOptions() const;

private:
  friend class GenericEventBasedTreeExecutor;

  rclcpp::NodeOptions ros_node_options_;
  bool enable_command_action_ = true;
  bool enable_clear_blackboard_service_ = true;
  bool scripting_enum_parameters_from_overrides_ = true;
  bool scripting_enum_parameters_dynamic_ = true;
  bool blackboard_parameters_from_overrides_ = true;
  bool blackboard_parameters_dynamic_ = true;
  std::map<std::string, rclcpp::ParameterValue> custom_default_parameters_;
  std::string command_action_name_;
  std::string clear_blackboard_service_name_;
};

}  // namespace auto_apms_behavior_tree