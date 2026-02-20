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

#include "auto_apms_behavior_tree/build_handler/build_handler_loader.hpp"
#include "auto_apms_behavior_tree/executor/executor_base.hpp"
#include "auto_apms_behavior_tree/executor_params.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_interfaces/action/command_tree_executor.hpp"
#include "auto_apms_util/action_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Configuration options for GenericEventBasedTreeExecutor.
 *
 * This allows configuring which features of the event-based executor are enabled, such as the command action
 * interface, parameter/blackboard synchronization, and scripting enum support.
 */
class GenericEventBasedTreeExecutorOptions
{
public:
  /**
   * @brief Constructor.
   * @param ros_node_options ROS 2 node options.
   */
  GenericEventBasedTreeExecutorOptions(const rclcpp::NodeOptions & ros_node_options = rclcpp::NodeOptions());

  /**
   * @brief Enable or disable the CommandTreeExecutor action interface.
   * @param enable `true` to enable, `false` to disable.
   * @return Modified options object.
   */
  GenericEventBasedTreeExecutorOptions & enableCommandAction(bool enable);

  /**
   * @brief Enable or disable the clear blackboard service.
   * @param enable `true` to enable, `false` to disable.
   * @return Modified options object.
   */
  GenericEventBasedTreeExecutorOptions & enableClearBlackboardService(bool enable);

  /**
   * @brief Configure whether the executor accepts scripting enum parameters.
   * @param from_overrides `true` allows to set scripting enums from parameter overrides.
   * @param dynamic `true` allows to dynamically set scripting enums at runtime.
   * @return Modified options object.
   */
  GenericEventBasedTreeExecutorOptions & enableScriptingEnumParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Configure whether the executor accepts global blackboard parameters.
   * @param from_overrides `true` allows to set global blackboard entries from parameter overrides.
   * @param dynamic `true` allows to dynamically set global blackboard entries at runtime.
   * @return Modified options object.
   */
  GenericEventBasedTreeExecutorOptions & enableGlobalBlackboardParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Specify a default behavior tree build handler.
   * @param name Fully qualified class name of the behavior tree build handler plugin.
   * @return Modified options object.
   */
  GenericEventBasedTreeExecutorOptions & setDefaultBuildHandler(const std::string & name);

  /**
   * @brief Set a custom name for the command tree executor action server.
   * @param name Full action name.
   * @return Modified options object.
   */
  GenericEventBasedTreeExecutorOptions & setCommandActionName(const std::string & name);

  /**
   * @brief Set a custom name for the clear blackboard service.
   * @param name Full service name.
   * @return Modified options object.
   */
  GenericEventBasedTreeExecutorOptions & setClearBlackboardServiceName(const std::string & name);

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
  bool scripting_enum_parameters_from_overrides_ = false;
  bool scripting_enum_parameters_dynamic_ = false;
  bool blackboard_parameters_from_overrides_ = false;
  bool blackboard_parameters_dynamic_ = false;
  std::map<std::string, rclcpp::ParameterValue> custom_default_parameters_;
  std::string command_action_name_;
  std::string clear_blackboard_service_name_;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Event-based behavior tree executor that starts execution when a trigger condition is met.
 *
 * This executor extends TreeExecutorBase with configurable support for:
 * - Build handler management (loading and switching build handlers)
 * - CommandTreeExecutor action interface (pause, resume, halt, terminate)
 * - Parameter/blackboard synchronization
 * - Scripting enum parameters
 *
 * Derived classes can start behavior tree execution by calling `startExecution()` with a build request
 * or a TreeConstructor directly.
 */
class GenericEventBasedTreeExecutor : public TreeExecutorBase
{
public:
  using Options = GenericEventBasedTreeExecutorOptions;
  using ExecutorParameters = executor_params::Params;
  using ExecutorParameterListener = executor_params::ParamListener;
  using CommandActionContext = auto_apms_util::ActionContext<auto_apms_interfaces::action::CommandTreeExecutor>;

  inline static const std::string SCRIPTING_ENUM_PARAM_PREFIX = "enum";
  inline static const std::string BLACKBOARD_PARAM_PREFIX = "bb";

  /// Value indicating that no build handler is loaded.
  inline static const std::string PARAM_VALUE_NO_BUILD_HANDLER = "none";

  /**
   * @brief Constructor.
   * @param name Name of the `rclcpp::Node`.
   * @param options Event-based executor options.
   */
  GenericEventBasedTreeExecutor(const std::string & name, GenericEventBasedTreeExecutorOptions options);

  /**
   * @brief Constructor with default options (everything disabled except build handler management).
   * @param options ROS 2 node options.
   */
  explicit GenericEventBasedTreeExecutor(rclcpp::NodeOptions options);

  virtual ~GenericEventBasedTreeExecutor() override = default;

  using TreeExecutorBase::startExecution;

  /**
   * @brief Start the behavior tree specified by a particular build request.
   * @param build_request Behavior build request for creating the behavior.
   * @param entry_point Single point of entry for behavior execution.
   * @param node_manifest Behavior tree node manifest to be loaded for behavior execution.
   * @return Shared future that completes once executing the tree is finished or an error occurs.
   */
  std::shared_future<ExecutionResult> startExecution(
    const std::string & build_request, const std::string & entry_point = "",
    const core::NodeManifest & node_manifest = {});

private:
  /* Virtual methods to be overridden by derived classes */

  /**
   * @brief Callback invoked before building the behavior tree.
   * @param builder Tree builder to be configured.
   * @param build_request Behavior build request.
   * @param entry_point Single point of entry for behavior execution.
   * @param node_manifest Behavior tree node manifest.
   * @param bb Local blackboard of the tree being created.
   */
  virtual void preBuild(
    core::TreeBuilder & builder, const std::string & build_request, const std::string & entry_point,
    const core::NodeManifest & node_manifest, TreeBlackboard & bb);

  /**
   * @brief Callback invoked after the behavior tree has been instantiated.
   * @param tree Behavior tree that has been created and is about to be executed.
   */
  virtual void postBuild(Tree & tree);

protected:
  /* Utility methods */

  /**
   * @brief Get a copy of the current executor parameters.
   * @return Current executor parameters.
   */
  ExecutorParameters getExecutorParameters() const;

  /**
   * @brief Assemble all parameters of this node that have a specific prefix.
   * @param prefix Only consider parameters that have this prefix in their names.
   * @return Map of parameter names and their respective values.
   */
  std::map<std::string, rclcpp::ParameterValue> getParameterValuesWithPrefix(const std::string & prefix);

  /**
   * @brief Get the name of a parameter without its prefix.
   * @param prefix Prefix to remove from @p param_name.
   * @param param_name Name of the parameter with its prefix.
   * @return Name of the parameter without its prefix.
   */
  static std::string stripPrefixFromParameterName(const std::string & prefix, const std::string & param_name);

  /**
   * @brief Update the internal buffer of scripting enums.
   * @param value_map Map of parameter names and their respective values.
   * @param simulate Set to `true` to only validate.
   * @return `true` if updating is possible.
   */
  bool updateScriptingEnumsWithParameterValues(
    const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate = false);

  /**
   * @brief Update the global blackboard using parameter values.
   * @param value_map Map of parameter names and their respective values.
   * @param simulate Set to `true` to only validate.
   * @return `true` if updating is possible.
   */
  bool updateGlobalBlackboardWithParameterValues(
    const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate = false);

  /**
   * @brief Load a particular behavior tree build handler plugin.
   * @param name Fully qualified name of the build handler class. Set to `"none"` to unload.
   */
  void loadBuildHandler(const std::string & name);

  /**
   * @brief Create a callback that builds a behavior tree according to a specific request.
   * @param build_request Request that specifies how to build the behavior tree.
   * @param entry_point Single point of entry for behavior execution.
   * @param node_manifest Behavior tree node manifest.
   * @return Callback for creating the behavior tree.
   */
  TreeConstructor makeTreeConstructor(
    const std::string & build_request, const std::string & entry_point = "",
    const core::NodeManifest & node_manifest = {});

  /**
   * @brief Reset the global blackboard and clear all entries.
   * @return `true` if blackboard was cleared, `false` if executor is not idle.
   */
  virtual bool clearGlobalBlackboard() override;

  bool onTick() override;

  bool afterTick() override;

private:
  /* Internal callbacks */

  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback_(
    const std::vector<rclcpp::Parameter> & parameters);

  void parameter_event_callback_(const rcl_interfaces::msg::ParameterEvent & event);

  rclcpp_action::GoalResponse handle_command_goal_(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CommandActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_command_cancel_(
    std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);
  void handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);

protected:
  const GenericEventBasedTreeExecutorOptions executor_options_;
  ExecutorParameterListener executor_param_listener_;

  core::NodeRegistrationLoader::SharedPtr tree_node_loader_ptr_;
  TreeBuildHandlerLoader::UniquePtr build_handler_loader_ptr_;
  core::TreeBuilder::UniquePtr builder_ptr_;
  TreeBuildHandler::UniquePtr build_handler_ptr_;
  std::string current_build_handler_name_;
  std::map<std::string, int> scripting_enums_;
  std::map<std::string, rclcpp::ParameterValue> translated_global_blackboard_entries_;

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_ptr_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_ptr_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr parameter_event_callback_handle_ptr_;

  // Command action interface (optional)
  rclcpp_action::Server<CommandActionContext::Type>::SharedPtr command_action_ptr_;
  rclcpp::TimerBase::SharedPtr command_timer_ptr_;

  // Clear blackboard service (optional)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_blackboard_service_ptr_;
};

}  // namespace auto_apms_behavior_tree