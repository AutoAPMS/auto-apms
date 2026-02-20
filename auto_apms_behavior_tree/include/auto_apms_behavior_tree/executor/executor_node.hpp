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

#pragma once

#include "auto_apms_behavior_tree/executor/action_based_executor.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Configuration options for TreeExecutorNode.
 *
 * This allows to hardcode certain configurations. During initialization, a TreeExecutorNode parses the provided options
 * and activates/deactivates the corresponding features. Inherits from GenericEventBasedTreeExecutorOptions and adds
 * the StartTreeExecutor action configuration.
 */
class TreeExecutorNodeOptions : public GenericEventBasedTreeExecutorOptions
{
public:
  /**
   * @brief Constructor.
   *
   * Executor options must be created by passing an existing `rclcpp::NodeOptions` object.
   * @param ros_node_options ROS 2 node options.
   */
  TreeExecutorNodeOptions(const rclcpp::NodeOptions & ros_node_options);

  /**
   * @brief Configure whether the executor node accepts scripting enum parameters.
   * @param from_overrides `true` allows to set scripting enums from parameter overrides, `false` forbids that.
   * @param dynamic `true` allows to dynamically set scripting enums at runtime, `false` forbids that.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableScriptingEnumParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Configure whether the executor node accepts global blackboard parameters.
   * @param from_overrides `true` allows to set global blackboard entries from parameter overrides, `false` forbids
   * that.
   * @param dynamic `true` allows to dynamically set global blackboard entries at runtime, `false` forbids that.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableGlobalBlackboardParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Specify a default behavior tree build handler that will be used initially.
   * @param name Fully qualified class name of the behavior tree build handler plugin.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & setDefaultBuildHandler(const std::string & name);

  /**
   * @brief Set a custom name for the start tree executor action server.
   *
   * If not set, the default name `<node_name>/start` is used.
   * @param name Full action name.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & setStartActionName(const std::string & name);

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

private:
  friend class TreeExecutorNode;

  std::string start_action_name_;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Flexible ROS 2 node implementing a standardized interface for dynamically executing behavior trees.
 *
 * This class extends GenericEventBasedTreeExecutor with a StartTreeExecutor action server that allows external
 * clients to trigger behavior tree execution via an action goal. The executor is configured using ROS 2 parameters.
 *
 * A behavior tree can be executed via command line:
 *
 * ```sh
 * ros2 run auto_apms_behavior_tree run_behavior <build_request>
 * ```
 *
 * Alternatively, an executor can also be included as part of a ROS 2 components container. The following executor
 * components are provided:
 *
 * - `%auto_apms_behavior_tree::TreeExecutorNode`
 *
 * - `auto_apms_behavior_tree::NoUndeclaredParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyScriptingEnumParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyBlackboardParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyInitialScriptingEnumParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyInitialBlackboardParamsExecutorNode`
 */
class TreeExecutorNode : public ActionBasedTreeExecutor<auto_apms_interfaces::action::StartTreeExecutor>
{
public:
  using Options = TreeExecutorNodeOptions;

  /**
   * @brief Constructor allowing to specify a custom node name and executor options.
   * @param name Default name of the `rclcpp::Node`.
   * @param executor_options Executor specific options. Simply pass a `rclcpp::NodeOptions` object to use the default
   * options.
   */
  TreeExecutorNode(const std::string & name, TreeExecutorNodeOptions executor_options);

  /**
   * @brief Constructor populating both the node's name and the executor options with the default.
   * @param options Options forwarded to rclcpp::Node constructor.
   */
  explicit TreeExecutorNode(rclcpp::NodeOptions options);

  virtual ~TreeExecutorNode() override = default;

protected:
  /* ActionBasedTreeExecutor overrides */

  /**
   * @brief Create a TreeConstructor from a StartTreeExecutor action goal.
   *
   * Loads the build handler (if specified), parses the node manifest, and creates a tree constructor using the
   * build request from the goal.
   * @param goal_ptr Shared pointer to the StartTreeExecutor action goal.
   * @return Callback for creating the behavior tree.
   * @throw std::exception if the goal cannot be processed.
   */
  TreeConstructor getTreeConstructorFromGoal(std::shared_ptr<const TriggerGoal> goal_ptr) override;

  /**
   * @brief Determine whether an incoming start action goal should be accepted.
   *
   * The default implementation rejects the goal if the executor is currently busy executing a behavior tree. Derived
   * classes may override this to add additional validation logic.
   * @param uuid The unique identifier of the incoming goal.
   * @param goal_ptr Shared pointer to the incoming goal.
   * @return `true` if the goal should be accepted, `false` if it should be rejected.
   */
  bool shouldAcceptGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGoal> goal_ptr) override;

  /**
   * @brief Hook called after a start action goal has been accepted and before execution begins.
   *
   * Clears the global blackboard if the goal's `clear_blackboard` flag is set.
   * @param goal_handle_ptr Shared pointer to the accepted goal handle.
   */
  void onAcceptedGoal(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr) override;

  /**
   * @brief Hook called after execution has been started successfully.
   *
   * Handles attached vs detached mode: in attached mode, sets up the action context to track execution;
   * in detached mode, immediately succeeds the goal.
   * @param goal_handle_ptr Shared pointer to the accepted goal handle.
   */
  void onExecutionStarted(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr) override;

  /**
   * @brief Handle the execution result for the StartTreeExecutor action client.
   *
   * Populates the result with tree status information and the terminated tree identity.
   * @param result The execution result.
   * @param context The action context for sending the result back.
   */
  void onGoalExecutionTermination(const ExecutionResult & result, TriggerActionContext & context) override;

  bool afterTick() override;

private:
  const TreeExecutorNodeOptions node_options_;
};

}  // namespace auto_apms_behavior_tree