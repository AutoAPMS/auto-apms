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

#include "auto_apms_behavior_tree/executor/generic_event_based_executor.hpp"
#include "auto_apms_util/action_context.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Template class for executing behavior trees triggered by a custom ROS 2 action goal.
 *
 * This class extends GenericEventBasedTreeExecutor to trigger behavior tree execution when an action goal of the
 * specified type @p ActionT is received. It creates an action server and manages the lifecycle of the action goal
 * in coordination with the behavior tree execution.
 *
 * Derived classes must implement:
 * - `getTreeConstructorFromGoal()`: Create a TreeConstructor from the received action goal.
 *
 * Derived classes may optionally override:
 * - `shouldAcceptGoal()`: Custom goal acceptance logic.
 * - `onAcceptedGoal()`: Hook called after a goal is accepted.
 * - `onExecutionStarted()`: Hook called after execution has started successfully.
 * - `onGoalExecutionTermination()`: Handle the result of the execution for the action client.
 *
 * @tparam ActionT The ROS 2 action type that triggers the behavior tree execution.
 */
template <typename ActionT>
class ActionBasedTreeExecutor : public GenericEventBasedTreeExecutor
{
public:
  using TriggerActionContext = auto_apms_util::ActionContext<ActionT>;
  using TriggerGoal = typename ActionT::Goal;
  using TriggerFeedback = typename ActionT::Feedback;
  using TriggerResult = typename ActionT::Result;
  using TriggerGoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  /**
   * @brief Constructor.
   * @param name Name of the `rclcpp::Node`.
   * @param action_name Name for the trigger action server.
   * @param options Executor options.
   */
  ActionBasedTreeExecutor(const std::string & name, const std::string & action_name, Options options);

  /**
   * @brief Constructor with default options.
   * @param name Name of the `rclcpp::Node`.
   * @param action_name Name for the trigger action server.
   * @param ros_options ROS 2 node options.
   */
  ActionBasedTreeExecutor(
    const std::string & name, const std::string & action_name, rclcpp::NodeOptions ros_options = rclcpp::NodeOptions());

  virtual ~ActionBasedTreeExecutor() override = default;

protected:
  /* Virtual methods to implement/override */

  /**
   * @brief Create a TreeConstructor from the received action goal.
   *
   * Derived classes must implement this to define how the action goal maps to a behavior tree execution.
   * This is where the build request, entry point, and node manifest should be determined from the goal.
   * @param goal_ptr Shared pointer to the action goal.
   * @return Callback for creating the behavior tree.
   * @throw std::exception if the goal cannot be processed (will cause goal rejection).
   */
  virtual TreeConstructor getTreeConstructorFromGoal(std::shared_ptr<const TriggerGoal> goal_ptr) = 0;

  /**
   * @brief Determine whether an incoming trigger action goal should be accepted.
   *
   * The default implementation rejects the goal if the executor is currently busy.
   * @param uuid The unique identifier of the incoming goal.
   * @param goal_ptr Shared pointer to the incoming goal.
   * @return `true` if the goal should be accepted, `false` if it should be rejected.
   */
  virtual bool shouldAcceptGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGoal> goal_ptr);

  /**
   * @brief Hook called after a trigger action goal has been accepted and before execution begins.
   * @param goal_handle_ptr Shared pointer to the accepted goal handle.
   */
  virtual void onAcceptedGoal(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr);

  /**
   * @brief Hook called after execution has been started successfully.
   *
   * The default implementation sets up the action context for tracking the execution result (attached mode).
   * Derived classes may override this to implement detached behavior by succeeding the goal handle immediately.
   * @param goal_handle_ptr Shared pointer to the accepted goal handle.
   */
  virtual void onExecutionStarted(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr);

  /**
   * @brief Handle the execution result for the action client.
   *
   * The default implementation succeeds for TREE_SUCCEEDED, aborts for TREE_FAILED, handles cancellation
   * for TERMINATED_PREMATURELY, and aborts for ERROR.
   * @param result The execution result.
   * @param context The action context for sending the result back.
   */
  virtual void onGoalExecutionTermination(const ExecutionResult & result, TriggerActionContext & context);

  TriggerActionContext trigger_action_context_;
  TreeConstructor pending_tree_constructor_;

private:
  void onTermination(const ExecutionResult & result) override final;

  /* Internal action callbacks */

  rclcpp_action::GoalResponse handle_trigger_goal_(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGoal> goal_ptr);
  rclcpp_action::CancelResponse handle_trigger_cancel_(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr);
  void handle_trigger_accept_(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr);

  typename rclcpp_action::Server<ActionT>::SharedPtr trigger_action_ptr_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename ActionT>
ActionBasedTreeExecutor<ActionT>::ActionBasedTreeExecutor(
  const std::string & name, const std::string & action_name, Options options)
: GenericEventBasedTreeExecutor(name, options), trigger_action_context_(logger_)
{
  using namespace std::placeholders;
  trigger_action_ptr_ = rclcpp_action::create_server<ActionT>(
    node_ptr_, action_name, std::bind(&ActionBasedTreeExecutor::handle_trigger_goal_, this, _1, _2),
    std::bind(&ActionBasedTreeExecutor::handle_trigger_cancel_, this, _1),
    std::bind(&ActionBasedTreeExecutor::handle_trigger_accept_, this, _1));
}

template <typename ActionT>
ActionBasedTreeExecutor<ActionT>::ActionBasedTreeExecutor(
  const std::string & name, const std::string & action_name, rclcpp::NodeOptions ros_options)
: ActionBasedTreeExecutor(name, action_name, Options(ros_options))
{
}

template <typename ActionT>
bool ActionBasedTreeExecutor<ActionT>::shouldAcceptGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGoal> /*goal_ptr*/)
{
  if (isBusy()) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Tree '%s' is currently executing.", rclcpp_action::to_string(uuid).c_str(),
      getTreeName().c_str());
    return false;
  }
  return true;
}

template <typename ActionT>
void ActionBasedTreeExecutor<ActionT>::onAcceptedGoal(std::shared_ptr<TriggerGoalHandle> /*goal_handle_ptr*/)
{
}

template <typename ActionT>
void ActionBasedTreeExecutor<ActionT>::onExecutionStarted(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr)
{
  trigger_action_context_.setUp(goal_handle_ptr);
  RCLCPP_INFO(logger_, "Successfully started execution of tree '%s' via action trigger.", getTreeName().c_str());
}

template <typename ActionT>
void ActionBasedTreeExecutor<ActionT>::onGoalExecutionTermination(
  const ExecutionResult & result, TriggerActionContext & context)
{
  switch (result) {
    case ExecutionResult::TREE_SUCCEEDED:
      context.succeed();
      break;
    case ExecutionResult::TREE_FAILED:
      context.abort();
      break;
    case ExecutionResult::TERMINATED_PREMATURELY:
      if (context.getGoalHandlePtr()->is_canceling()) {
        context.cancel();
      } else {
        context.abort();
      }
      break;
    case ExecutionResult::ERROR:
    default:
      context.abort();
      break;
  }
}

template <typename ActionT>
void ActionBasedTreeExecutor<ActionT>::onTermination(const ExecutionResult & result)
{
  if (trigger_action_context_.isValid()) {
    onGoalExecutionTermination(result, trigger_action_context_);
    trigger_action_context_.invalidate();
  }
}

template <typename ActionT>
rclcpp_action::GoalResponse ActionBasedTreeExecutor<ActionT>::handle_trigger_goal_(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGoal> goal_ptr)
{
  if (!shouldAcceptGoal(uuid, goal_ptr)) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  try {
    pending_tree_constructor_ = getTreeConstructorFromGoal(goal_ptr);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Exception in getTreeConstructorFromGoal(): %s",
      rclcpp_action::to_string(uuid).c_str(), e.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ActionT>
rclcpp_action::CancelResponse ActionBasedTreeExecutor<ActionT>::handle_trigger_cancel_(
  std::shared_ptr<TriggerGoalHandle> /*goal_handle_ptr*/)
{
  setControlCommand(ControlCommand::TERMINATE);
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT>
void ActionBasedTreeExecutor<ActionT>::handle_trigger_accept_(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr)
{
  onAcceptedGoal(goal_handle_ptr);

  const ExecutorParameters params = executor_param_listener_.get_params();
  try {
    startExecution(pending_tree_constructor_, params.tick_rate, params.groot2_port);
  } catch (const std::exception & e) {
    auto result_ptr = std::make_shared<TriggerResult>();
    goal_handle_ptr->abort(result_ptr);
    RCLCPP_ERROR(logger_, "An error occurred trying to start execution: %s", e.what());
    return;
  }

  onExecutionStarted(goal_handle_ptr);
}

}  // namespace auto_apms_behavior_tree