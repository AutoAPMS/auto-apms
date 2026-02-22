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

#include "auto_apms_behavior_tree/executor/executor_node.hpp"

#include <functional>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"
#include "rclcpp/utilities.hpp"

namespace auto_apms_behavior_tree
{

TreeExecutorNode::TreeExecutorNode(const std::string & name, const std::string & start_action_name, Options options)
: ActionBasedTreeExecutor<auto_apms_interfaces::action::StartTreeExecutor>(
    name,
    start_action_name.empty() ? name + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX : start_action_name,
    options)
{
  // Make sure ROS arguments are removed. When using rclcpp_components, this is typically not the case.
  std::vector<std::string> args_with_ros_arguments = node_ptr_->get_node_options().arguments();
  int argc = args_with_ros_arguments.size();
  char ** argv = new char *[argc + 1];  // +1 for the null terminator
  for (int i = 0; i < argc; ++i) {
    argv[i] = const_cast<char *>(args_with_ros_arguments[i].c_str());
  }
  argv[argc] = nullptr;  // Null-terminate the array as required for argv[]

  // Evaluate possible cli argument dictating to start executing with a specific build request immediately.
  // Note: First argument is always path of executable.
  if (const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv); args.size() > 1) {
    // Log relevant arguments. First argument is executable name (argv[0]) and won't be considered.
    std::vector<std::string> relevant_args{args.begin() + 1, args.end()};
    RCLCPP_DEBUG(
      logger_, "Additional cli arguments in rclcpp::NodeOptions: [ %s ]",
      auto_apms_util::join(relevant_args, ", ").c_str());

    const ExecutorParameters initial_params = executor_param_listener_.get_params();
    // Start tree execution with the build handler request being the first relevant argument
    startExecution(makeTreeConstructor(relevant_args[0]), initial_params.tick_rate, initial_params.groot2_port);
  }
}

TreeExecutorNode::TreeExecutorNode(const std::string & name, Options options) : TreeExecutorNode(name, "", options) {}

TreeExecutorNode::TreeExecutorNode(rclcpp::NodeOptions ros_options)
: TreeExecutorNode(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME, Options(ros_options))
{
}

bool TreeExecutorNode::shouldAcceptGoal(
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

TreeConstructor TreeExecutorNode::getTreeConstructorFromGoal(std::shared_ptr<const TriggerGoal> goal_ptr)
{
  if (!goal_ptr->build_handler.empty()) {
    if (executor_param_listener_.get_params().allow_other_build_handlers) {
      loadBuildHandler(goal_ptr->build_handler);
    } else if (goal_ptr->build_handler != current_build_handler_name_) {
      throw exceptions::TreeExecutorError(
        "Current tree build handler '" + current_build_handler_name_ +
        "' must not change since the 'Allow other build handlers' option is disabled.");
    }
  }

  core::NodeManifest node_manifest = core::NodeManifest::decode(goal_ptr->node_manifest);
  return makeTreeConstructor(goal_ptr->build_request, goal_ptr->entry_point, node_manifest);
}

void TreeExecutorNode::onAcceptedGoal(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr)
{
  // Clear blackboard parameters if desired
  if (goal_handle_ptr->get_goal()->clear_blackboard) {
    clearGlobalBlackboard();
  }
}

void TreeExecutorNode::onExecutionStarted(std::shared_ptr<TriggerGoalHandle> goal_handle_ptr)
{
  const std::string started_tree_name = getTreeName();

  // If attach is true, the goal's life time is synchronized with the execution. Otherwise we succeed immediately and
  // leave the executor running (Detached mode).
  if (goal_handle_ptr->get_goal()->attach) {
    trigger_action_context_.setUp(goal_handle_ptr);
    RCLCPP_INFO(logger_, "Successfully started execution of tree '%s' (Mode: Attached).", started_tree_name.c_str());
  } else {
    auto result_ptr = std::make_shared<TriggerResult>();
    result_ptr->message = "Successfully started execution of tree '" + started_tree_name + "' (Mode: Detached).";
    result_ptr->tree_result = TriggerResult::TREE_RESULT_NOT_SET;
    result_ptr->terminated_tree_identity = started_tree_name;
    goal_handle_ptr->succeed(result_ptr);
    RCLCPP_INFO_STREAM(logger_, result_ptr->message);
  }
}

bool TreeExecutorNode::afterTick()
{
  // Call parent's afterTick for blackboard synchronization etc.
  if (!GenericEventBasedTreeExecutor::afterTick()) return false;

  // Send feedback (only in attached mode)
  if (trigger_action_context_.isValid()) {
    TreeStateObserver & state_observer = getStateObserver();
    auto feedback_ptr = trigger_action_context_.getFeedbackPtr();
    feedback_ptr->execution_state_str = toStr(getExecutionState());
    feedback_ptr->running_tree_identity = getTreeName();
    auto running_action_history = state_observer.getRunningActionHistory();
    if (!running_action_history.empty()) {
      // If there are multiple nodes running (ParallelNode), join the IDs to a single string
      feedback_ptr->running_action_name = auto_apms_util::join(running_action_history, " + ");
      feedback_ptr->running_action_timestamp =
        std::chrono::duration<double>{std::chrono::high_resolution_clock::now().time_since_epoch()}.count();

      // Reset the history cache
      state_observer.flush();
    }
    trigger_action_context_.publishFeedback();
  }

  return true;
}

void TreeExecutorNode::onGoalExecutionTermination(const ExecutionResult & result, TriggerActionContext & context)
{
  auto result_ptr = context.getResultPtr();
  result_ptr->terminated_tree_identity = getTreeName();
  switch (result) {
    case ExecutionResult::TREE_SUCCEEDED:
      result_ptr->tree_result = TriggerResult::TREE_RESULT_SUCCESS;
      result_ptr->message = "Tree execution finished with status SUCCESS";
      context.succeed();
      break;
    case ExecutionResult::TREE_FAILED:
      result_ptr->tree_result = TriggerResult::TREE_RESULT_FAILURE;
      result_ptr->message = "Tree execution finished with status FAILURE";
      context.abort();
      break;
    case ExecutionResult::TERMINATED_PREMATURELY:
      result_ptr->tree_result = TriggerResult::TREE_RESULT_NOT_SET;
      if (context.getGoalHandlePtr()->is_canceling()) {
        result_ptr->message = "Tree execution canceled successfully";
        context.cancel();
      } else {
        result_ptr->message = "Tree execution terminated prematurely";
        context.abort();
      }
      break;
    case ExecutionResult::ERROR:
      result_ptr->tree_result = TriggerResult::TREE_RESULT_NOT_SET;
      result_ptr->message = "An unexpected error occurred during tree execution";
      context.abort();
      break;
    default:
      result_ptr->tree_result = TriggerResult::TREE_RESULT_NOT_SET;
      result_ptr->message = "Execution result unkown";
      context.abort();
      break;
  }
}

}  // namespace auto_apms_behavior_tree