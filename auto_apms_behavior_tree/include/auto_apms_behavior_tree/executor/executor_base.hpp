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

#include <chrono>
#include <functional>
#include <future>
#include <memory>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/executor/state_observer.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Base class that offers basic functionality for executing behavior trees.
 *
 * This class implements a highly configurable execution routine for behavior trees. Inheriting classes are able to
 * modify certain callbacks by overriding the corresponding virtual methods. The user is able to control the execution
 * by giving certain commands at runtime.
 *
 * @sa TreeExecutorNode for a standardized implementation as a `rclcpp::Node`.
 */
class TreeExecutorBase : public std::enable_shared_from_this<TreeExecutorBase>
{
public:
  /**
   * @brief Enum representing possible behavior tree execution states.
   */
  enum class ExecutionState : uint8_t
  {
    IDLE,      ///< Nothing to do
    STARTING,  ///< Initializing execution
    RUNNING,   ///< Executor is busy and tree has been ticked at least once
    PAUSED,    ///< Execution routine is active, but tree is not being ticked
    HALTED     ///< Execution routine is active, but tree is not being ticked and has been halted before
  };

  /**
   * @brief Enum representing possible commands for controlling the behavior tree execution routine.
   */
  enum class ControlCommand : uint8_t
  {
    RUN,       ///< Start/Resume the execution routine
    PAUSE,     ///< Pause the execution routine
    HALT,      ///< Halt the currently executing tree and pause the execution routine
    TERMINATE  ///< Halt the currently executing tree and terminate the execution routine
  };

  /**
   * @brief Enum representing possible options for what to do when a behavior tree is completed.
   */
  enum class TreeExitBehavior : uint8_t
  {
    TERMINATE,  ///< Stop execution and reset the timer
    RESTART     ///< Restart execution and keep on running
  };

  /**
   * @brief Enum representing possible behavior tree execution results.
   */
  enum class ExecutionResult : uint8_t
  {
    TREE_SUCCEEDED,          ///< Tree completed with `BT::NodeStatus::SUCCESS`.
    TREE_FAILED,             ///< Tree completed with `BT::NodeStatus::FAILURE`.
    TERMINATED_PREMATURELY,  ///< Execution terminated before the tree was able to propagate the tick to all its nodes
    ERROR                    ///< An unexpected error occurred
  };

private:
  using TerminationCallback = std::function<void(ExecutionResult, const std::string &)>;

public:
  /**
   * @brief Constructor.
   * @param node_ptr Parent ROS 2 node to be used to set up the required communication interfaces.
   * @param tree_node_callback_group_ptr Callback group that is used for all waitables registered by behavior tree
   * nodes.
   */
  TreeExecutorBase(
    rclcpp::Node::SharedPtr node_ptr, rclcpp::CallbackGroup::SharedPtr tree_node_callback_group_ptr = nullptr);

  virtual ~TreeExecutorBase() = default;

  /**
   * @brief Start a behavior tree that is built using a callback.
   *
   * Executing the behavior tree is achieved by regularly invoking the internal routine that ticks the behavior tree
   * created using @p make_tree. This requires to register a timer with the associated ROS 2 node. Consequently, the
   * behavior tree is executed asynchronously. The user is provided a shared future object that allows to check whether
   * the execution finished. Once this future completes, the execution result can be evaluated.
   * @param make_tree Callback that creates a `BT::Tree` object which will be ticked to execute the tree.
   * @param tick_rate_sec Behavior tree tick rate in seconds (1 tick every @p tick_rate_sec seconds) i.e. the interval
   * of the timer that regularly invokes the execution routine.
   * @param groot2_port Port number used for introspection and debugging with Groot2. `-1` means that no
   * `BT::Groot2Publisher` will be installed.
   * @return Shared future that completes once executing the tree is finished or an error occurs. In that case, it is
   * assigned an execution result code.
   */
  std::shared_future<ExecutionResult> startExecution(
    TreeConstructor make_tree, double tick_rate_sec = 0.1, int groot2_port = -1);

  /**
   * @brief Start a behavior tree that is built using a callback.
   *
   * Executing the behavior tree is achieved by regularly invoking the internal routine that ticks the behavior tree
   * created using @p make_tree. This requires to register a timer with the associated ROS 2 node. Consequently, the
   * behavior tree is executed asynchronously. The user is provided a shared future object that allows to check whether
   * the execution finished. Once this future completes, the execution result can be evaluated.
   * @param make_tree Callback that creates a `BT::Tree` object which will be ticked to execute the tree.
   * @param tick_rate Behavior tree tick rate i.e. the interval of the timer that regularly invokes the
   * execution routine.
   * @param groot2_port Port number used for introspection and debugging with Groot2. `-1` means that no
   * `BT::Groot2Publisher` will be installed.
   * @return Shared future that completes once executing the tree is finished or an error occurs. In that case, it is
   * assigned an execution result code.
   */
  template <typename TimeRepT = int64_t, typename TimeT = std::milli>
  std::shared_future<ExecutionResult> startExecution(
    TreeConstructor make_tree, const std::chrono::duration<TimeRepT, TimeT> & tick_rate, int groot2_port = -1);

  /**
   * @brief Start executing a behavior tree that has already been created.
   *
   * Unlike the TreeConstructor overloads, this variant lets the caller construct the `BT::Tree` themselves (e.g. via
   * GenericTreeExecutorNode::makeTreeConstructor) and hand over ownership. This decouples the potentially expensive
   * tree construction from starting the execution routine, so a caller can build the tree ahead of time and react to
   * build failures before committing to execution (e.g. abort registration if the tree cannot be built).
   *
   * @note For global blackboard entries (the `@`-prefixed keys) to resolve at runtime, @p tree must have been created
   * with a blackboard rooted at this executor's global blackboard (see getGlobalBlackboardPtr).
   * @param tree Ownership of the behavior tree to execute. Must not be `nullptr`.
   * @param tick_rate_sec Behavior tree tick rate in seconds (1 tick every @p tick_rate_sec seconds) i.e. the interval
   * of the timer that regularly invokes the execution routine.
   * @param groot2_port Port number used for introspection and debugging with Groot2. `-1` means that no
   * `BT::Groot2Publisher` will be installed.
   * @return Shared future that completes once executing the tree is finished or an error occurs.
   * @throw auto_apms_behavior_tree::exceptions::TreeExecutorError if the executor is busy or @p tree is `nullptr`.
   */
  std::shared_future<ExecutionResult> startExecution(
    std::unique_ptr<Tree> tree, double tick_rate_sec = 0.1, int groot2_port = -1);

  /**
   * @brief Start executing a behavior tree that has already been created.
   *
   * @sa startExecution(std::unique_ptr<Tree>, double, int) for details.
   * @param tree Ownership of the behavior tree to execute. Must not be `nullptr`.
   * @param tick_rate Behavior tree tick rate i.e. the interval of the timer that regularly invokes the execution
   * routine.
   * @param groot2_port Port number used for introspection and debugging with Groot2. `-1` means that no
   * `BT::Groot2Publisher` will be installed.
   * @return Shared future that completes once executing the tree is finished or an error occurs.
   */
  template <typename TimeRepT = int64_t, typename TimeT = std::milli>
  std::shared_future<ExecutionResult> startExecution(
    std::unique_ptr<Tree> tree, const std::chrono::duration<TimeRepT, TimeT> & tick_rate, int groot2_port = -1);

private:
  void tick_callback_(TerminationCallback termination_callback);

  /* Virtual methods */

  /**
   * @brief Callback invoked once **before** the behavior tree is ticked for the very first time.
   * @return `false` if the execution should be aborted, `true` to continue.
   */
  virtual bool onInitialTick();

  /**
   * @brief Callback invoked every time **before** the behavior tree is ticked.
   * @return `false` if the execution should be aborted, `true` to continue.
   */
  virtual bool onTick();

  /**
   * @brief Callback invoked every time **after** the behavior tree is ticked.
   * @return `false` if the execution should be aborted, `true` to continue.
   */
  virtual bool afterTick();

  /**
   * @brief Callback invoked last thing when the execution routine completes because the behavior tree is finished.
   *
   * This callback is only invoked, if there haven't been any errors during execution.
   * @return Whether to terminate or restart the routine.
   */
  virtual TreeExitBehavior onTreeExit(bool success);

  /**
   * @brief Callback invoked when the execution routine terminates.
   *
   * If the execution routine is restarted, this won't be called. It's only when a final result is available, that this
   * callback takes effect.
   * @param result Final result code of the behavior tree execution.
   */
  virtual void onTermination(const ExecutionResult & result);

public:
  /**
   * @brief Reset the global blackboard and clear all entries.
   * @return `true` if blackboard was cleared, `false` if executor is not idle meaning that the blackboard cannot be
   * cleared.
   */
  virtual bool clearGlobalBlackboard();

  /**
   * @brief Set the command that handles the control flow of the execution routine.
   *
   * @note The given control command is not validated, so the user must be careful when using this low-level setter.
   * @param cmd Control command.
   */
  void setControlCommand(ControlCommand cmd);

  /**
   * @brief Determine whether this executor is currently executing a behavior tree.
   * @return `true` if this executor is busy, `false` otherwise.
   */
  bool isBusy();

  /**
   * @brief Get a status code indicating the current state of execution.
   * @return Execution state.
   */
  ExecutionState getExecutionState();

  /**
   * @brief Get the name of the tree that is currently executing.
   *
   * If the executor is not busy, `NO_TREE_NAME` is returned.
   * @return Name of the currently executing tree.
   */
  std::string getTreeName();

  /**
   * @brief Get a shared pointer to the global blackboard instance.
   *
   * The global blackboard is used as a root blackboard on every tree that is being created. This means, that all
   * entries of the global blackboard are publicly available to the trees at runtime and can be queried using the '@'
   * prefix.
   *
   * \note This method creates a copy of the internal shared pointer object, so you are not able to modify the internal
   * pointer variable itself.
   * @return Shared pointer to the global blackboard.
   */
  TreeBlackboardSharedPtr getGlobalBlackboardPtr();

  /**
   * @brief Get a reference to the current behavior tree state observer.
   * @return Current state observer reference.
   * @throw auto_apms_behavior_tree::exceptions::TreeExecutorError if executor is not busy.
   */
  TreeStateObserver & getStateObserver();

  /**
   * @brief Get a shared pointer to the parent ROS 2 node.
   * @return Shared pointer to the `rclcpp::Node` associated with this executor.
   */
  rclcpp::Node::SharedPtr getNodePtr();

  /**
   * @brief Get the node's base interface. Is required to be able to register derived classes as ROS2 components.
   * @return Base interface of the `rclcpp::Node` associated with this executor.
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

  /**
   * @brief Get the callback group used for all waitables registered by behavior tree nodes.
   * @return Shared pointer to the behavior tree node waitables callback group.
   */
  rclcpp::CallbackGroup::SharedPtr getTreeNodeWaitablesCallbackGroupPtr();

  /**
   * @brief Get the ROS 2 executor instance used for spinning waitables registered by behavior tree nodes.
   * @return Shared pointer to the `rclcpp::executors::SingleThreadedExecutor` for behavior tree node waitables.
   */
  rclcpp::executors::SingleThreadedExecutor::SharedPtr getTreeNodeWaitablesExecutorPtr();

protected:
  /// Shared pointer to the parent ROS 2 node.
  rclcpp::Node::SharedPtr node_ptr_;
  /// Logger associated with the parent ROS 2 node.
  const rclcpp::Logger logger_;

private:
  rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group_ptr_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor_ptr_;
  TreeBlackboardSharedPtr global_blackboard_ptr_;
  std::unique_ptr<Tree> tree_ptr_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_ptr_;
  std::unique_ptr<TreeStateObserver> state_observer_ptr_;
  rclcpp::TimerBase::SharedPtr execution_timer_ptr_;
  ExecutionState prev_execution_state_;
  ControlCommand control_command_;
  bool execution_stopped_;
  std::string termination_reason_;
};

std::string toStr(TreeExecutorBase::ExecutionState state);

std::string toStr(TreeExecutorBase::ControlCommand cmd);

std::string toStr(TreeExecutorBase::TreeExitBehavior behavior);

std::string toStr(TreeExecutorBase::ExecutionResult result);

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename TimeRepT, typename TimeT>
inline std::shared_future<TreeExecutorBase::ExecutionResult> TreeExecutorBase::startExecution(
  TreeConstructor make_tree, const std::chrono::duration<TimeRepT, TimeT> & tick_rate, int groot2_port)
{
  return startExecution(
    make_tree, std::chrono::duration_cast<std::chrono::duration<double>>(tick_rate).count(), groot2_port);
}

template <typename TimeRepT, typename TimeT>
inline std::shared_future<TreeExecutorBase::ExecutionResult> TreeExecutorBase::startExecution(
  std::unique_ptr<Tree> tree, const std::chrono::duration<TimeRepT, TimeT> & tick_rate, int groot2_port)
{
  return startExecution(
    std::move(tree), std::chrono::duration_cast<std::chrono::duration<double>>(tick_rate).count(), groot2_port);
}

}  // namespace auto_apms_behavior_tree
