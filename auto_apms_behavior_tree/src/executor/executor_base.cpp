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

#include "auto_apms_behavior_tree/executor/executor_base.hpp"

#include <chrono>

#include "auto_apms_util/container.hpp"
#include "auto_apms_util/logging.hpp"

namespace auto_apms_behavior_tree
{

TreeExecutorBase::TreeExecutorBase(
  rclcpp::Node::SharedPtr node_ptr, rclcpp::CallbackGroup::SharedPtr tree_node_callback_group_ptr)
: node_ptr_(node_ptr),
  logger_(node_ptr_->get_logger()),
  tree_node_waitables_callback_group_ptr_(tree_node_callback_group_ptr),
  tree_node_waitables_executor_ptr_(rclcpp::executors::SingleThreadedExecutor::make_shared()),
  global_blackboard_ptr_(TreeBlackboard::create()),
  control_command_(ControlCommand::RUN),
  execution_stopped_(true)
{
  // The behavior tree node callback group is intended to be passed to all nodes and used when adding subscriptions,
  // publishers, services, actions etc. It is associated with a standalone single threaded executor, which is spun in
  // between ticks, to make sure that pending work is executed while the main tick routine is sleeping.
  if (!tree_node_waitables_callback_group_ptr_) {
    tree_node_waitables_callback_group_ptr_ =
      node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  }

  // Add the behavior tree node callback group to the internal executor
  tree_node_waitables_executor_ptr_->add_callback_group(
    tree_node_waitables_callback_group_ptr_, get_node_base_interface());
}

std::shared_future<TreeExecutorBase::ExecutionResult> TreeExecutorBase::startExecution(
  TreeConstructor make_tree, double tick_rate_sec, int groot2_port)
{
  if (isBusy()) {
    throw exceptions::TreeExecutorError(
      "Cannot start execution with tree '" + getTreeName() + "' currently executing.");
  }

  try {
    // Lives inside BT::Tree once it is created
    TreeBlackboardSharedPtr main_tree_bb_ptr = TreeBlackboard::create(global_blackboard_ptr_);
    // Create the tree from the provided callback and store it in the executor
    tree_ptr_.reset(new Tree(make_tree(main_tree_bb_ptr)));
  } catch (const std::exception & e) {
    throw exceptions::TreeBuildError(
      "Cannot start execution because creating the tree failed: " + std::string(e.what()));
  }

  // Groot2 publisher
  groot2_publisher_ptr_.reset();
  if (groot2_port != -1) {
    try {
      groot2_publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(*tree_ptr_, groot2_port);
    } catch (const std::exception & e) {
      throw exceptions::TreeExecutorError(
        "Failed to initialize Groot2 publisher with port " + std::to_string(groot2_port) + ": " + e.what());
    }
  }

  // Tree state observer
  state_observer_ptr_.reset();
  state_observer_ptr_ = std::make_unique<TreeStateObserver>(*tree_ptr_, logger_);
  state_observer_ptr_->enableTransitionToIdle(false);

  /* Start execution timer */

  // Reset state variables
  prev_execution_state_ = getExecutionState();
  control_command_ = ControlCommand::RUN;
  termination_reason_ = "";
  execution_stopped_ = true;

  // Create promise for asynchronous execution and configure termination callback
  auto promise_ptr = std::make_shared<std::promise<ExecutionResult>>();
  TerminationCallback termination_callback = [this, promise_ptr](ExecutionResult result, const std::string & msg) {
    RCLCPP_INFO(
      logger_, "Terminating tree '%s' from state %s.", getTreeName().c_str(), toStr(getExecutionState()).c_str());
    if (result == ExecutionResult::ERROR) {
      RCLCPP_ERROR(logger_, "Termination reason: %s", msg.c_str());
    } else {
      RCLCPP_INFO(logger_, "Termination reason: %s", msg.c_str());
    }
    onTermination(result);  // is evaluated before the timer is cancelled, which means the execution state has not
                            // changed yet during the callback and can be evaluated to inspect the terminal state.
    promise_ptr->set_value(result);
    execution_timer_ptr_->cancel();
    tree_ptr_.reset();  // Release the memory allocated by the tree
  };

  // NOTE: The main callback timer is using the node's default callback group
  const std::chrono::nanoseconds period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(tick_rate_sec));
  execution_timer_ptr_ = node_ptr_->create_wall_timer(period, [this, period, termination_callback]() {
    // Collect and process incoming messages before ticking
    tree_node_waitables_executor_ptr_->spin_all(period);

    // Tick the tree, evaluate control commands and handle the returned tree status
    tick_callback_(termination_callback);
  });
  return promise_ptr->get_future();
}

void TreeExecutorBase::tick_callback_(TerminationCallback termination_callback)
{
  const ExecutionState this_execution_state = getExecutionState();
  if (prev_execution_state_ != this_execution_state) {
    RCLCPP_DEBUG(
      logger_, "Executor for tree '%s' changed state from '%s' to '%s'.", getTreeName().c_str(),
      toStr(prev_execution_state_).c_str(), toStr(this_execution_state).c_str());
    prev_execution_state_ = this_execution_state;
  }

  /* Evaluate control command */

  execution_stopped_ = false;
  bool do_on_tick = true;
  switch (control_command_) {
    case ControlCommand::PAUSE:
      execution_stopped_ = true;
      return;
    case ControlCommand::RUN:
      if (this_execution_state == ExecutionState::STARTING) {
        // Evaluate initial tick callback before ticking for the first time since the timer has been created
        if (!onInitialTick()) {
          do_on_tick = false;
          termination_reason_ = "onInitialTick() returned false.";
        }
      }
      // Evaluate tick callback everytime before actually ticking.
      // This also happens the first time except if onInitialTick() returned false
      if (do_on_tick) {
        if (onTick()) {
          break;
        } else {
          termination_reason_ = "onTick() returned false.";
        }
      }

      // Fall through to terminate if any of the callbacks returned false
      [[fallthrough]];
    case ControlCommand::TERMINATE:
      if (this_execution_state == ExecutionState::HALTED) {
        termination_callback(
          ExecutionResult::TERMINATED_PREMATURELY,
          termination_reason_.empty() ? "Control command was set to TERMINATE." : termination_reason_);
        return;
      }

      // Fall through to halt tree before termination
      [[fallthrough]];
    case ControlCommand::HALT:
      // Check if already halted
      if (this_execution_state != ExecutionState::HALTED) {
#ifdef AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_THROW_ON_TICK_ERROR
        tree_ptr_->haltTree();
#else
        try {
          tree_ptr_->haltTree();
        } catch (const std::exception & e) {
          termination_callback(
            ExecutionResult::ERROR,
            "Error during haltTree() on command " + toStr(control_command_) + ": " + std::string(e.what()));
        }
#endif
      }
      return;
    default:
      throw std::logic_error(
        "Handling control command " + std::to_string(static_cast<int>(control_command_)) + " '" +
        toStr(control_command_) + "' is not implemented.");
  }

  /* Tick the tree instance */

  BT::NodeStatus bt_status = BT::NodeStatus::IDLE;
#ifdef AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_THROW_ON_TICK_ERROR
  bt_status = tree_ptr_->tickExactlyOnce();
#else
  try {
    // It is important to tick EXACTLY once to prevent loops induced by BT nodes from blocking
    bt_status = tree_ptr_->tickExactlyOnce();
  } catch (const std::exception & e) {
    std::string msg = "Ran into an exception during tick: " + std::string(e.what());
    try {
      tree_ptr_->haltTree();  // Try to halt tree before aborting
    } catch (const std::exception & e) {
      msg += "\nDuring haltTree(), another exception occurred: " + std::string(e.what());
    }
    termination_callback(ExecutionResult::ERROR, msg);
    return;
  }
#endif

  if (!afterTick()) {
    termination_callback(ExecutionResult::TERMINATED_PREMATURELY, "afterTick() returned false.");
    return;
  }

  if (bt_status == BT::NodeStatus::RUNNING) return;

  /* Determine how to handle the behavior tree execution result */

  if (!(bt_status == BT::NodeStatus::SUCCESS || bt_status == BT::NodeStatus::FAILURE)) {
    throw std::logic_error(
      "bt_status is " + BT::toStr(bt_status) + ". Must be one of SUCCESS or FAILURE at this point.");
  }
  const bool success = bt_status == BT::NodeStatus::SUCCESS;
  switch (onTreeExit(success)) {
    case TreeExitBehavior::TERMINATE:
      termination_callback(
        success ? ExecutionResult::TREE_SUCCEEDED : ExecutionResult::TREE_FAILED,
        "Terminated on tree result " + BT::toStr(bt_status) + ".");
      return;
    case TreeExitBehavior::RESTART:
      control_command_ = ControlCommand::RUN;
      return;
  }

  throw std::logic_error("Execution routine is not intended to proceed to this statement.");
}

bool TreeExecutorBase::onInitialTick() { return true; }

bool TreeExecutorBase::onTick() { return true; }

bool TreeExecutorBase::afterTick() { return true; }

TreeExecutorBase::TreeExitBehavior TreeExecutorBase::onTreeExit(bool /*success*/)
{
  return TreeExitBehavior::TERMINATE;
}

void TreeExecutorBase::onTermination(const ExecutionResult & /*result*/) {}

bool TreeExecutorBase::clearGlobalBlackboard()
{
  ExecutionState curr_state = getExecutionState();
  if (curr_state == ExecutionState::IDLE) {
    global_blackboard_ptr_ = TreeBlackboard::create();
    return true;
  }
  RCLCPP_WARN(
    logger_, "clearGlobalBlackboard() was called when executor is %s, but this is only allowed when IDLE. Ignoring...",
    toStr(curr_state).c_str());
  return false;
}

void TreeExecutorBase::setControlCommand(ControlCommand cmd) { control_command_ = cmd; }

bool TreeExecutorBase::isBusy() { return execution_timer_ptr_ && !execution_timer_ptr_->is_canceled(); }

TreeExecutorBase::ExecutionState TreeExecutorBase::getExecutionState()
{
  if (isBusy()) {
    if (!tree_ptr_) throw std::logic_error("tree_ptr_ cannot be nullptr when execution is started.");
    if (tree_ptr_->rootNode()->status() == BT::NodeStatus::IDLE) {
      // The root node being IDLE here means one of the following:
      // - the tree hasn't been ticked yet since its creation
      // - the tree was halted or just finished executing
      return execution_stopped_ ? ExecutionState::STARTING : ExecutionState::HALTED;
    }
    return execution_stopped_ ? ExecutionState::PAUSED : ExecutionState::RUNNING;
  }
  return ExecutionState::IDLE;
}

std::string TreeExecutorBase::getTreeName()
{
  if (tree_ptr_) return tree_ptr_->subtrees[0]->tree_ID;
  return "NO_TREE_NAME";
}

TreeBlackboardSharedPtr TreeExecutorBase::getGlobalBlackboardPtr() { return global_blackboard_ptr_; }

TreeStateObserver & TreeExecutorBase::getStateObserver()
{
  if (!state_observer_ptr_) {
    throw exceptions::TreeExecutorError("Cannot get state observer because executor is not busy.");
  }
  return *state_observer_ptr_;
}

rclcpp::Node::SharedPtr TreeExecutorBase::getNodePtr() { return node_ptr_; }

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr TreeExecutorBase::get_node_base_interface()
{
  return node_ptr_->get_node_base_interface();
}

rclcpp::CallbackGroup::SharedPtr TreeExecutorBase::getTreeNodeWaitablesCallbackGroupPtr()
{
  return tree_node_waitables_callback_group_ptr_;
}

rclcpp::executors::SingleThreadedExecutor::SharedPtr TreeExecutorBase::getTreeNodeWaitablesExecutorPtr()
{
  return tree_node_waitables_executor_ptr_;
}

std::string toStr(TreeExecutorBase::ExecutionState state)
{
  switch (state) {
    case TreeExecutorBase::ExecutionState::IDLE:
      return "IDLE";
    case TreeExecutorBase::ExecutionState::STARTING:
      return "STARTING";
    case TreeExecutorBase::ExecutionState::RUNNING:
      return "RUNNING";
    case TreeExecutorBase::ExecutionState::PAUSED:
      return "PAUSED";
    case TreeExecutorBase::ExecutionState::HALTED:
      return "HALTED";
  }
  return "undefined";
}

std::string toStr(TreeExecutorBase::ControlCommand cmd)
{
  switch (cmd) {
    case TreeExecutorBase::ControlCommand::RUN:
      return "RUN";
    case TreeExecutorBase::ControlCommand::PAUSE:
      return "PAUSE";
    case TreeExecutorBase::ControlCommand::HALT:
      return "HALT";
    case TreeExecutorBase::ControlCommand::TERMINATE:
      return "TERMINATE";
  }
  return "undefined";
}

std::string toStr(TreeExecutorBase::TreeExitBehavior behavior)
{
  switch (behavior) {
    case TreeExecutorBase::TreeExitBehavior::TERMINATE:
      return "TERMINATE";
    case TreeExecutorBase::TreeExitBehavior::RESTART:
      return "RESTART";
  }
  return "undefined";
}

std::string toStr(TreeExecutorBase::ExecutionResult result)
{
  switch (result) {
    case TreeExecutorBase::ExecutionResult::TREE_SUCCEEDED:
      return "TREE_SUCCEEDED";
    case TreeExecutorBase::ExecutionResult::TREE_FAILED:
      return "TREE_FAILED";
    case TreeExecutorBase::ExecutionResult::TERMINATED_PREMATURELY:
      return "TERMINATED_PREMATURELY";
    case TreeExecutorBase::ExecutionResult::ERROR:
      return "ERROR";
  }
  return "undefined";
}

}  // namespace auto_apms_behavior_tree
