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
#include <memory>
#include <string>

#include "action_msgs/srv/cancel_goal.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/executors.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace auto_apms_behavior_tree::core
{

enum ActionNodeErrorCode
{
  SERVER_UNREACHABLE,
  SEND_GOAL_TIMEOUT,
  GOAL_REJECTED_BY_SERVER,
  INVALID_GOAL
};

/**
 * @brief Convert the action error code to string.
 * @param err Error code.
 * @return C-style string.
 */
inline const char * toStr(const ActionNodeErrorCode & err)
{
  switch (err) {
    case SERVER_UNREACHABLE:
      return "SERVER_UNREACHABLE";
    case SEND_GOAL_TIMEOUT:
      return "SEND_GOAL_TIMEOUT";
    case GOAL_REJECTED_BY_SERVER:
      return "GOAL_REJECTED_BY_SERVER";
    case INVALID_GOAL:
      return "INVALID_GOAL";
  }
  return nullptr;
}

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Generic behavior tree node wrapper for a ROS 2 action client.
 *
 * When ticked, this node sends an action goal and awaits the execution result asynchronously. When halted, the node
 * blocks until both the action was cancelled and the result was received. Inheriting classes must reimplement the
 * virtual methods as described below.
 *
 * By default, the name of the action will be determined as follows:
 *
 * 1. If a value is passed using the input port named `topic`, use that.
 *
 * 2. Otherwise, use the value from NodeRegistrationOptions::topic passed on construction as part of
 * RosNodeContext.
 *
 * It is possible to customize which port is used to determine the action name and also extend the input's value
 * with a prefix or suffix. This is achieved by including the special pattern `(input:<port_name>)` in
 * NodeRegistrationOptions::topic and replacing `<port_name>` with the desired input port name.
 *
 * **Example**: Given the user implements an input port `BT::InputPort<std::string>("my_port")`, one may create a client
 * for the action "foo/bar" by defining NodeRegistrationOptions::topic as `(input:my_port)/bar` and providing the
 * string "foo" to the port with name `my_port`.
 *
 * Additionally, the following characteristics depend on NodeRegistrationOptions:
 *
 * - wait_timeout: Period [s] (measured since tree construction) after the action is considered unreachable.
 *
 * - request_timeout: Period [s] (measured since sending a goal request) after the node aborts waiting for a server
 * response.
 *
 * - allow_unreachable: Flag whether to tolerate if the action is unreachable when trying to create the client.
 * If set to `true`, a warning is logged. Otherwise, an exception is raised.
 *
 * - logger_level: Minimum severity level enabled for logging using the ROS 2 Logger API.
 *
 * @tparam ActionT Type of the ROS 2 action.
 */
template <class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;

  struct ActionClientInstance
  {
    ActionClientInstance(
      rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & action_name);

    ActionClientPtr action_client;
    std::string name;
  };

  using ClientsRegistry = std::unordered_map<std::string, std::weak_ptr<ActionClientInstance>>;

public:
  using ActionType = ActionT;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  /**
   * @brief Constructor.
   *
   * Derived nodes are automatically created by TreeBuilder::instantiate when included inside a node manifest
   * associated with the behavior tree resource.
   * @param instance_name Name given to this specific node instance.
   * @param config Structure of internal data determined at runtime by `BT::BehaviorTreeFactory`.
   * @param context Additional parameters specific to ROS 2 determined at runtime by TreeBuilder.
   */
  explicit RosActionNode(const std::string & instance_name, const Config & config, Context context);

  virtual ~RosActionNode() = default;

  /**
   * @brief Derived nodes implementing the static method RosActionNode::providedPorts may call this method to also
   * include the default port for ROS 2 behavior tree nodes.
   *
   * @param addition Additional ports to add to the ports list.
   * @return List of ports containing the default port along with node-specific ports.
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>("topic", "Name of the ROS 2 action.")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief If a behavior tree requires input/output data ports, the developer must define this method accordingly.
   * @return List of ports used by this node.
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief Callback invoked when the node is halted by the behavior tree.
   *
   * Afterwards, the ROS 2 action will be cancelled.
   *
   * By default, this callback does nothing.
   */
  virtual void onHalt();

  /**
   * @brief Set the goal message to be sent to the ROS 2 action.
   *
   * The node may deny to send a goal by returning `false`. Otherwise, this method should return `true`.
   *
   * By default, this callback always returns `true`.
   * @param goal Reference to the goal message.
   * @return `false` if the request should not be sent. In that case, onFailure(INVALID_GOAL) will be called.
   */
  virtual bool setGoal(Goal & goal);

  /**
   * @brief Callback invoked after the result that is sent by the action server when the goal terminated was received.
   *
   * Based on the result message, the node may return `BT::NodeStatus::SUCCESS` or `BT::NodeStatus::FAILURE`.
   *
   * By default, this callback returns `BT::NodeStatus::SUCCESS` if the action finished or was canceled successfully.
   * Otherwise it returns `BT::NodeStatus::FAILURE`.
   * @param result Reference to the result message.
   * @return Final return status of the node.
   */
  virtual BT::NodeStatus onResultReceived(const WrappedResult & result);

  /**
   * @brief Callback invoked after action feedback was received.
   *
   * The node may cancel the current action by returning `BT::NodeStatus::SUCCESS` or `BT::NodeStatus::FAILURE`.
   * Otherwise, this should return `BT::NodeStatus::RUNNING` to indicate that the action should continue.
   *
   * By default, this callback always returns `BT::NodeStatus::RUNNING`.
   * @param feedback Received feedback message.
   * @return Final return status of the node.
   */
  virtual BT::NodeStatus onFeedback(const Feedback & feedback);

  /**
   * @brief Callback invoked when one of the errors in ActionNodeErrorCode occur.
   *
   * Based on the error code, the node may return `BT::NodeStatus::SUCCESS` or `BT::NodeStatus::FAILURE`.
   *
   * By default, this callback throws `auto_apms_behavior_tree::exceptions::RosNodeError` and interrupts the tree.
   * @param error Code for the error that has occurred.
   * @return Final return status of the node.
   */
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error);

  /**
   * @brief Synchronous method that sends a request to the server to cancel the current action.
   *
   * This method returns as soon as the action is successfully cancelled or an error occurred in the process.
   */
  void cancelGoal();

  /**
   * @brief Create the client of the ROS 2 action.
   * @param action_name Name of the action.
   * @return `true` if the client was created successfully, `false` otherwise.
   */
  bool createClient(const std::string & action_name);

  /**
   * @brief Get the name of the action this node connects with.
   * @return String representing the action name.
   */
  std::string getActionName() const;

protected:
  const Context context_;
  const rclcpp::Logger logger_;

  void halt() override final;

  BT::NodeStatus tick() override final;

private:
  static std::mutex & getMutex();

  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry & getRegistry();

  bool dynamic_client_instance_ = false;
  std::shared_ptr<ActionClientInstance> client_instance_;
  std::string action_client_key_;
  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  typename GoalHandle::SharedPtr goal_handle_;
  rclcpp::Time time_goal_sent_;
  BT::NodeStatus on_feedback_state_change_;
  bool goal_response_received_;  // We must use this additional flag because goal_handle_ may be nullptr if rejected
  bool goal_rejected_;
  bool result_received_;
  bool cancel_requested_;
  WrappedResult result_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ActionT>
RosActionNode<ActionT>::ActionClientInstance::ActionClientInstance(
  rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & action_name)
{
  action_client = rclcpp_action::create_client<ActionT>(node, action_name, group);
  name = action_name;
}

template <class ActionT>
inline RosActionNode<ActionT>::RosActionNode(const std::string & instance_name, const Config & config, Context context)
: BT::ActionNodeBase(instance_name, config),
  context_(context),
  logger_(context.getChildLogger(auto_apms_util::toSnakeCase(instance_name)))
{
  // Consider aliasing in ports and copy the values from aliased to original ports
  this->modifyPortsRemapping(context_.copyAliasedPortValuesToOriginalPorts(this));

  if (const BT::Expected<std::string> expected_name = context_.getTopicName(this)) {
    createClient(expected_name.value());
  } else {
    // We assume that determining the action name requires a blackboard pointer, which cannot be evaluated at
    // construction time. The expression will be evaluated each time before the node is ticked the first time after
    // successful execution.
    dynamic_client_instance_ = true;
  }
}

template <class ActionT>
inline void RosActionNode<ActionT>::onHalt()
{
}

template <class ActionT>
inline bool RosActionNode<ActionT>::setGoal(Goal & /*goal*/)
{
  return true;
}

template <class ActionT>
inline BT::NodeStatus RosActionNode<ActionT>::onResultReceived(const WrappedResult & result)
{
  std::string result_str;
  switch (result.code) {
    case rclcpp_action::ResultCode::ABORTED:
      result_str = "ABORTED";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      result_str = "CANCELED";
      break;
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_str = "SUCCEEDED";
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      result_str = "UNKNOWN";
      break;
  }
  RCLCPP_DEBUG(
    logger_, "%s - Goal completed. Received result %s.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
    result_str.c_str());
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) return BT::NodeStatus::SUCCESS;
  if (cancel_requested_ && result.code == rclcpp_action::ResultCode::CANCELED) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

template <class ActionT>
inline BT::NodeStatus RosActionNode<ActionT>::onFeedback(const Feedback & /*feedback*/)
{
  return BT::NodeStatus::RUNNING;
}

template <class ActionT>
inline BT::NodeStatus RosActionNode<ActionT>::onFailure(ActionNodeErrorCode error)
{
  const std::string msg = context_.getFullyQualifiedTreeNodeName(this) + " - Unexpected error " +
                          std::to_string(error) + ": " + toStr(error) + ".";
  RCLCPP_ERROR_STREAM(logger_, msg);
  throw exceptions::RosNodeError(msg);
}

template <class T>
inline void RosActionNode<T>::cancelGoal()
{
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_ptr = context_.executor_.lock();
  if (!executor_ptr) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) + " - Cannot cancel goal for action '" + client_instance_->name +
      "' since the pointer to the associated ROS 2 executor expired.");
  }

  if (future_goal_handle_.valid()) {
    RCLCPP_DEBUG(
      logger_, "%s - Awaiting goal response before trying to cancel goal...",
      context_.getFullyQualifiedTreeNodeName(this).c_str());
    // Here the discussion is if we should block or put a timer for the waiting
    const rclcpp::FutureReturnCode ret =
      executor_ptr->spin_until_future_complete(future_goal_handle_, context_.registration_options_.request_timeout);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      // Do nothing in case of INTERRUPT or TIMEOUT
      return;
    }
    goal_handle_ = future_goal_handle_.get();
    future_goal_handle_ = {};
    goal_rejected_ = goal_handle_ == nullptr;
  }

  // If goal was rejected or handle has been invalidated, we do not need to cancel
  if (goal_rejected_) {
    RCLCPP_DEBUG(
      logger_, "%s - Goal was rejected. Nothing to cancel.", context_.getFullyQualifiedTreeNodeName(this).c_str());
    return;
  };

  // If goal was accepted, but goal handle is nullptr, result callback was already called which means that the goal has
  // already reached a terminal state
  if (!goal_handle_) {
    RCLCPP_DEBUG(
      logger_, "%s - Goal has already reached a terminal state. Nothing to cancel.",
      context_.getFullyQualifiedTreeNodeName(this).c_str());
    return;
  };

  const std::string uuid_str = rclcpp_action::to_string(goal_handle_->get_goal_id());
  RCLCPP_DEBUG(
    logger_, "%s - Canceling goal %s for action '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
    uuid_str.c_str(), client_instance_->name.c_str());

  // Send the cancellation request
  std::shared_future<std::shared_ptr<typename ActionClient::CancelResponse>> future_cancel_response =
    client_instance_->action_client->async_cancel_goal(goal_handle_);
  if (const auto code = executor_ptr->spin_until_future_complete(
        future_cancel_response, context_.registration_options_.request_timeout);
      code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(
      logger_, "%s - Failed to wait for response for cancellation request (Code: %s).",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), rclcpp::to_string(code).c_str());

    // Make sure goal handle is invalidated
    goal_handle_ = nullptr;
    return;
  }

  // Check the response for the cancellation request
  if (!future_cancel_response.get()) {
    throw std::logic_error("Shared pointer to cancel response is nullptr.");
  }
  typename ActionClient::CancelResponse cancel_response = *future_cancel_response.get();
  std::string cancel_response_str;
  switch (cancel_response.return_code) {
    case action_msgs::srv::CancelGoal::Response::ERROR_REJECTED:
      cancel_response_str = "ERROR_REJECTED";
      break;
    case action_msgs::srv::CancelGoal::Response::ERROR_UNKNOWN_GOAL_ID:
      cancel_response_str = "ERROR_UNKNOWN_GOAL_ID";
      break;
    case action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED:
      cancel_response_str = "ERROR_GOAL_TERMINATED";
      break;
    default:
      cancel_response_str = "ERROR_NONE";
      break;
  }
  if (cancel_response.return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
    RCLCPP_DEBUG(
      logger_,
      "%s - Cancellation request of goal %s for action '%s' was accepted (Response: %s). Awaiting completion...",
      context_.getFullyQualifiedTreeNodeName(this).c_str(),
      rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str(), client_instance_->name.c_str(),
      cancel_response_str.c_str());

    // Wait for the cancellation to be complete (goal result received)
    std::shared_future<WrappedResult> future_goal_result =
      client_instance_->action_client->async_get_result(goal_handle_);
    if (const auto code =
          executor_ptr->spin_until_future_complete(future_goal_result, context_.registration_options_.request_timeout);
        code == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_DEBUG(
        logger_, "%s - Goal %s for action '%s' was cancelled successfully.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), uuid_str.c_str(), client_instance_->name.c_str());
    } else {
      RCLCPP_WARN(
        logger_, "%s - Failed to wait until cancellation completed (Code: %s).",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), rclcpp::to_string(code).c_str());
    }
  } else {
    // The cancellation request was rejected. If this was due to the goal having terminated normally before the request
    // was processed by the server, we consider the cancellation as a success. Otherwise we warn.
    if (cancel_response.return_code == action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED) {
      RCLCPP_DEBUG(
        logger_, "%s - Goal %s for action '%s' has already terminated (Response: %s). Nothing to cancel.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), uuid_str.c_str(), client_instance_->name.c_str(),
        cancel_response_str.c_str());
    } else {
      RCLCPP_WARN(
        logger_, "%s - Cancellation request was rejected (Response: %s).",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), cancel_response_str.c_str());
    }
  }

  // Make sure goal handle is invalidated
  goal_handle_ = nullptr;
}

template <class T>
inline void RosActionNode<T>::halt()
{
  if (status() == BT::NodeStatus::RUNNING) {
    cancel_requested_ = true;
    onHalt();
    cancelGoal();
    resetStatus();
  }
}

template <class T>
inline BT::NodeStatus RosActionNode<T>::tick()
{
  if (!rclcpp::ok()) {
    halt();
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) + " - ROS 2 context has been shut down.");
  }

  // If client has been set up in derived constructor, event though this constructor couldn't, we discard the intention
  // of dynamically creating the client
  if (dynamic_client_instance_ && client_instance_) {
    dynamic_client_instance_ = false;
  }

  // Try again to create the client on first tick if this was not possible during construction or if client should be
  // created from a blackboard entry on the start of every iteration
  if (status() == BT::NodeStatus::IDLE && dynamic_client_instance_) {
    const BT::Expected<std::string> expected_name = context_.getTopicName(this);
    if (expected_name) {
      createClient(expected_name.value());
    } else {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) +
        " - Cannot create the action client because the action name couldn't be resolved using "
        "the expression specified by the node's registration parameters (" +
        NodeRegistrationOptions::PARAM_NAME_ROS2TOPIC + ": " + context_.registration_options_.topic +
        "). Error message: " + expected_name.error());
    }
  }

  if (!client_instance_) {
    throw exceptions::RosNodeError(context_.getFullyQualifiedTreeNodeName(this) + " - client_instance_ is nullptr.");
  }

  auto & action_client = client_instance_->action_client;

  //------------------------------------------
  auto check_status = [this](BT::NodeStatus status) {
    if (!isStatusCompleted(status)) {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - The callback must return either SUCCESS or FAILURE.");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if (status() == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);

    goal_response_received_ = false;
    goal_rejected_ = false;
    result_received_ = false;
    cancel_requested_ = false;
    on_feedback_state_change_ = BT::NodeStatus::RUNNING;
    result_ = {};

    // Check if server is ready
    if (!action_client->action_server_is_ready()) {
      return onFailure(SERVER_UNREACHABLE);
    }

    Goal goal;
    if (!setGoal(goal)) {
      return check_status(onFailure(INVALID_GOAL));
    }

    typename ActionClient::SendGoalOptions goal_options;
    goal_options.goal_response_callback = [this](typename GoalHandle::SharedPtr goal_handle) {
      // Indicate that a goal response has been received and let tick() do the rest
      this->goal_response_received_ = true;
      this->goal_rejected_ = goal_handle == nullptr;
      this->goal_handle_ = goal_handle;
    };
    goal_options.feedback_callback =
      [this](typename GoalHandle::SharedPtr /*goal_handle*/, const std::shared_ptr<const Feedback> feedback) {
        this->on_feedback_state_change_ = onFeedback(*feedback);
        if (this->on_feedback_state_change_ == BT::NodeStatus::IDLE) {
          throw std::logic_error(
            this->context_.getFullyQualifiedTreeNodeName(this) + " - onFeedback() must not return IDLE.");
        }
        this->emitWakeUpSignal();
      };
    goal_options.result_callback = [this](const WrappedResult & result) {
      // The result callback is also invoked when goal is rejected (code: CANCELED), but we only want to call
      // onResultReceived if the goal was accepted and we want to return prematurely. Therefore, we use the
      // cancel_requested_ flag.
      if (this->cancel_requested_) {
        // If the node is to return prematurely, we must invoke onResultReceived here. The returned status has no effect
        // in this case.
        this->onResultReceived(result);
      }
      this->result_received_ = true;
      this->goal_handle_ = nullptr;  // Reset internal goal handle when result was received
      this->result_ = result;
      this->emitWakeUpSignal();
    };

    future_goal_handle_ = action_client->async_send_goal(goal, goal_options);
    time_goal_sent_ = context_.getCurrentTime();
    return BT::NodeStatus::RUNNING;
  }

  if (status() == BT::NodeStatus::RUNNING) {
    std::unique_lock<std::mutex> lock(getMutex());

    // FIRST case: check if the goal request has a timeout as long as goal_response_received_ is false (Is set to true
    // as soon as a goal response is received)
    if (!goal_response_received_) {
      // See if we must time out
      if ((context_.getCurrentTime() - time_goal_sent_) > context_.registration_options_.request_timeout) {
        return check_status(onFailure(SEND_GOAL_TIMEOUT));
      }
      return BT::NodeStatus::RUNNING;
    } else if (future_goal_handle_.valid()) {
      // We noticed, that a goal response has just been received and have to prepare the next steps now
      future_goal_handle_ = {};  // Invalidate future since it's obsolete now and it indicates that we've done this step

      if (goal_rejected_) return check_status(onFailure(GOAL_REJECTED_BY_SERVER));
      RCLCPP_DEBUG(
        logger_, "%s - Goal %s accepted by server, waiting for result.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(),
        rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
    }

    // SECOND case: onFeedback requested a stop
    if (on_feedback_state_change_ != BT::NodeStatus::RUNNING) {
      cancel_requested_ = true;
      cancelGoal();
      return on_feedback_state_change_;
    }

    // THIRD case: result received
    if (result_received_) {
      return check_status(onResultReceived(result_));
    }
  }
  return BT::NodeStatus::RUNNING;
}

template <class ActionT>
inline bool RosActionNode<ActionT>::createClient(const std::string & action_name)
{
  if (action_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - Argument action_name is empty when trying to create the client.");
  }

  // Check if the action with given name is already set up
  if (
    client_instance_ && action_name == client_instance_->name &&
    client_instance_->action_client->action_server_is_ready()) {
    return true;
  }

  std::unique_lock lk(getMutex());

  rclcpp::Node::SharedPtr node = context_.nh_.lock();
  if (!node) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - The weak pointer to the ROS 2 node expired. The tree node doesn't "
      "take ownership of it.");
  }
  rclcpp::CallbackGroup::SharedPtr group = context_.cb_group_.lock();
  if (!group) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - The weak pointer to the ROS 2 callback group expired. The tree node doesn't "
      "take ownership of it.");
  }
  action_client_key_ = std::string(node->get_fully_qualified_name()) + "/" + action_name;

  auto & registry = getRegistry();
  auto it = registry.find(action_client_key_);
  if (it == registry.end() || it->second.expired()) {
    client_instance_ = std::make_shared<ActionClientInstance>(node, group, action_name);
    registry.insert({action_client_key_, client_instance_});
    RCLCPP_DEBUG(
      logger_, "%s - Created client for action '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
      action_name.c_str());
  } else {
    client_instance_ = it->second.lock();
  }

  bool found = client_instance_->action_client->wait_for_action_server(context_.registration_options_.wait_timeout);
  if (!found) {
    std::string msg = context_.getFullyQualifiedTreeNodeName(this) + " - Action server with name '" +
                      client_instance_->name + "' is not reachable.";
    if (context_.registration_options_.allow_unreachable) {
      RCLCPP_WARN_STREAM(logger_, msg);
    } else {
      RCLCPP_ERROR_STREAM(logger_, msg);
      throw exceptions::RosNodeError(msg);
    }
  }
  return found;
}

template <class ActionT>
inline std::string RosActionNode<ActionT>::getActionName() const
{
  if (client_instance_) return client_instance_->name;
  return "unknown";
}

template <class ActionT>
inline std::mutex & RosActionNode<ActionT>::getMutex()
{
  static std::mutex action_client_mutex;
  return action_client_mutex;
}

template <class ActionT>
inline typename RosActionNode<ActionT>::ClientsRegistry & RosActionNode<ActionT>::getRegistry()
{
  static ClientsRegistry action_clients_registry;
  return action_clients_registry;
}

}  // namespace auto_apms_behavior_tree::core