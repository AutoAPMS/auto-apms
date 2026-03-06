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

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/qos.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Generic behavior tree node wrapper for a ROS 2 subscriber.
 *
 * When ticked, this node may inspect the last message that has been received on a specific topic. Inheriting classes
 * must reimplement the virtual methods as described below.
 *
 * By default, the name of the topic will be determined as follows:
 *
 * 1. If a value is passed using the input port named `topic`, use that.
 *
 * 2. Otherwise, use the value from NodeRegistrationOptions::topic passed on construction as part of
 * RosNodeContext.
 *
 * It is possible to customize which port is used to determine the topic name and also extend the input's value
 * with a prefix or suffix. This is achieved by including the special pattern `(input:<port_name>)` in
 * NodeRegistrationOptions::topic and replacing `<port_name>` with the desired input port name.
 *
 * **Example**: Given the user implements an input port `BT::InputPort<std::string>("my_port")`, one may create a client
 * for the topic "foo/bar" by defining NodeRegistrationOptions::topic as `(input:my_port)/bar` and providing the
 * string "foo" to the port with name `my_port`.
 *
 * Additionally, the following characteristics depend on NodeRegistrationOptions:
 *
 * - logger_level: Minimum severity level enabled for logging using the ROS 2 Logger API.
 *
 * @tparam MessageT Type of the ROS 2 message.
 */
template <class MessageT>
class RosSubscriberNode : public BT::ConditionNode
{
  using Subscriber = typename rclcpp::Subscription<MessageT>;

  struct SubscriberInstance
  {
    SubscriberInstance(
      rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & topic_name,
      const rclcpp::QoS & qos);

    std::shared_ptr<Subscriber> subscriber;
    std::vector<std::pair<const void *, std::function<void(const std::shared_ptr<MessageT>)>>> callbacks;
    std::shared_ptr<MessageT> last_msg;
    std::string name;

    void addCallback(
      const void * callback_owner, const std::function<void(const std::shared_ptr<MessageT>)> & callback);
    void removeCallback(const void * callback_owner);
    void broadcast(const std::shared_ptr<MessageT> & msg);
  };

  using SubscribersRegistry = std::unordered_map<std::string, std::weak_ptr<SubscriberInstance>>;

public:
  using MessageType = MessageT;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  /**
   * @brief Constructor.
   *
   * Derived nodes are automatically created by TreeBuilder::instantiate when included inside a node manifest
   * associated with the behavior tree resource.
   * @param instance_name Name given to this specific node instance.
   * @param config Structure of internal data determined at runtime by BT::BehaviorTreeFactory.
   * @param context Additional parameters specific to ROS 2 determined at runtime by TreeBuilder.
   * @param qos Quality of service settings forwarded to the subscriber.
   */
  explicit RosSubscriberNode(
    const std::string & instance_name, const Config & config, Context context, const rclcpp::QoS & qos = {10});

  virtual ~RosSubscriberNode()
  {
    if (sub_instance_) {
      sub_instance_->removeCallback(this);
    }
  }

  /**
   * @brief Derived nodes implementing the static method RosSubscriberNode::providedPorts may call this method to also
   * include the default port for ROS 2 behavior tree nodes.
   *
   * @param addition Additional ports to add to the ports list.
   * @return List of ports containing the default port along with node-specific ports.
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>("topic", "Name of the ROS 2 topic to subscribe to.")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief If a behavior tree requires input/output data ports, the developer must define this method accordingly.
   * @return List of ports used by this node.
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief Callback invoked when the node is ticked.
   *
   * By default, this method forwards the most recent message to RosSubscriberNode::onMessageReceived and leaves it up
   * to this method to determine the node's return status. If no message has been received yet, it immediately returns
   * BT::NodeStatus::FAILURE. If you want to change that behavior, you should override this method.
   * @param last_msg_ptr The latest message received, since the last tick. Will be `nullptr` if no new message was
   * received.
   * @return Return status of the node based on @p last_msg_ptr.
   */
  virtual BT::NodeStatus onTick(const std::shared_ptr<MessageT> & last_msg_ptr);

  /**
   * @brief Callback invoked when the node is ticked and a valid message has been received.
   *
   * This callback won't be invoked if no message is available at the time this node is ticked. If you need
   * to evaluate an expression every time this node is ticked, refer to RosSubscriberNode::onTick instead and change its
   * default implementation.
   * @param msg Most recently received message.
   * @return Return status of the node based on @p msg.
   */
  virtual BT::NodeStatus onMessageReceived(const MessageT & msg);

  /**
   * @brief Create the ROS 2 subscriber.
   * @param topic_name Name of the topic.
   * @return `true` if the subscriber was created successfully, `false` otherwise.
   */
  bool createSubscriber(const std::string & topic_name);

  /**
   * @brief Get the name of the topic name this node subscribes to.
   * @return String representing the topic name.
   */
  std::string getTopicName() const;

protected:
  const Context context_;
  const rclcpp::Logger logger_;

  BT::NodeStatus tick() override final;

private:
  static std::mutex & registryMutex();

  // contains the fully-qualified name of the node and the name of the topic
  static SubscribersRegistry & getRegistry();

  const rclcpp::QoS qos_;
  std::string topic_name_;
  bool dynamic_client_instance_ = false;
  std::shared_ptr<SubscriberInstance> sub_instance_;
  std::shared_ptr<MessageT> last_msg_;
  std::string subscriber_key_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class MessageT>
inline RosSubscriberNode<MessageT>::SubscriberInstance::SubscriberInstance(
  rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & topic_name,
  const rclcpp::QoS & qos)
{
  rclcpp::SubscriptionOptions option;
  option.callback_group = group;

  // The callback will broadcast to all the instances of RosSubscriberNode<MessageT>
  auto callback = [this](const std::shared_ptr<MessageT> msg) {
    this->last_msg = msg;
    this->broadcast(msg);
  };
  subscriber = node->create_subscription<MessageT>(topic_name, qos, callback, option);
  name = topic_name;
}

template <class MessageT>
inline void RosSubscriberNode<MessageT>::SubscriberInstance::addCallback(
  const void * callback_owner, const std::function<void(const std::shared_ptr<MessageT>)> & callback)
{
  callbacks.emplace_back(callback_owner, callback);
}

template <class MessageT>
inline void RosSubscriberNode<MessageT>::SubscriberInstance::removeCallback(const void * callback_owner)
{
  callbacks.erase(
    std::remove_if(
      callbacks.begin(), callbacks.end(), [callback_owner](const auto & pair) { return pair.first == callback_owner; }),
    callbacks.end());
}

template <class MessageT>
inline void RosSubscriberNode<MessageT>::SubscriberInstance::broadcast(const std::shared_ptr<MessageT> & msg)
{
  for (auto & callback_pair : callbacks) {
    callback_pair.second(msg);
  }
}

template <class MessageT>
inline RosSubscriberNode<MessageT>::RosSubscriberNode(
  const std::string & instance_name, const Config & config, Context context, const rclcpp::QoS & qos)
: BT::ConditionNode{instance_name, config},
  context_{context},
  logger_(context.getChildLogger(auto_apms_util::toSnakeCase(instance_name))),
  qos_{qos}
{
  // Consider aliasing in ports and copy the values from aliased to original ports
  this->modifyPortsRemapping(context_.copyAliasedPortValuesToOriginalPorts(this));

  if (const BT::Expected<std::string> expected_name = context_.getTopicName(this)) {
    createSubscriber(expected_name.value());
  } else {
    // We assume that determining the topic name requires a blackboard pointer, which cannot be evaluated at
    // construction time. The expression will be evaluated each time before the node is ticked the first time after
    // successful execution.
    dynamic_client_instance_ = true;
  }
}

template <class MessageT>
inline bool RosSubscriberNode<MessageT>::createSubscriber(const std::string & topic_name)
{
  if (topic_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) + " - Argument topic_name is empty when trying to create a client.");
  }

  // Check if the subscriber with given name is already set up
  if (sub_instance_ && topic_name == sub_instance_->name) return true;

  std::unique_lock lk(registryMutex());

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
  subscriber_key_ = std::string(node->get_fully_qualified_name()) + "/" + topic_name;

  auto & registry = getRegistry();
  auto it = registry.find(subscriber_key_);
  if (it == registry.end() || it->second.expired()) {
    sub_instance_ = std::make_shared<SubscriberInstance>(node, group, topic_name, qos_);
    registry.insert({subscriber_key_, sub_instance_});
    RCLCPP_DEBUG(
      logger_, "%s - Created subscriber for topic '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
      topic_name.c_str());
  } else {
    sub_instance_ = it->second.lock();
  }

  // Check if there was a message received before the creation of this subscriber action
  if (sub_instance_->last_msg) {
    last_msg_ = sub_instance_->last_msg;
  }

  // add "this" as received of the broadcaster
  sub_instance_->addCallback(this, [this](const std::shared_ptr<MessageT> msg) { last_msg_ = msg; });

  return true;
}

template <class MessageT>
inline BT::NodeStatus RosSubscriberNode<MessageT>::tick()
{
  if (!rclcpp::ok()) {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  // If client has been set up in derived constructor, event though this constructor couldn't, we discard the intention
  // of dynamically creating the client
  if (dynamic_client_instance_ && sub_instance_) {
    dynamic_client_instance_ = false;
  }

  // Try again to create the client on first tick if this was not possible during construction or if client should be
  // created from a blackboard entry on the start of every iteration
  if (status() == BT::NodeStatus::IDLE && dynamic_client_instance_) {
    const BT::Expected<std::string> expected_name = context_.getTopicName(this);
    if (expected_name) {
      createSubscriber(expected_name.value());
    } else {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) +
        " - Cannot create the subscriber because the topic name couldn't be resolved using "
        "the expression specified by the node's registration parameters (" +
        NodeRegistrationOptions::PARAM_NAME_ROS2TOPIC + ": " + context_.registration_options_.topic +
        "). Error message: " + expected_name.error());
    }
  }

  if (!sub_instance_) {
    throw exceptions::RosNodeError(context_.getFullyQualifiedTreeNodeName(this) + " - sub_instance_ is nullptr.");
  }

  auto check_status = [this](BT::NodeStatus status) {
    if (!isStatusCompleted(status)) {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - The callback must return either SUCCESS or FAILURE.");
    }
    return status;
  };
  auto status = check_status(onTick(last_msg_));
  last_msg_.reset();
  return status;
}

template <class MessageT>
inline BT::NodeStatus RosSubscriberNode<MessageT>::onTick(const std::shared_ptr<MessageT> & last_msg_ptr)
{
  if (!last_msg_ptr) return BT::NodeStatus::FAILURE;
  return onMessageReceived(*last_msg_ptr);
}

template <class MessageT>
inline BT::NodeStatus RosSubscriberNode<MessageT>::onMessageReceived(const MessageT & /*msg*/)
{
  return BT::NodeStatus::SUCCESS;
}

template <class MessageT>
inline std::string RosSubscriberNode<MessageT>::getTopicName() const
{
  if (sub_instance_) return sub_instance_->name;
  return "unknown";
}

template <class MessageT>
inline std::mutex & RosSubscriberNode<MessageT>::registryMutex()
{
  static std::mutex sub_mutex;
  return sub_mutex;
}

template <class MessageT>
inline typename RosSubscriberNode<MessageT>::SubscribersRegistry & RosSubscriberNode<MessageT>::getRegistry()
{
  static SubscribersRegistry subscribers_registry;
  return subscribers_registry;
}

}  // namespace auto_apms_behavior_tree::core