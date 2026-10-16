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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/logger.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Common state and node-manifest support shared by the ROS-aware behavior tree node base classes.
 *
 * This is a plain (non-`BT::TreeNode`) mixin that holds the RosNodeContext and a per-instance logger. It exists so the
 * ROS-aware wrappers around the basic BehaviorTree.CPP node types (RosSyncActionNode, RosStatefulActionNode,
 * RosConditionNode, RosDecoratorNode, RosControlNode) share identical construction logic instead of duplicating it.
 *
 * Applying the node-manifest port aliasing is triggered by the derived node itself via applyPortAliasing(); the
 * wrappers call it from their constructor once the node is fully constructed.
 *
 * It additionally provides getSharedEntity(), the shared-entity registry used by the provided ROS node bases
 * (RosPublisherNode, RosSubscriberNode, RosServiceNode, RosActionNode) and available to any downstream node so that
 * equivalent ROS 2 entities (publishers, subscriptions, clients, ...) are created once and shared instead of being
 * duplicated per tree node instance.
 */
class RosNodeBase
{
public:
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

protected:
  RosNodeBase(const std::string & instance_name, Context context);

  ~RosNodeBase() = default;

  /**
   * @brief Support the node manifest 'port_alias' feature: copy the aliased port values onto the original ports the
   * node implementation reads.
   *
   * Must be called by the derived (BT::TreeNode) node from its constructor.
   *
   * @param node The tree node instance (usually `this`).
   */
  void applyPortAliasing(const BT::TreeNode * node) const;

  /**
   * @brief Retrieve a process-wide shared ROS 2 entity, creating it via @p factory on first use.
   *
   * Behavior tree nodes are frequently instantiated multiple times (e.g. the same topic used by several nodes in a
   * tree). To avoid creating one ROS 2 entity - publisher, subscription, service/action client, or any wrapper thereof
   * - per node instance, entities are shared through a registry keyed by the owning ROS 2 node's fully qualified name
   * and @p entity_name. The first caller for a given key creates the instance via @p factory; subsequent callers
   * receive the same instance for as long as it is still held by at least one node. Once the last owner is destroyed,
   * the entry expires and the next caller creates a fresh instance.
   *
   * A distinct registry is kept per @p InstanceT, so different entity types never collide even if they share a name.
   *
   * @tparam InstanceT Type wrapping (or being) the shared ROS 2 entity.
   * @param entity_name Name identifying the entity (e.g. the topic/service/action name) used to form the registry key.
   * @param factory Callable creating a new instance; only invoked when no live instance exists for the key.
   * @return Shared pointer to the (possibly newly created) shared instance.
   */
  template <typename InstanceT>
  std::shared_ptr<InstanceT> getSharedEntity(
    const std::string & entity_name, const std::function<std::shared_ptr<InstanceT>()> & factory);

  const Context context_;
  const rclcpp::Logger logger_;
};

template <typename InstanceT>
std::shared_ptr<InstanceT> RosNodeBase::getSharedEntity(
  const std::string & entity_name, const std::function<std::shared_ptr<InstanceT>()> & factory)
{
  // A separate registry (and guarding mutex) is instantiated for each InstanceT. Being function-local statics of a
  // template, they have vague linkage and are therefore merged into a single instance process-wide across all
  // translation units.
  static std::mutex mutex;
  static std::unordered_map<std::string, std::weak_ptr<InstanceT>> registry;

  // Key by the owning ROS 2 node so that entities belonging to different nodes never collide.
  const std::string key = context_.getFullyQualifiedRosNodeName() + "/" + entity_name;

  std::unique_lock lock(mutex);
  if (const auto it = registry.find(key); it != registry.end()) {
    if (const std::shared_ptr<InstanceT> existing = it->second.lock()) return existing;
  }
  const std::shared_ptr<InstanceT> instance = factory();
  registry[key] = instance;
  return instance;
}

}  // namespace auto_apms_behavior_tree::core
