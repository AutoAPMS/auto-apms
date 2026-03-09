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

#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Interface used for registering behavior tree node plugins.
 * @sa NodeRegistrationTemplate
 * @sa AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE
 */
class NodeRegistrationInterface
{
public:
  NodeRegistrationInterface() = default;
  virtual ~NodeRegistrationInterface() = default;

  /**
   * @brief Determine whether a valid instance of RosNodeContext is required for registering the behavior tree node
   * associated with this instance.
   *
   * This method effectively allows to observe whether the underlying behavior tree node is a ROS 2 specific node or
   * not.
   * @return `true` if you must pass a RosNodeContext instance to registerWithBehaviorTreeFactory, `false` otherwise.
   */
  virtual bool requiresRosNodeContext() const = 0;

  /**
   * @brief Register the node associated with this instance with a behavior tree factory.
   * @param factory Behavior tree factory instance.
   * @param registration_name Type name under which this node is to be registered.
   * @param context_ptr Optional pointer to a RosNodeContext required if the node to be registered is ROS 2 specific.
   * @throw std::invalid_argument if node to be registered is ROS 2 specific but @p context_ptr was omitted.
   */
  virtual void registerWithBehaviorTreeFactory(
    BT::BehaviorTreeFactory & factory, const std::string & registration_name,
    const RosNodeContext * const context_ptr = nullptr) const = 0;
};

}  // namespace auto_apms_behavior_tree::core
