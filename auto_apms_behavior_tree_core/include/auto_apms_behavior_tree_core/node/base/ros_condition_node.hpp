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

#include <string>

#include "auto_apms_behavior_tree_core/node/base/ros_node_base.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace auto_apms_behavior_tree::core
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief ROS-aware wrapper for the basic `BT::ConditionNode`.
 *
 * Adds the RosNodeContext, a per-instance logger and node-manifest support to the plain condition node. Derive from it
 * for a synchronous condition that needs access to the ROS 2 node (via RosNodeContext::getRosNode()) but doesn't fit
 * one of the dedicated communication bases (e.g. RosSubscriberNode for topic-based conditions).
 *
 * Implement `tick()` (returning SUCCESS/FAILURE) as usual for a `BT::ConditionNode`.
 */
class RosConditionNode : public BT::ConditionNode, public RosNodeBase
{
public:
  RosConditionNode(const std::string & instance_name, const Config & config, Context context);

  static BT::PortsList providedPorts();
};

}  // namespace auto_apms_behavior_tree::core
