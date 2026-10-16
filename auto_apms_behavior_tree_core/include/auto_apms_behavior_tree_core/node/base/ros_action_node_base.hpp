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
#include "behaviortree_cpp/action_node.h"

namespace auto_apms_behavior_tree::core
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief ROS-aware wrapper for the basic `BT::ActionNodeBase` (generic asynchronous action).
 *
 * This is the most generic of the ROS-aware action bases: it adds the RosNodeContext, a per-instance logger and
 * node-manifest support to the plain `BT::ActionNodeBase` without imposing any particular execution structure. Unlike
 * RosStatefulActionNode (which supplies the `onStart()`/`onRunning()`/`onHalted()` state machine), the derived node
 * implements `tick()` and `halt()` directly and is therefore free to run its own state machine - useful when the
 * built-in stateful flow doesn't fit, e.g. a node that publishes a command and then waits for an acknowledgement while
 * managing the transition between those phases itself.
 *
 * The dedicated communication bases RosServiceNode and RosActionNode build on this wrapper too.
 *
 * Implement `tick()` and `halt()` as usual for a `BT::ActionNodeBase`. Use `status()`/`setStatus()`/`resetStatus()` to
 * track the phase (IDLE on the first tick, RUNNING while awaiting completion).
 */
class RosActionNodeBase : public BT::ActionNodeBase, public RosNodeBase
{
public:
  RosActionNodeBase(const std::string & instance_name, const Config & config, Context context);

  static BT::PortsList providedPorts();
};

}  // namespace auto_apms_behavior_tree::core
