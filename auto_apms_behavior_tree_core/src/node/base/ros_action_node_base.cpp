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

#include "auto_apms_behavior_tree_core/node/base/ros_action_node_base.hpp"

namespace auto_apms_behavior_tree::core
{

RosActionNodeBase::RosActionNodeBase(const std::string & instance_name, const Config & config, Context context)
: BT::ActionNodeBase(instance_name, config), RosNodeBase(instance_name, context)
{
  // Support the node manifest 'port_alias' feature. Applied here (before the derived constructor body runs) so the
  // aliased ports are already in effect for any port access the derived node performs during construction.
  applyPortAliasing(this);
}

BT::PortsList RosActionNodeBase::providedPorts() { return {}; }

}  // namespace auto_apms_behavior_tree::core
