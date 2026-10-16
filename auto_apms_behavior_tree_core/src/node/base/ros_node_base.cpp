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

#include "auto_apms_behavior_tree_core/node/base/ros_node_base.hpp"

#include "auto_apms_util/string.hpp"

namespace auto_apms_behavior_tree::core
{

RosNodeBase::RosNodeBase(const std::string & instance_name, Context context)
: context_(context), logger_(context.getChildLogger(auto_apms_util::toSnakeCase(instance_name)))
{
}

void RosNodeBase::applyPortAliasing(const BT::TreeNode * node) const
{
  context_.copyAliasedPortValuesToOriginalPorts(node);
}

}  // namespace auto_apms_behavior_tree::core
