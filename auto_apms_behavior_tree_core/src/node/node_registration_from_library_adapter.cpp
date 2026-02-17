// Copyright 2026 Robin Müller
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

#include "auto_apms_behavior_tree_core/node/node_registration_from_library_adapter.hpp"

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "behaviortree_cpp/utils/shared_library.h"

namespace auto_apms_behavior_tree::core
{

bool NodeRegistrationFromLibraryAdapter::requiresRosNodeContext() const { return false; }

void NodeRegistrationFromLibraryAdapter::registerWithBehaviorTreeFactory(
  BT::BehaviorTreeFactory & factory, const std::string & name,
  const auto_apms_behavior_tree::core::RosNodeContext * const /*context_ptr*/) const
{
  const std::string lib_path = getPluginLibraryPath();

  // Load the library in a temporary factory and extract the builder for the given node registration name to seperate it
  // from other nodes that are registered in the same plugin library.
  BT::BehaviorTreeFactory temp_factory;
  try {
    temp_factory.registerFromPlugin(lib_path);
  } catch (const std::exception & e) {
    throw exceptions::NodeRegistrationError(
      "Failed to load plugin library '" + lib_path + "' for node registration name '" + name + "': " + e.what());
  }

  // Extract the builder for the given node registration name and register it in the provided factory.
  const auto & manifests = temp_factory.manifests();
  const auto manifest_it = manifests.find(name);
  if (manifest_it == manifests.end()) {
    throw exceptions::NodeRegistrationError(
      "Node registration name '" + name + "' not found in plugin library '" + lib_path + "'.");
  }
  factory.registerBuilder(manifest_it->second, temp_factory.builders().at(name));
}

}  // namespace auto_apms_behavior_tree::core