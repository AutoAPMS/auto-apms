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

#include <vector>

#include "auto_apms_behavior_tree_core/node/node_registration_interface.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief A node registration interface that loads and registers behavior tree nodes from one or more plugin
 * libraries using `BT::BehaviorTreeFactory::registerFromPlugin`.
 *
 * The libraries must export the `BT_RegisterNodesFromPlugin(factory)` symbol which is the ROS-agnostic approach of
 * distributing behavior tree nodes through plugins in BehaviorTree.CPP.
 *
 * You can find an example of how to implement the required plugin export macro in the official BehaviorTree.CPP
 * repository:
 * https://github.com/BehaviorTree/BehaviorTree.CPP/blob/3ff6a32ba0497a08519c77a1436e3b81eff1bcd6/examples/plugin_example/plugin_action.cpp
 *
 * **Usage:**
 *
 * 1. Create a subclass of this adapter for each plugin library you want to load nodes from and implement the
 *    NodeRegistrationFromLibraryAdapter::getPluginLibraryPath() method to return the correct library path.
 *
 * 2. In your node manifest YAML, specify the fully qualified name of the adapter subclass as the class name for the
 *    nodes you want to register from the plugin library. You can specify the same adapter subclass multiple times with
 *    different registration names to register multiple nodes from the same library.
 */
class NodeRegistrationFromLibraryAdapter : public NodeRegistrationInterface
{
public:
  NodeRegistrationFromLibraryAdapter() = default;
  ~NodeRegistrationFromLibraryAdapter() override = default;

  bool requiresRosNodeContext() const override;

  void registerWithBehaviorTreeFactory(
    BT::BehaviorTreeFactory & factory, const std::string & name,
    const auto_apms_behavior_tree::core::RosNodeContext * const context_ptr = nullptr) const override;

private:
  /**
   * @brief Get the library path that exports the node registration function associated with this class.
   * @return Library path.
   */
  virtual std::string getPluginLibraryPath() const = 0;
};

}  // namespace auto_apms_behavior_tree::core