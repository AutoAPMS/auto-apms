// Copyright 2026 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <set>
#include <string>

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

/**
 * @brief A testable TreeDocument that allows directly adding nodes to the internal manifest
 * without going through the plugin loader validation.
 */
class TestableTreeDocument : public auto_apms_behavior_tree::core::TreeDocument
{
public:
  using TreeDocument::TreeDocument;

  /**
   * @brief Add a test node to the internal manifest and factory.
   * This bypasses the plugin loader check that registerNodes performs.
   */
  TestableTreeDocument & addTestNode(const std::string & node_name, const std::string & class_name = "test::TestClass")
  {
    // Add to internal manifest (accessible since it's now protected)
    auto_apms_behavior_tree::core::NodeManifest::RegistrationOptions opts;
    opts.class_name = class_name;
    registered_nodes_manifest_.add(node_name, opts);

    // Also register with factory so the document can work with these nodes
    if (factory_.builtinNodes().count(node_name) == 0) {
      factory_.registerSimpleAction(node_name, [](BT::TreeNode &) { return BT::NodeStatus::SUCCESS; });
    }

    return *this;
  }

  /**
   * @brief Get the set of registered node names (non-native only)
   */
  std::set<std::string> getTestNodeNames() const { return getRegisteredNodeNames(false); }
};
