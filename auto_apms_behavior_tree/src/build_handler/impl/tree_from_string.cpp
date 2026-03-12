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

#include "auto_apms_behavior_tree/build_handler.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Standard build handler for building a behavior tree directly from an XML string.
 *
 * This build handler requires the user to encode the XML of a behavior tree and pass it as the build request.
 * This may be done for example using TreeDocument::writeToString.
 */
class TreeFromStringBuildHandler : public TreeBuildHandler
{
public:
  TreeFromStringBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
  : TreeBuildHandler("tree_from_string", ros_node_ptr, tree_node_loader_ptr),
    working_doc_(TreeDocument::BTCPP_FORMAT_DEFAULT_VERSION, tree_node_loader_ptr)
  {
  }

  bool setBuildRequest(
    const std::string & build_request, const std::string & entry_point,
    const NodeManifest & node_manifest) override final
  {
    // Adopt the root tree if specified
    working_doc_.reset().mergeString(build_request, true);
    working_doc_.registerNodes(node_manifest, false);

    if (const BT::Result res = working_doc_.verify(); !res) {
      RCLCPP_WARN(logger_, "Tree verification failed: %s", res.error().c_str());
      return false;
    }

    // Try to determine root tree name
    if (entry_point.empty()) {
      if (!working_doc_.hasRootTreeName()) {
        RCLCPP_WARN(
          logger_,
          "Cannot determine root tree: You must either encode the root tree within the tree XML or provide a non-empty "
          "name using the entry_point argument.");
        return false;
      }
    } else {
      try {
        entry_point_ = TreeBasedEntryPoint(entry_point);
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "Cannot determine root tree from entry point: %s", e.what());
        return false;
      }
      if (!working_doc_.hasTreeName(entry_point_.root_tree_name)) {
        RCLCPP_WARN(
          logger_, "The specified entry point root tree '%s' does not exist in the provided tree XML.",
          entry_point_.root_tree_name.c_str());
        return false;
      }
      working_doc_.setRootTreeName(entry_point_.root_tree_name);
    }

    return true;
  }

  TreeDocument::TreeElement buildTree(TreeDocument & doc, TreeBlackboard & bb) override final
  {
    // Merge document and adopt root tree
    doc.mergeTreeDocument(working_doc_, true);

    // Reset the local tree document, as the tree moved to doc
    working_doc_.reset();

    // Set initial blackboard values if specified
    for (const auto & [key, value] : entry_point_.inital_blackboard) {
      bb.set(key, value);
    }

    // The document MUST have a root tree. We made sure of that during setBuildRequest
    return doc.getRootTree();
  }

private:
  TreeBasedEntryPoint entry_point_;
  core::TreeDocument working_doc_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(auto_apms_behavior_tree::TreeFromStringBuildHandler)