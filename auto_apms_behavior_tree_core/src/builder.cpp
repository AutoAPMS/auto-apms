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

#include "auto_apms_behavior_tree_core/builder.hpp"

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/xml_parsing.h"

namespace auto_apms_behavior_tree::core
{

TreeBuilder::TreeBuilder(
  rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group,
  rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
  NodeRegistrationLoader::SharedPtr tree_node_loader)
: TreeDocument(BTCPP_FORMAT_DEFAULT_VERSION, tree_node_loader)
{
  if (ros_node) logger_ = ros_node->get_logger().get_child(LOGGER_NAME);
  ros_node_wptr_ = ros_node;
  tree_node_waitables_callback_group_wptr_ = tree_node_waitables_callback_group;
  tree_node_waitables_executor_wptr_ = tree_node_waitables_executor;
  only_non_ros_nodes_ = false;
}

TreeBuilder::TreeBuilder(NodeRegistrationLoader::SharedPtr tree_node_loader)
: TreeBuilder(
    std::shared_ptr<rclcpp::Node>(), std::shared_ptr<rclcpp::CallbackGroup>(),
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>(), tree_node_loader)
{
  only_non_ros_nodes_ = true;
}

TreeBuilder & TreeBuilder::registerNodes(const NodeManifest & tree_node_manifest, bool override)
{
  std::vector<std::string> provided_names;
  std::vector<std::string> already_registered_names;
  std::vector<std::string> new_names;
  for (const auto & [name, _] : tree_node_manifest.map()) provided_names.push_back(name);
  for (const auto & [name, _] : factory_.builders()) already_registered_names.push_back(name);
  already_registered_names = auto_apms_util::getCommonElements(provided_names, already_registered_names);
  for (const std::string & name : provided_names) {
    if (!auto_apms_util::contains(already_registered_names, name)) new_names.push_back(name);
  }
  RCLCPP_DEBUG(
    logger_, "Registering nodes with TreeBuilder\n-> Already registered node names: [ %s ]\n-> New node names: [ %s ]",
    auto_apms_util::join(already_registered_names, ", ").c_str(), auto_apms_util::join(new_names, ", ").c_str());
  TreeDocument::registerNodes(tree_node_manifest, override);
  return *this;
}

TreeBuilder & TreeBuilder::setScriptingEnum(const std::string & enum_name, int val)
{
  factory_.registerScriptingEnum(enum_name, val);
  return *this;
}

Tree TreeBuilder::instantiate(const TreeElement & tree_ele, TreeBlackboardSharedPtr bb_ptr)
{
  const std::string tree_name = tree_ele.getName();
  if (hasTreeName(tree_name) && getTree(tree_name) == tree_ele) {
    // If tree already exists and the given element points to the same tree, we can directly instantiate it
    return instantiate(tree_name, bb_ptr);
  }
  // If the given tree element is not part of this document, we throw (users must use the associated builder for
  // instantiating it)
  throw exceptions::TreeBuildError(
    "Cannot instantiate tree with name '" + tree_name +
    "' because it doesn't belong to this TreeBuilder. Make sure to merge the tree before trying to instantiate it.");
}

Tree TreeBuilder::instantiate(const std::string & root_tree_name, TreeBlackboardSharedPtr bb_ptr)
{
  if (!hasTreeName(root_tree_name)) {
    throw exceptions::TreeDocumentError(
      "Cannot instantiate tree with name '" + root_tree_name + "' because it doesn't exist.");
  }
  Tree tree;
  try {
    factory_.registerBehaviorTreeFromText(writeToString());
    tree = factory_.createTree(root_tree_name, bb_ptr);
    factory_.clearRegisteredBehaviorTrees();
  } catch (const std::exception & e) {
    throw exceptions::TreeBuildError("Error during TreeBuilder::instantiate(): " + std::string(e.what()));
  }
  return tree;
}

Tree TreeBuilder::instantiate(TreeBlackboardSharedPtr bb_ptr)
{
  if (hasRootTreeName()) return instantiate(getRootTreeName(), bb_ptr);
  throw exceptions::TreeBuildError(
    "Cannot instantiate tree without a root tree name. You must either specify the attribute '" +
    std::string(TreeDocument::ROOT_TREE_ATTRIBUTE_NAME) +
    "' of the tree document's root element or call TreeDocument::setRootTreeName() to define the root tree. "
    "Alternatively, you may call a different signature of TreeBuilder::instantiate().");
}

}  // namespace auto_apms_behavior_tree::core