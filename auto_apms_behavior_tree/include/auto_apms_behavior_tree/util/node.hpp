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

#include "auto_apms_behavior_tree/behavior_tree_nodes.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @ingroup auto_apms_behavior_tree
 * @{
 */

/**
 * @brief Helper that extends the children of a given parent node by a `StartExecutor` node for a specific behavior tree
 * to be created from string.
 * @param parent Parent behavior tree node.
 * @param tree_str XML string of the behavior tree to be executed.
 * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
 * insert at the end.
 * @return Inserted `StartExecutor` node element.
 * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of @p
 * parent.
 */
model::StartExecutor insertStartExecutorFromString(
  core::TreeDocument::NodeElement & parent, const std::string & tree_str,
  const core::TreeDocument::NodeElement * before_this = nullptr);

/**
 * @brief Helper that extends the children of a given parent node by a `StartExecutor` node for a specific behavior tree
 * to be created from a string.
 * @param parent Parent behavior tree node.
 * @param tree Tree element representing the behavior tree to be executed.
 * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
 * insert at the end.
 * @return Inserted `StartExecutor` node element.
 * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of @p
 * parent.
 */
model::StartExecutor insertStartExecutorFromString(
  core::TreeDocument::NodeElement & parent, const core::TreeDocument::TreeElement & tree,
  const core::TreeDocument::NodeElement * before_this = nullptr);

/**
 * @brief Helper that extends the children of a given parent node by a `StartExecutor` node for a specific behavior tree
 * to be created from a resource.
 * @param parent Parent behavior tree node.
 * @param identity Identity object for the behavior tree resource that holds the tree to be executed. You may explicitly
 * set the root tree by specifying the `<tree_name>` token of the identity.
 * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
 * insert at the end.
 * @return Inserted `StartExecutor` node element.
 * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of @p
 * parent.
 */
model::StartExecutor insertStartExecutorFromResource(
  core::TreeDocument::NodeElement & parent, const core::TreeResourceIdentity & identity,
  const core::TreeDocument::NodeElement * before_this = nullptr);

/**
 * @}
 */

}  // namespace auto_apms_behavior_tree