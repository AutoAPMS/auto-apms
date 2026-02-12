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

#include <map>

#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "behaviortree_cpp/basic_types.h"

/// @cond INTERNAL

#define AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_NON_LEAF_THISREF_METHODS(ClassType)                                   \
  ClassType & removeFirstChild(                                                                                   \
    const std::string & registration_name = "", const std::string & instance_name = "", bool deep_search = false) \
  {                                                                                                               \
    NodeElement::removeFirstChild(registration_name, instance_name, deep_search);                                 \
    return *this;                                                                                                 \
  }                                                                                                               \
  template <class T>                                                                                              \
  typename std::enable_if_t<std::is_base_of_v<NodeModelType, T>, ClassType &> removeFirstChild(                   \
    const std::string & instance_name = "", bool deep_search = false)                                             \
  {                                                                                                               \
    NodeElement::removeFirstChild<T>(instance_name, deep_search);                                                 \
    return *this;                                                                                                 \
  }                                                                                                               \
  ClassType & removeChildren()                                                                                    \
  {                                                                                                               \
    NodeElement::removeChildren();                                                                                \
    return *this;                                                                                                 \
  }

#define AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_LEAF_THISREF_METHODS(ClassType)                                      \
  ClassType & setPorts(const auto_apms_behavior_tree::core::TreeDocument::NodeElement::PortValues & port_values) \
  {                                                                                                              \
    NodeElement::setPorts(port_values);                                                                          \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & resetPorts()                                                                                       \
  {                                                                                                              \
    NodeElement::resetPorts();                                                                                   \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & setConditionalScript(BT::PreCond type, const auto_apms_behavior_tree::core::Script & script)       \
  {                                                                                                              \
    NodeElement::setConditionalScript(type, script);                                                             \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & setConditionalScript(BT::PostCond type, const auto_apms_behavior_tree::core::Script & script)      \
  {                                                                                                              \
    NodeElement::setConditionalScript(type, script);                                                             \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & setName(const std::string & instance_name)                                                         \
  {                                                                                                              \
    NodeElement::setName(instance_name);                                                                         \
    return *this;                                                                                                \
  }

/// @endcond

namespace auto_apms_behavior_tree
{
namespace core
{

class TreeBuilder;

/**
 * @brief Abstract base for all automatically generated behavior tree node model classes to be used in interaction with
 * the TreeDocument API.
 */
class NodeModelType : public TreeDocument::NodeElement
{
protected:
  using NodeElement::NodeElement;

public:
  using RegistrationOptions = NodeRegistrationOptions;
  using PortInfos = std::map<std::string, BT::PortInfo>;

  virtual std::string getRegistrationName() const override = 0;

  /**
   * @brief Convert to a lower level NodeElement object.
   * @return Base object representing this node.
   */
  NodeElement toNodeElement();
};

/**
 * @brief Abstract base for automatically generated model classes of behavior tree nodes that are not allowed to have
 * any children.
 */
class LeafNodeModelType : public NodeModelType
{
protected:
  using NodeModelType::NodeModelType;

public:
  /* Not supported methods for LeafNodeModelType instances */

  LeafNodeModelType insertNode() = delete;
  LeafNodeModelType insertSubTreeNode() = delete;
  LeafNodeModelType insertTree() = delete;
  LeafNodeModelType insertTreeFromDocument() = delete;
  LeafNodeModelType insertTreeFromString() = delete;
  LeafNodeModelType insertTreeFromFile() = delete;
  LeafNodeModelType insertTreeFromResource() = delete;
  LeafNodeModelType & removeFirstChild() = delete;
  LeafNodeModelType & removeChildren() = delete;
};

}  // namespace core

/**
 * @brief Models for all available behavior tree nodes.
 *
 * Behavior tree node models are generated under the namespace `my_package_name::model` when specifying the
 * `NODE_MODEL_HEADER_TARGET` argument of the CMake macro `auto_apms_behavior_tree_register_nodes` in the CMakeLists.txt
 * of a package with name `my_package_name`.
 *
 * ## Usage
 *
 * In the CMakeLists.txt of the package call the macro as follows:
 * ```cmake
 * auto_apms_behavior_tree_register_nodes(node_library_target
 *     ... # Fully qualified class names of the node plugins
 *     NODE_MODEL_HEADER_TARGET model_header_target
 * )
 * ```
 * After building the package, there is a header file named `node_library_target.hpp` which may be included by the C++
 * files associated with the target `model_header_target` like this:
 * ```cpp
 * #include "my_package_name/node_library_target.hpp"
 * ```
 *
 * @sa <a
 * href="https://autoapms.github.io/auto-apms-guide/concept/common-resources#behavior-tree-node-models">
 * Common Resources: Behavior Tree Node Models</a>
 */
namespace model
{

/**
 * @brief Subtree behavior tree node model.
 *
 * This model implements extra methods specific to the subtree concept.
 */
class SubTree : public core::LeafNodeModelType
{
  friend class core::TreeDocument::NodeElement;

private:
  using LeafNodeModelType::LeafNodeModelType;

public:
  /// @brief Static method that provides the hard coded registration name of subtree nodes.
  static std::string name();

  /// @brief Type of the behavior tree node.
  static BT::NodeType type();

  /**
   * @brief Get the type specific name under which all subtree nodes are registered with the behavior tree factory.
   * @return Registration name of a subtree node.
   */
  std::string getRegistrationName() const override final;

  AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_LEAF_THISREF_METHODS(SubTree)

  /**
   * @brief Configure which blackboard entries of the subtree node's parent tree should be also available for the
   * children of the tree this subtree node is pointing to.
   * @param remapping Mapping of blackboard entry names in the format {subtree_entry_name: original_tree_entry_name}
   * @return Modified subtree model.
   */
  SubTree & setBlackboardRemapping(const PortValues & remapping);

  /**
   * @brief Set automatic blackboard remapping.
   * @param val `true` to enable and `false` to disable.
   * @return Modified subtree model.
   */
  SubTree & set_auto_remap(bool val = false);

  /**
   * @brief Get automatic blackboard remapping.
   * @return Boolean flag for the currently configured option.
   */
  bool get_auto_remap() const;
};

}  // namespace model
}  // namespace auto_apms_behavior_tree