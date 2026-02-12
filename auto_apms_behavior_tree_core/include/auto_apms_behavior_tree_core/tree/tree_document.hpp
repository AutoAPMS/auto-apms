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

#include <tinyxml2.h>

#include <functional>
#include <iterator>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "auto_apms_behavior_tree_core/tree/script.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree::model
{
class SubTree;
}

namespace auto_apms_behavior_tree::core
{

class TreeResource;
class NodeModelType;

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Document Object Model (DOM) for the behavior tree XML schema. This class offers a programmatic approach for
 * building behavior trees and stores the registration data of all associated nodes.
 *
 * A single tree document may contain multiple behavior trees (represented by TreeElement). Each behavior tree may
 * contain an arbitrary amount of tree nodes (represented by NodeElement). There are various different types of nodes.
 * For each one of the [standard
 * nodes](https://autoapms.github.io/auto-apms-guide/reference/behavior-tree-nodes),
 * there is a model available under the `auto_apms_behavior_tree::model` namespace. Refer to the
 * [BehaviorTree.CPP](https://www.behaviortree.dev/docs/learn-the-basics/BT_basics) website for more infos on the basic
 * concepts of behavior trees used in this implementation.
 *
 * A TreeDocument instance must be kept alive for as long as its behavior trees are accessed in any way. When a document
 * instance is destroyed, the memory of the associated behavior trees is deleted. Hence, a TreeDocument acts the same as
 * a `tinyxml2::XMLDocument`.
 *
 * Copy assignment is not supported to enforce the builder design pattern. To copy certain
 * behavior trees from one document to another, the user must explicitly invoke TreeDocument::mergeTreeDocument of
 * another document instance.
 *
 * The XML schema of a tree document is defined [here](https://www.behaviortree.dev/docs/learn-the-basics/xml_format)
 * and looks similar to this:
 *
 * ```xml
 * <root BTCPP_format="4" main_tree_to_execute="RootTree">
 *   <!-- Each TreeDocument may have zero or more behavior tree elements -->
 *   <BehaviorTree ID="RootTree">
 *     <!-- Each behavior tree element has exactly one child. This child may again have zero or more children -->
 *   </BehaviorTree>
 *   <BehaviorTree ID="AnotherTree">
 *     <!-- ... -->
 *   </BehaviorTree>
 * </root>
 * ```
 *
 * @note A tree document always has a single root element that holds the format version and the name of the root tree
 * acting as an entry point for execution.
 *
 * The content of the document can be configured as desired using its member functions. For example like this:
 *
 *  ```cpp
 * #include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
 *
 * // Bring the behavior tree API into scope.
 * using namespace auto_apms_behavior_tree;
 *
 * // Initially, the document is created. It must be kept alive throughout this scope.
 * core::TreeDocument doc;
 *
 * // You may hold on to the returned tree and node elements (they're only valid as long as the document is).
 * core::TreeDocument::TreeElement root_tree = doc.newTree("RootTree");
 * core::TreeDocument::NodeElement sequence = root_tree.insertNode("Sequence");
 * // You may add more nodes to the sequence node created above.
 * // ...
 *
 * // Method chaining is supported as well.
 * doc.newTree("AnotherTree").insertNode("AlwaysSuccess");
 * // "AnotherTree" has an action node as first child. Action nodes act as tree leaves, meaning they cannot have
 * // children themselves.
 * ```
 *
 * Alternatively (and preferably), the user may profit from static type validation and more convenience methods by
 * incorporating node models as template arguments:
 *
 * ```cpp
 * #include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
 * #include "auto_apms_behavior_tree/behavior_tree_nodes.hpp"  // This includes the node models
 *
 * // Bring the behavior tree API into scope.
 * using namespace auto_apms_behavior_tree;
 *
 * // Initially, the document is created. It must be kept alive throughout this scope.
 * core::TreeDocument doc;
 *
 * // You may hold on to the returned tree and node elements (they're only valid as long as the document is).
 * core::TreeDocument::TreeElement root_tree = doc.newTree("RootTree");
 * model::Sequence sequence = root_tree.insertNode<model::Sequence>();
 * // You may add more nodes to the sequence node created above.
 * // ...
 *
 * // Method chaining is supported as well.
 * doc.newTree("AnotherTree").insertNode<model::AlwaysSuccess>();
 * // "AnotherTree" has an action node as first child. Action nodes act as tree leaves, meaning they cannot have
 * // children themselves. This is enforced by the compiler when using a node model as a template argument.
 * ```
 * The former approach can be achieved using the API offered by the `auto_apms_behavior_tree_core` package, while the
 * latter requires to build and link against the `%auto_apms_behavior_tree` package, since the node models are generated
 * automatically by that package.
 *
 * @sa <a
 * href="https://autoapms.github.io/auto-apms-guide/tutorial/building-behavior-trees#using-treedocument">
 * Tutorial: Building Behavior Trees Programmatically</a>
 */
class TreeDocument : private tinyxml2::XMLDocument
{
  using XMLElement = tinyxml2::XMLElement;

  inline static const std::string LOGGER_NAME = "tree_document";

public:
  static inline const char BTCPP_FORMAT_ATTRIBUTE_NAME[] = "BTCPP_format";
  static inline const char BTCPP_FORMAT_DEFAULT_VERSION[] = "4";
  static inline const char ROOT_ELEMENT_NAME[] = "root";
  static inline const char ROOT_TREE_ATTRIBUTE_NAME[] = "main_tree_to_execute";
  static inline const char TREE_ELEMENT_NAME[] = "BehaviorTree";
  static inline const char SUBTREE_ELEMENT_NAME[] = "SubTree";
  static inline const char TREE_NAME_ATTRIBUTE_NAME[] = "ID";
  static inline const char TREE_NODE_MODEL_ELEMENT_NAME[] = "TreeNodesModel";
  static inline const char NODE_INSTANCE_NAME_ATTRIBUTE_NAME[] = "name";
  static inline const char INCLUDE_ELEMENT_NAME[] = "include";
  static inline const char INCLUDE_PATH_ATTRIBUTE_NAME[] = "path";
  static inline const char INCLUDE_ROS_PKG_ATTRIBUTE_NAME[] = "ros_pkg";

  class TreeElement;

  /**
   * @brief Handle for a single node of a TreeDocument.
   *
   * New node elements can only be created by other existing NodeElement instances. Therefore, before being able to
   * insert nodes to a tree, the user must create a tree element using TreeDocument::getTree, TreeDocument::newTree or
   * the other **newTree** methods. Once a tree has been created, the returned TreeElement acts as the entry point for
   * adding nodes.
   *
   * Once created, the NodeElement instance acts as a handle for the node data encoded in a tree document. Therefore, it
   * is not strictly necessary to hold on to the object. The required information is stored as part of the original
   * TreeDocument and can be queried as long as the document object exists.
   */
  class NodeElement
  {
  public:
    /// Mapping of port names and its respective value encoded as string.
    using PortValues = std::map<std::string, std::string>;
    /// Callback invoked for every node found under another node. It may modify the current node.
    using DeepApplyCallback = std::function<bool(NodeElement &)>;
    /// Callback invoked for every node found under another node. It cannot modify the current node.
    using ConstDeepApplyCallback = std::function<bool(const NodeElement &)>;

  protected:
    /**
     * @brief Protected constructor intended for internal use only.
     * @param doc_ptr Pointer to the tree document that created the tree this node belongs to.
     * @param ele_ptr Pointer to the corresponding `XMLElement` of the base document.
     */
    explicit NodeElement(TreeDocument * doc_ptr, XMLElement * ele_ptr);

  public:
    /**
     * @brief Copy constructor for a node element.
     * @param ele Other node element.
     */
    NodeElement(const NodeElement & ele) = default;

    /**
     * @brief Replace this node with another.
     *
     * The other node is copied together with all of its potential child nodes. It **does not** necessarily have
     * to belong to this node's parent document. The assignment operator is only supported for the same static types. So
     * when using node models, the user cannot replace this node with one that is registered under a different name,
     * since different nodes are represented by different static types. However, when using the lower level NodeElement
     * objects, it's possible to use this operator interchangeably with different nodes. The latter approach therefore
     * enables to replace for example a `AlwaysSuccess` node with a `Sequence` node that has already been assigned some
     * children.
     */
    NodeElement & operator=(const NodeElement & other);

    /**
     * @brief Determine if two node elements refer to the same node.
     *
     * The comparison operator returns `true` if two NodeElement objects refer to the same node inside a particular
     * tree. Additionally, the node they refer to must belong to the same document. So even if one of the compared
     * elements points to a similar node inside a tree with the same composition, but this tree belongs to a different
     * document, this operator returns `false`.
     */
    bool operator==(const NodeElement & other) const;

    bool operator!=(const NodeElement & other) const;

    /**
     * @brief Add a new node to the children of this node.
     *
     * The given node can only be inserted if it's specified inside the internal node manifest (see also
     * TreeDocument::registerNodes).
     * @param name Registration name of the node to be inserted.
     * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
     * insert at the end.
     * @return Inserted node element.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the node is not available with the current node
     * manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertNode(const std::string & name, const NodeElement * before_this = nullptr);

    /**
     * @brief Register a new node and add it to the children of this node.
     *
     * This method automatically tries to register the node with the given @p registration_options.
     * @param name Registration name of the node to be inserted.
     * @param registration_options Configuration parameters required for registering the node.
     * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
     * insert at the end.
     * @return Inserted node element.
     * @throw auto_apms_behavior_tree::exceptions::NodeRegistrationError if node registration fails.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertNode(
      const std::string & name, const NodeRegistrationOptions & registration_options,
      const NodeElement * before_this = nullptr);

    /**
     * @brief Register a new node and add it to the children of this node.
     *
     * This method automatically queries the name and the registration options of the node using the model provided as a
     * template argument.
     * @tparam T Node model type.
     * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
     * insert at the end.
     * @return Node model of the inserted element.
     * @throw auto_apms_behavior_tree::exceptions::NodeRegistrationError if node registration fails.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    template <class T>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, T> && !std::is_same_v<model::SubTree, T>, T> insertNode(
      const NodeElement * before_this = nullptr);

    /**
     * @brief Add a subtree node for a specific tree element to the children of this node.
     *
     * This method is a convenience overload for NodeElement::insertSubTreeNode enabled only for the subtree node model.
     * @tparam SubTreeT Subtree model type.
     * @param tree_name Name of the tree which the subtree node should point to. The parent document must have a tree
     * element with that name.
     * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
     * insert at the end.
     * @return Model of the inserted subtree node.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no tree with name @p tree_name exists inside the
     * document this node belongs to.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    template <class SubTreeT>
    typename std::enable_if_t<std::is_same_v<model::SubTree, SubTreeT>, model::SubTree> insertNode(
      const std::string & tree_name, const NodeElement * before_this = nullptr);

    /**
     * @brief Add a subtree node for a specific tree element to the children of this node.
     *
     * The given tree referred to by @p tree will be copied to the document this node belongs to, if it doesn't exist
     * already.
     * This method is a convenience overload for NodeElement::insertSubTreeNode enabled only for the subtree
     * node model.
     * @tparam SubTreeT Subtree model type.
     * @param tree Tree element which the subtree node should point to.
     * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
     * insert at the end.
     * @return Model of the inserted subtree node.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree refers to an element of another document
     * (which means it must be copied), but a tree with its name already exists inside the document this node belongs
     * to.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    template <class SubTreeT>
    typename std::enable_if_t<std::is_same_v<model::SubTree, SubTreeT>, model::SubTree> insertNode(
      const TreeElement & tree, const NodeElement * before_this = nullptr);

    /**
     * @brief Add a subtree node for a specific tree element to the children of this node.
     * @param tree_name Name of the tree which the subtree node should point to. The parent document must have a tree
     * element with that name.
     * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
     * insert at the end.
     * @return Model of the inserted subtree node.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no tree with name @p tree_name exists inside the
     * document this node belongs to.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    model::SubTree insertSubTreeNode(const std::string & tree_name, const NodeElement * before_this = nullptr);

    /**
     * @brief Add a subtree node for a specific tree element to the children of this node.
     *
     * The given tree referred to by @p tree will be copied to the document this node belongs to, if it doesn't exist
     * already.
     * @param tree Tree element which the subtree node should point to.
     * @param before_this Pointer to an existing child node before which the new node will be placed. If `nullptr`,
     * insert at the end.
     * @return Model of the inserted subtree node.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree refers to an element of another document
     * (which means it must be copied), but a tree with its name already exists inside the document this node belongs
     * to.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    model::SubTree insertSubTreeNode(const TreeElement & tree, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate a tree and add its first child node to the children of this node.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children of the inserted node are
     * preserved. Therefore, this method effectively concatenates @p tree and copies all its nodes to the tree this node
     * belongs to.
     * @param tree Tree element to be concatenated.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree contains any nodes that are not
     * available with the currently configured node manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTree(const TreeElement & tree, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate a tree from a document and add its first child node to the children of this node.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children of the inserted node are
     * preserved. Therefore, this method effectively concatenates another tree and copies all its nodes to the tree this
     * node belongs to.
     * @param doc Tree document that holds the tree to be inserted.
     * @param tree_name Name of the tree to be inserted.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name cannot be found in @p doc.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree contains any nodes that are not
     * available with the currently configured node manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromDocument(
      const TreeDocument & doc, const std::string & tree_name, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate the root tree of a document and add its first child node to the children of this node.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children of the inserted node are
     * preserved. Therefore, this method effectively concatenates another tree and copies all its nodes to the tree this
     * node belongs to.
     * @param doc Tree document that holds the tree to be inserted. It must specify a root tree or have a single tree
     * only.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p doc doesn't specify which tree is the root
     * tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree contains any nodes that are not
     * available with the currently configured node manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromDocument(const TreeDocument & doc, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate a tree from a document created from a string and add its first child node to the children
     * of this node.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children of the inserted node are
     * preserved. Therefore, this method effectively concatenates another tree and copies all its nodes to the tree this
     * node belongs to.
     * @param tree_str String specifying the XML of a tree document that holds the tree to be inserted.
     * @param tree_name Name of the tree to be inserted.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name cannot be found in the document
     * created from @p tree_str.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree contains any nodes that are not
     * available with the currently configured node manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromString(
      const std::string & tree_str, const std::string & tree_name, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate the root tree of a document created from a string and add its first child node to the
     * children of this node.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children of the inserted node are
     * preserved. Therefore, this method effectively concatenates another tree and copies all its nodes to the tree this
     * node belongs to.
     * @param tree_str String specifying the XML of a tree document that holds the tree to be inserted. It must specify
     * a root tree or have a single tree only.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the XML in @p tree_str doesn't specify which
     * tree is the root tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree contains any nodes that are not
     * available with the currently configured node manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromString(const std::string & tree_str, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate a tree from a document created from a file and add its first child node to the children of
     * this node.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children of the inserted node are
     * preserved. Therefore, this method effectively concatenates another tree and copies all its nodes to the tree this
     * node belongs to.
     * @param path Absolute path to an XML file specifying a tree document that holds the tree to be inserted.
     * @param tree_name Name of the tree to be inserted.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name cannot be found in the document
     * created using @p path.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree contains any nodes that are not
     * available with the currently configured node manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromFile(
      const std::string & path, const std::string & tree_name, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate the root tree of a document created from a file and add its first child node to the
     * children of this node.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children of the inserted node are
     * preserved. Therefore, this method effectively concatenates another tree and copies all its nodes to the tree this
     * node belongs to.
     * @param path Absolute path to an XML file specifying a tree document that holds the tree to be inserted. It must
     * specify a root tree or have a single tree only.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the XML in @p path doesn't specify which tree is
     * the root tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree contains any nodes that are not
     * available with the currently configured node manifest.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromFile(const std::string & path, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate a tree from one of the installed package's behavior tree resources and add its first child
     * node to the children of this node.
     *
     * Behavior tree resources are registered by calling the CMake macro `auto_apms_behavior_tree_register_trees` in the
     * CMakeLists.txt of a package. They can be inserted once the corresponding package has been installed to the ROS 2
     * workspace.
     *
     * Additionally, this method automatically extends the internal node manifest with the nodes associated with the
     * tree resource using the `NODE_MANIFEST` argument of `auto_apms_behavior_tree_register_trees`.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children
     * of the inserted node are preserved. Therefore, this method effectively concatenates another tree and copies all
     * its nodes to the tree this node belongs to.
     *
     * @sa TreeResourceIdentity for more information about how to refer to a specific resource.
     * @param resource Tree resource that contains the tree to be inserted.
     * @param tree_name Name of the tree to be inserted.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name does not exist in the resource.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromResource(
      const TreeResource & resource, const std::string & tree_name, const NodeElement * before_this = nullptr);

    /**
     * @brief Concatenate the root tree of one of the installed package's behavior tree resources and add its first
     * child node to the children of this node.
     *
     * Behavior tree resources are registered by calling the CMake macro `auto_apms_behavior_tree_register_trees` in the
     * CMakeLists.txt of a package. They can be inserted once the corresponding package has been installed to the ROS 2
     * workspace.
     *
     * Additionally, this method automatically extends the internal node manifest with the nodes associated with the
     * tree resource using the `NODE_MANIFEST` argument of `auto_apms_behavior_tree_register_trees`.
     *
     * The tree's first child is inserted as a child of this node. All subsequent children
     * of the inserted node are preserved. Therefore, this method effectively concatenates another tree and copies all
     * its nodes to the tree this node belongs to.
     *
     * @sa TreeResourceIdentity for more information about how to refer to a specific resource.
     * @param resource Tree resource that contains the tree to be inserted. It must specify a root tree or have a single
     * tree only.
     * @param before_this Pointer to an existing child node before which the first child of @p tree will be placed. If
     * `nullptr`, insert at the end.
     * @return Inserted node element representing the first child node of the concatenated tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree resource doesn't specify which of the
     * registered trees is the root tree.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p before_this is provided but not a child of
     * this node.
     */
    NodeElement insertTreeFromResource(const TreeResource & resource, const NodeElement * before_this = nullptr);

    /**
     * @brief Determine whether any children have been given to this node.
     * @return `true` if the node has one or more children, `false` otherwise.
     */
    bool hasChildren() const;

    // clang-format off
    /**
     * @brief Get the first node with a particular registration and instance name.
     *
     * This method determines which node element to return by evaluating its arguments as follows:
     *
     * |                                   | `instance_name` empty                                                                         |  `instance_name` non-empty                                                                    |
     * | :---:                             | ---                                                                                           | ---                                                                                           |
     * | `registration_name` **empty**     | Simply return the first child node without inspecting it further                              | Return the first child node that has the given instance name regardless its registration name |
     * | `registration_name` **non-empty** | Return the first child node that has the given registration name regardless its instance name | Return the first child node that has both the given registration *AND* instance name          |
     *
     * Additionally, the user may specify whether to recursively visit all descendants of this node in execution order by setting @p deep_search to `true`. By default, only direct children of this node are considered for the search.
     *
     * @param registration_name Registration name of the node that is searched for.
     * @param instance_name Name of the specific node instance that is searched for.
     * @param deep_search If `true`, recursively visit all descendants in execution order. If `false` (default), only consider
     * direct children of this node.
     * @return First child node element that meets the search criterion defined according to the above.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no node matching the search criterion could be
     * found.
     */
    NodeElement getFirstNode(
      const std::string & registration_name = "", const std::string & instance_name = "",
      bool deep_search = false) const;

    // clang-format off
    /**
     * @brief Get the first node with a particular instance name.
     *
     * The kind of node to search for is specified using a node model type passed as the template argument @p T.
     * Additionally, the user may provide an instance name. According to @p instance_name, the node element to be
     * returned is determined as follows:
     *
     * | `instance_name` empty                                                                                         |  `instance_name` non-empty                                                                                 |
     * | ---                                                                                                           |                                                                                                            |
     * | Return the first child node that has the registration name provided by the model regardless its instance name | Return the first child node that has both the registration name of the model *AND* the given instance name |
     *
     * Additionally, the user may specify whether to recursively visit all descendants of this node in execution order by setting @p deep_search to `true`. By default, only direct children of this node are considered for the search.
     *
     * @tparam T Node model type.
     * @param instance_name Name of the specific node instance that is searched for.
     * @param deep_search If `true`, recursively visit all descendants in execution order. If `false`, only consider
     * direct children of this node.
     * @return First child node element that meets the search criterion defined according to the above.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no node matching the search criterion could be
     * found.
     */
    template <class T>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, T>, T> getFirstNode(
      const std::string & instance_name = "", bool deep_search = false) const;

    // clang-format off
    /**
     * @brief Remove the first node with a particular registration and instance name.
     *
     * This method determines which node element to remove by evaluating its arguments as follows:
     *
     * |                                   | `instance_name` empty                                                                         |  `instance_name` non-empty                                                                    |
     * | :---:                             | ---                                                                                           | ---                                                                                           |
     * | `registration_name` **empty**     | Simply remove the first child node without inspecting it further                              | Remove the first child node that has the given instance name regardless its registration name |
     * | `registration_name` **non-empty** | Remove the first child node that has the given registration name regardless its instance name | Remove the first child node that has both the given registration *AND* instance name          |
     *
     * Additionally, the user may specify whether to recursively visit all descendants of this node in execution order by setting @p deep_search to `true`. By default, only direct children of this node are considered for the removal.
     *
     * @param registration_name Registration name of the node to be removed.
     * @param instance_name Name of the specific node instance to be removed.
     * @param deep_search If `true`, recursively visit all descendants in execution order. If `false` (default), only consider
     * direct children of this node.
     * @return Modified node element.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no node matching the search criterion could be
     * found.
     */
    NodeElement & removeFirstChild(
      const std::string & registration_name = "", const std::string & instance_name = "",
      bool deep_search = false);

    // clang-format off
    /**
     * @brief Remove the first node with a particular instance name.
     *
     * The kind of node to remove is specified using a node model type passed as the template argument @p T.
     * Additionally, the user may provide an instance name. According to @p instance_name, the child node to be removed
     * is determined as follows:
     *
     * | `instance_name` empty                                                                                         |  `instance_name` non-empty                                                                                 |
     * | ---                                                                                                           |                                                                                                            |
     * | Remove the first child node that has the registration name provided by the model regardless its instance name | Remove the first child node that has both the registration name of the model *AND* the given instance name |
     *
     * Additionally, the user may specify whether to recursively visit all descendants of this node in execution order by setting @p deep_search to `true`. By default, only direct children of this node are considered for the removal.
     *
     * @tparam T Node model type.
     * @param instance_name Name of the specific node instance to be removed.
     * @param deep_search If `true`, recursively visit all descendants in execution order. If `false` (default), only consider
     * direct children of this node.
     * @return Modified node element.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no node matching the search criterion could be
     * found.
     */
    template <class T>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, T>, NodeElement &> removeFirstChild(
      const std::string & instance_name = "", bool deep_search = false);
    // clang-format on

    /**
     * @brief Recursively remove all children of this node element.
     * @return Modified node element.
     */
    NodeElement & removeChildren();

    /**
     * @brief Get the names of the data ports implemented by the node represented by this element.
     * @return Names of all implemented data ports.
     */
    const std::vector<std::string> & getPortNames() const;

    /**
     * @brief Assemble the values given to each data port implemented by this node.
     * @return Mapping of port names and their corresponding values encoded as strings. If a port name is missing in
     * this map, no value has been assigned to it yet.
     */
    PortValues getPorts() const;

    /**
     * @brief Populate the the node's data ports.
     *
     * This method verifies that @p port_values only refers to implemented ports and throws an exception if any
     * values for unkown port names are provided.
     *
     * @param port_values Port values to be used to populate the corresponding attributes of the node element.
     * @return Reference to the modified instance.
     * @throws auto_apms_behavior_tree::exceptions::TreeDocumentError if @p port_values contains any unkown keys, e.g.
     * names for ports that are not implemented.
     */
    NodeElement & setPorts(const PortValues & port_values);

    /**
     * @brief Delete all currently specified port values and reset with the defaults.
     * @return Modified node element.
     */
    NodeElement & resetPorts();

    /**
     * @brief Specify a script that is evaluated before this node is ticked.
     *
     * @note Only the script assigned to `BT::PreCond::WHILE_TRUE` is evaluated before each tick. The other precondition
     * types are only evaluated before the first tick and ignored afterwards. This is important to know for asynchronous
     * nodes.
     * @param type Type of the precondition.
     * @param script Script to be evaluated.
     * @return Modified node element.
     */
    NodeElement & setConditionalScript(BT::PreCond type, const Script & script);

    /**
     * @brief Specify a script that is evaluated after the tree is done executing this node.
     * @param type Type of the postcondition.
     * @param script Script to be evaluated.
     * @return Modified node element.
     */
    NodeElement & setConditionalScript(BT::PostCond type, const Script & script);

    /**
     * @brief Assign a name for this specific node instance.
     * @param instance_name Instance name.
     * @return Modified node element.
     */
    virtual NodeElement & setName(const std::string & instance_name);

    /**
     * @brief Get the name of this node given during registration representing its dynamic type.
     * @return Registration name of this node.
     */
    virtual std::string getRegistrationName() const;

    /**
     * @brief Get the name of this node given to this specific instance by the developer.
     * @return Instance name of this node.
     */
    virtual std::string getName() const;

    /**
     * @brief Create a string that uniquely identifies this node considering its registration and its instance name.
     * @return Fully qualified name of this node.
     */
    std::string getFullyQualifiedName() const;

    /**
     * @brief Get a const view of this node's parent tree document.
     *
     * The returned reference to the parent tree document does not allow modification in any way. This is a deliberate
     * design decision effectively creating a development scope that is limited to this node element.
     * Therefore, the user cannot access any nodes above the current nesting level.
     * @return Tree document that this node belongs to.
     */
    const TreeDocument & getParentDocument() const;

    /**
     * @brief Recursively apply a callback to this node's children.
     *
     * @p apply_callback is applied recursively to each of the child nodes starting from the first (left) child and must
     * return a boolean value. Once the children of the first child have been visited, the recursion proceeds to the
     * next one (right neighbor) until this has happened for all children. Everytime the callback returns `true`, the
     * corresponding node element is appended to a vector that is returned at the end. If the callback returns `false`,
     * the respective node element will **not** be included.
     *
     * If you need to modify the node elements that are visited, refer to NodeElement::deepApply.
     *
     * @note This method only applies the callback to nodes of the current parent tree and **does not**
     * automatically visit nodes of subtrees referred to by possible subtree nodes.
     * @param apply_callback Callback to be applied to each subsequent node elements. It is passed a const reference to
     * the respective node element.
     * @return Vector of node elements that the callback returned `true` for.
     */
    const std::vector<NodeElement> deepApplyConst(ConstDeepApplyCallback apply_callback) const;

    /**
     * @brief Recursively apply a callback to this node's children.
     *
     * @p apply_callback is applied recursively to each of the child nodes starting from the first (left) child and must
     * return a boolean value. Once the children of the first child have been visited, the recursion proceeds to the
     * next one (right neighbor) until this has happened for all children. Everytime the callback returns `true`, the
     * corresponding node element is appended to a vector that is returned at the end. If the callback returns `false`,
     * the respective node element will **not** be included.
     *
     * @note This method only applies the callback to nodes of the current parent tree and **does not**
     * automatically visit nodes of subtrees referred to by possible subtree nodes.
     * @param apply_callback Callback to be applied to each subsequent node elements. It is passed a reference to
     * the respective node element.
     * @return Vector of node elements that the callback returned `true` for.
     */
    std::vector<NodeElement> deepApply(DeepApplyCallback apply_callback);

    /**
     * @brief Forward iterator for traversing the first-level children of a node.
     *
     * This iterator yields NodeElement handles for each direct child element. It does **not** recurse into children of
     * children. For recursive traversal, use NodeElement::deepApply or NodeElement::deepApplyConst instead.
     */
    class ChildIterator
    {
      friend class NodeElement;

    public:
      using iterator_category = std::forward_iterator_tag;
      using value_type = NodeElement;
      using difference_type = std::ptrdiff_t;
      using pointer = value_type *;
      using reference = value_type;

      /// Create a past-the-end iterator.
      ChildIterator();

      /// Dereference the iterator to obtain a NodeElement handle for the current child.
      value_type operator*() const;

      /// Pre-increment: advance to the next sibling element.
      ChildIterator & operator++();

      /// Post-increment: advance to the next sibling element, returning the previous state.
      ChildIterator operator++(int);

      bool operator==(const ChildIterator & other) const;
      bool operator!=(const ChildIterator & other) const;

    private:
      ChildIterator(TreeDocument * doc_ptr, tinyxml2::XMLElement * current);

      TreeDocument * doc_ptr_;
      tinyxml2::XMLElement * current_;
    };

    /**
     * @brief Get an iterator to the first child of this node.
     * @return Iterator pointing to the first child, or a past-the-end iterator if there are no children.
     */
    ChildIterator begin() const;

    /**
     * @brief Get a past-the-end iterator for the children of this node.
     * @return Past-the-end iterator.
     */
    ChildIterator end() const;

  private:
    NodeElement insertBeforeImpl(const NodeElement * before_this, XMLElement * add_this);

    static void deepApplyImpl(
      const NodeElement & parent, ConstDeepApplyCallback apply_callback, std::vector<NodeElement> & vec);

    static void deepApplyImpl(NodeElement & parent, DeepApplyCallback apply_callback, std::vector<NodeElement> & vec);

  protected:
    /// Pointer to the tree document that created the tree this node belongs to.
    TreeDocument * doc_ptr_;
    ///  Pointer to the corresponding `XMLElement` of the base document.
    tinyxml2::XMLElement * ele_ptr_;

  private:
    std::vector<std::string> port_names_;
    PortValues port_default_values_;
  };

  /**
   * @brief Handle for a single behavior tree of a TreeDocument.
   *
   * Tree elements act as an entry point for adding nodes to a behavior tree. They must be created using
   * TreeDocument::getTree, TreeDocument::newTree or the other **newTree** methods. A behavior tree document can have an
   * arbitrary amount of behavior trees. For each behavior tree, there is a unique tree element with the corresponding
   * tree name as its instance name.
   */
  class TreeElement : public NodeElement
  {
    friend class TreeDocument;

  protected:
    /**
     * @brief Protected constructor intended to be used only by certain factory methods of TreeDocument.
     * @param doc_ptr Pointer to the tree document that creates this instance.
     * @param ele_ptr Pointer to the corresponding `XMLElement` of the base document.
     */
    explicit TreeElement(TreeDocument * doc_ptr, XMLElement * ele_ptr);

  public:
    /**
     * @brief Copy constructor for a tree element.
     * @param ele Other tree element.
     */
    TreeElement(const TreeElement & ele) = default;

    /**
     * @brief Replace the behavior tree represented by this element with another
     *
     * The assignment operator copies the other behavior tree and places it in this position. This is only allowed if
     * the name of the other tree is not already taken within the parent document. However, it is supported to replace
     * this tree with another tree that has the same name.
     * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if another tree with the same name already exists
     * inside the parent document.
     */
    TreeElement & operator=(const TreeElement & other);

    /**
     * @brief Set the name of the behavior tree.
     * @param tree_name Name of the behavior tree.
     * @return Modified tree element.
     */
    TreeElement & setName(const std::string & tree_name) override;

    /**
     * @brief Get the name of the behavior tree.
     * @return Name of the behavior tree.
     */
    std::string getName() const override;

    /**
     * @brief Set this behavior tree as the root tree of the parent document.
     *
     * This function doesn't modify the tree element, but rather its parent document by modifying the corresponding
     * attribute.
     * @return This tree element.
     */
    TreeElement & makeRoot();

    /**
     * @brief Assemble the node manifest that is required for successfully creating an instance of this tree.
     *
     * This function recursively visits all nodes inside this behavior tree starting from its first child node and looks
     * up the registration options passed when inserting it. If it comes across a subtree node, the corresponding tree
     * **is not** parsed. Therefore, the returned node manifest only contains the nodes associated with this very tree.
     * @return Node manifest which contains the registration options for all nodes associated with this tree.
     * @throw auto_apms_behavior_tree::exceptions::NodeManifestError if there are no registration options for a specific
     * node.
     */
    NodeManifest getRequiredNodeManifest() const;

    /**
     * @brief Verify that this behavior tree is structured correctly and can be created successfully.
     * @return Result that evaluates to `true` if verification succeeded. Otherwise, it encapsulates a reason why it
     * failed.
     */
    BT::Result verify() const;

    /**
     * @brief Write this behavior tree to an XML encoded in a string.
     *
     * In contrast to TreeDocument::writeToString, the resulting XML contains the tree element as its root element,
     * emphasizing the fact that this function returns an XML for a single behavior tree only.
     * @return String representing the behavior tree in XML format.
     */
    std::string writeToString() const;

    /**
     * This function is an exact reimplementation of NodeElement::removeFirstChild(const std::string &, const
     * std::string &, bool) only that it returns a tree element instead of a node element to support method chaining.
     */
    TreeElement & removeFirstChild(
      const std::string & registration_name = "", const std::string & instance_name = "", bool deep_search = false);

    /**
     * This function is an exact reimplementation of NodeElement::removeFirstChild<T>(const std::string &, bool) only
     * that it returns a tree element instead of a node element to support method chaining.
     */
    template <class T>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, T>, TreeElement &> removeFirstChild(
      const std::string & instance_name = "", bool deep_search = false);

    /**
     * This function is an exact reimplementation of NodeElement::removeChildren only that it
     * returns a tree element instead of a node element to support method chaining.
     */
    TreeElement & removeChildren();

    /* Not supported methods for TreeElement instances */

    const std::vector<std::string> & getPortNames() = delete;
    PortValues getPorts() = delete;
    NodeElement & setPorts() = delete;
    NodeElement & resetPorts() = delete;
    NodeElement & setConditionalScript() = delete;
  };

  /**
   * @brief Create a an empty tree document.
   *
   * You may specify a format version. Two tree documents must have the same format version for them to be mergeable.
   *
   * Additionally, you may pass a pointer to an already existing behavior tree node loader. By default, a new one is
   * created with default arguments.
   * @param format_version Format version number encoded in a string.
   * @param tree_node_loader Shared pointer to an existing behavior tree node loader.
   */
  TreeDocument(
    const std::string & format_version = BTCPP_FORMAT_DEFAULT_VERSION,
    NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

  TreeDocument(const TreeDocument & other) = delete;

  virtual ~TreeDocument() = default;

  /**
   * @brief Merge another tree document with this one.
   *
   * This function copies all behavior trees of @p other and adds them to this document. The other document must either
   * have a root element `<root>` that may contain an arbitrary amount of `<BehaviorTree>` child elements, or a single
   * `<BehaviorTree>` element as its root element.
   *
   * If @p adopt_root_tree is `true` and there is only a single behavior tree, this tree will be considered the new root
   * tree of this document. However, if there are multiple behavior trees under a top-level `<root>` element and this
   * element has the optional `main_tree_to_execute` attribute specifying the root tree name, the value of this
   * attribute will be copied to this document. If the other root element doesn't have this attribute, the current
   * root tree name is kept.
   * @param other Low-level XML document containing the trees to be merged.
   * @param adopt_root_tree Set to `true` if you additionally want to update this document's root tree name according to
   * @p other.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the tree names found in @p other are
   * already taken by this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the format of @p other is not the same as the
   * format of this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the trees inside @p other are malformed.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the content of @p other cannot be interpreted.
   */
  TreeDocument & mergeTreeDocument(const XMLDocument & other, bool adopt_root_tree = false);

  /**
   * @brief Merge another tree document with this one.
   *
   * This function copies all behavior trees of @p other and adds them to this document.
   *
   * If @p adopt_root_tree is `true` and there is only a single behavior tree, this tree will be considered the new root
   * tree of this document. However, if there are multiple behavior trees, the root tree name of the other document will
   * be copied to this document. If the other document doesn't specify which tree is the root tree, the current one is
   * kept.
   * @param other Other tree document containing the trees to be merged.
   * @param adopt_root_tree Set to `true` if you additionally want to update this document's root tree name according to
   * @p other.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the tree names found in @p other are
   * already taken by this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the format of @p other is not the same as the
   * format of this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the trees inside @p other are malformed.
   */
  TreeDocument & mergeTreeDocument(const TreeDocument & other, bool adopt_root_tree = false);

  /**
   * @brief Create a tree document from a string and merge it with this one.
   *
   * This function copies all behavior trees found inside @p tree_str and adds them to this document. The XML string
   * must either have a root element `<root>` that may contain an arbitrary amount of `<BehaviorTree>` child elements,
   * or a single `<BehaviorTree>` element as its root element.
   *
   * If @p adopt_root_tree is `true` and there is only a single behavior tree, this tree will be considered the new root
   * tree of this document. However, if there are multiple behavior trees under a top-level `<root>` element and this
   * element has the optional `main_tree_to_execute` attribute specifying the root tree name, the value of this
   * attribute will be copied to this document. If the other root element doesn't have this attribute, the current
   * root tree name is kept.
   * @param tree_str XML string specifying the tree document to be merged.
   * @param adopt_root_tree Set to `true` if you additionally want to update this document's root tree name according to
   * @p tree_str.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the tree names found in @p tree_str are
   * already taken by this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the format of the XML in @p tree_str is not the
   * same as the format of this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the trees inside @p tree_str are malformed.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the content of @p tree_str cannot be interpreted.
   */
  TreeDocument & mergeString(const std::string & tree_str, bool adopt_root_tree = false);

  /**
   * @brief Create a tree document from a file and merge it with this one.
   *
   * This function copies all behavior trees found inside @p path and adds them to this document. The XML file must
   * either have a root element `<root>` that may contain an arbitrary amount of `<BehaviorTree>` child elements, or a
   * single `<BehaviorTree>` element as its root element.
   *
   * If @p adopt_root_tree is `true` and there is only a single behavior tree, this tree will be considered the new root
   * tree of this document. However, if there are multiple behavior trees under a top-level `<root>` element and this
   * element has the optional `main_tree_to_execute` attribute specifying the root tree name, the value of this
   * attribute will be copied to this document. If the other root element doesn't have this attribute, the current
   * root tree name is kept.
   * @param path Path to an XML file specifying the tree document to be merged.
   * @param adopt_root_tree Set to `true` if you additionally want to update this document's root tree name according to
   * @p path.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the tree names found under @p path are
   * already taken by this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the format of the XML in @p path is not the same
   * as the format of this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the trees inside the XML in @p path are
   * malformed.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the content of @p path cannot be interpreted.
   */
  TreeDocument & mergeFile(const std::string & path, bool adopt_root_tree = false);

  /**
   * @brief Merge the behavior trees from one of the installed package's behavior tree resources.
   *
   * This function parses the XML and the node manifest associated with @p resource. First, it registers all associated
   * behavior tree nodes with this document and then merges the tree document created from the resource's XML file.
   *
   * If @p adopt_root_tree is `true` and there is only a single behavior tree, this tree will be considered the new root
   * tree of this document. However, if there are multiple behavior trees, the root tree name of the other document will
   * be copied to this document. If the other document doesn't specify which one is the root tree, the current one is
   * kept.
   *
   * @sa TreeResourceIdentity for more information about how to refer to a specific resource.
   * @param resource Behavior tree resource that specifies the tree document to be merged.
   * @param adopt_root_tree Set to `true` if you additionally want to update this document's root tree name according to
   * @p resource.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the tree names found under @p resource are
   * already taken by this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the format of the XML associated with @p resource
   * is not the same as the format of this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if any of the trees associated with @p resource are
   * malformed.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the content of the XML associated with @p resource
   * cannot be interpreted.
   */
  TreeDocument & mergeResource(const TreeResource & resource, bool adopt_root_tree = false);

  /**
   * @brief Merge an existing behavior tree with this tree document.
   *
   * Before merging the behavior tree represented by @p tree, all required nodes are automatically registered with this
   * document.
   *
   * This function is intended for copying a behavior tree from another tree document. If you call this with a
   * tree element that was created using this document or the other tree has the same name as one of the existing trees,
   * an error is raised since a document's tree names must be unique.
   * @param tree Tree element representing the tree to merge.
   * @param make_root_tree Set to `true` if you want to make @p tree the new root tree of this document.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree's name already exists inside this
   * document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the tree is malformed.
   */
  TreeDocument & mergeTree(const TreeElement & tree, bool make_root_tree = false);

  /**
   * @brief Create a new behavior tree inside this document.
   * @param tree_name Name of the behavior tree.
   * @return Tree element representing the new behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if a tree with name @p tree_name already exists
   * inside this document.
   */
  TreeElement newTree(const std::string & tree_name);

  /**
   * @brief Create a new behavior tree inside this document with the same content of another.
   *
   * Before creating a new tree using @p other_tree, all required nodes are automatically registered with this document.
   *
   * This function is intended for copying a behavior tree from another tree document. If you call this with a
   * tree element that was created using this document or the other tree has the same name as one of the existing trees,
   * an error is raised since a document's tree names must be unique.
   * @param other_tree Other behavior tree to be copied.
   * @return Tree element representing the new behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p other_tree has the same name as one of this
   * document's trees.
   */
  TreeElement newTree(const TreeElement & other_tree);

  /**
   * @brief Create a new behavior tree inside this document with the content of one found inside another tree document.
   *
   * Which behavior tree will be used to create the new tree is determined by @p tree_name. By default, this argument is
   * empty, which means that the root tree will be used. If @p other only contains a single tree, this one is
   * considered the root tree.
   * @param other Other tree document that contains the tree to be copied.
   * @param tree_name Name of the tree to be copied. It must exist inside @p other.
   * @return Tree element representing the new behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name is empty but @p other doesn't specify
   * which tree is the root tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name cannot be found in @p other.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if a tree with name @p tree_name already exists
   * inside this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the respective tree contains any nodes that are
   * not available with the currently configured node manifest.
   */
  TreeElement newTreeFromDocument(const TreeDocument & other, const std::string & tree_name = "");

  /**
   * @brief Create a new behavior tree inside this document with the content of one found inside the XML string.
   *
   * Which behavior tree will be used to create the new tree is determined by @p tree_name. By default, this argument is
   * empty, which means that the root tree will be used. If @p tree_str only contains a single tree, this one is
   * considered the root tree.
   * @param tree_str String specifying the XML of a tree document that holds the tree to be copied.
   * @param tree_name Name of the tree to be copied. It must exist inside @p tree_str.
   * @return Tree element representing the new behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name is empty but @p tree_str doesn't
   * specify which tree is the root tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name cannot be found in @p tree_str.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if a tree with name @p tree_name already exists
   * inside this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the respective tree contains any nodes that are
   * not available with the currently configured node manifest.
   */
  TreeElement newTreeFromString(const std::string & tree_str, const std::string & tree_name = "");

  /**
   * @brief Create a new behavior tree inside this document with the content of one found inside the XML file.
   *
   * Which behavior tree will be used to create the new tree is determined by @p tree_name. By default, this argument is
   * empty, which means that the root tree will be used. If the file under @p path only contains a single tree, this one
   * is considered the root tree.
   * @param path Path to an XML file specifying the tree document that holds the tree to be copied.
   * @param tree_name Name of the tree to be copied. It must exist inside the given file.
   * @return Tree element representing the new behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name is empty but the given file doesn't
   * specify which tree is the root tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name cannot be found in the given file.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if a tree with name @p tree_name already exists
   * inside this document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if the respective tree contains any nodes that are
   * not available with the currently configured node manifest.
   */
  TreeElement newTreeFromFile(const std::string & path, const std::string & tree_name = "");

  /**
   * @brief Create a new behavior tree inside this document with the content of one the trees found inside a particular
   * behavior tree resource.
   *
   * Before creating a new tree using @p resource, all required nodes are automatically registered with this document.
   *
   * Which behavior tree will be used to create the new tree is determined by @p tree_name. By default, this argument is
   * empty, which means that the root tree will be used. If the given resource only contains a single tree, this one is
   * considered the root tree. Another way to define the root tree with this function is to specify the `<tree_name>`
   * token when creating the resource object with an identity string. However, the @p tree_name argument takes
   * precedence over this token.
   *
   * @sa TreeResourceIdentity for more information about how to refer to a specific resource.
   * @param resource Behavior tree resource to use.
   * @param tree_name Name of the tree to be copied. It must exist inside the resource's tree document. By default the
   * root tree is used.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name is empty but the root tree cannot be
   * determined.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree_name cannot be found in the resource's
   * tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if a tree with name @p tree_name already exists
   * inside this document.
   */
  TreeElement newTreeFromResource(const TreeResource & resource, const std::string & tree_name = "");

  /**
   * @brief Determine if this document contains a behavior tree with a particular name.
   * @param tree_name Name of the tree.
   * @return `true` if a tree named @p tree_name exists in this document, `false` otherwise.
   */
  bool hasTreeName(const std::string & tree_name) const;

  /**
   * @brief Get the corresponding behavior tree element for a tree inside this document.
   * @param tree_name Name of the tree.
   * @return Tree element representing an existing behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no tree named @p tree_name exists inside this
   * document.
   */
  TreeElement getTree(const std::string & tree_name);

  /**
   * @brief Define the root tree of this document.
   * @param tree_name Name of an existing tree to be considered the root tree.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no tree named @p tree_name exists inside this
   * document.
   */
  TreeDocument & setRootTreeName(const std::string & tree_name);

  /**
   * @brief Determine if this document specifies which of its trees is the root tree.
   * @return `true` if this document specifies a root tree, `false` otherwise.
   */
  bool hasRootTreeName() const;

  /**
   * @brief Get the name of this document's root tree.
   * @return Name of the root tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if this document doesn't specify which tree is the
   * root tree.
   */
  std::string getRootTreeName() const;

  /**
   * @brief Get the corresponding behavior tree element for the root tree of this document.
   * @return Tree element representing this document's root tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if this document doesn't specify which tree is the
   * root tree.
   */
  TreeElement getRootTree();

  /**
   * @brief Remove a particular behavior tree from this document.
   *
   * We only remove the tree and it's child nodes, but not the corresponding node registrations with the behavior tree
   * factory, since this just means more work and is really unnecessary, especially if you consider that if we remove
   * any node registration now and insert it again later somewhere inside the document, we would need to register the
   * node again. Since we assume that the amount of memory is not an issue (and the callbacks that we store when
   * registering don't need much anyways), it would just be extra work.
   * @param tree_name Name of the tree to be removed.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if no tree named @p tree_name exists inside this
   * document.
   */
  TreeDocument & removeTree(const std::string & tree_name);

  /**
   * @brief Remove a particular behavior tree from this document.
   *
   * We only remove the tree and it's child nodes, but not the corresponding node registrations with the behavior tree
   * factory, since this just means more work and is really unnecessary, especially if you consider that if we remove
   * any node registration now and insert it again later somewhere inside the document, we would need to register the
   * node again. Since we assume that the amount of memory is not an issue (and the callbacks that we store when
   * registering don't need much anyways), it would just be extra work.
   * @param tree Tree element representing the tree to be removed.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::TreeDocumentError if @p tree does not belong to this document.
   */
  TreeDocument & removeTree(const TreeElement & tree);

  /**
   * @brief Get the names of all behavior trees inside this document.
   * @return Vector of all behavior tree names inside this document.
   */
  std::vector<std::string> getAllTreeNames() const;

  /**
   * @brief Prepend a namespace to all nodes associated with this document.
   *
   * This is useful to avoid name clashes when merging multiple tree documents that may contain nodes with the same
   * name.
   *
   * @param node_namespace Namespace that is prepended to each registered node name and its corresponding usages in the
   * document.
   * @param sep Separator that is placed between the namespace and the original node name.
   * @return Modified tree document.
   */
  TreeDocument & applyNodeNamespace(
    const std::string & node_namespace,
    const std::string & sep = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_NAMESPACE_DEFAULT_SEP);

  /**
   * @brief Load behavior tree node plugins and register them with the internal behavior tree factory.
   *
   * This makes it possible to add any nodes specified in @p tree_node_manifest to the tree.
   *
   * @param tree_node_manifest Parameters for locating and configuring the behavior tree node plugins.
   * @param override If @p tree_node_manifest specifies node registration names that have been seen before in other
   * manifests, setting this argument to `true` will override the previous registrations. If set to `false`, an error is
   * raised in this case.
   * @return Modified tree document.
   * @throw auto_apms_behavior_tree::exceptions::NodeRegistrationError if registration fails.
   */
  virtual TreeDocument & registerNodes(const NodeManifest & tree_node_manifest, bool override = false);

  /**
   * @brief Get the names of all nodes that are known to this document.
   * @param include_native Set to `true` if the native *BehaviorTree.CPP* nodes should be included, `false` to only
   * consider node plugins.
   * @return Names of all nodes registered with this document.
   */
  std::set<std::string> getRegisteredNodeNames(bool include_native = true) const;

  /**
   * @brief Assemble the node manifest that is required for successfully creating an instance of any of the document's
   * trees.
   * @return Node manifest which contains the registration options for all nodes inside this document.
   * @throw auto_apms_behavior_tree::exceptions::NodeManifestError if there are no registration options for a specific
   * node.
   */
  NodeManifest getRequiredNodeManifest() const;

  /**
   * @brief Add a behavior tree node model element to the document by parsing the contents of @p model_map.
   *
   * This is required when using the Groot2 visual editor.
   * @param model_map Mapping of node models to be added.
   * @return Modified tree document.
   */
  TreeDocument & addNodeModel(NodeModelMap model_map);

  /**
   * @brief Convert a behavior tree node model document to the corresponding data structure.
   * @param doc XML document containing the node model.
   * @param manifest Node manifest associated with the nodes specified by @p doc. This is only used to query optional
   * information that affects the node model (e.g., hidden ports).
   * @return Mapping of node models for all nodes specified by @p doc.
   */
  static NodeModelMap getNodeModel(tinyxml2::XMLDocument & doc, const NodeManifest & manifest);

  /**
   * @brief Create a behavior tree node model for all nodes registered with this document.
   *
   * Hidden ports are automatically extracted from the NodeRegistrationOptions::hidden_ports field of registered nodes.
   *
   * @param include_native Set to `true` if the native *BehaviorTree.CPP* nodes should be included, `false` to only
   * consider registered node plugins.
   * @return Mapping of node models for all nodes registered with this document.
   */
  NodeModelMap getNodeModel(bool include_native = false) const;

  /**
   * @brief Verify that all behavior trees of this document are structured correctly and can be created successfully.
   * @return Result that evaluates to `true` if verification succeeded. Otherwise, it encapsulates a reason why it
   * failed.
   */
  BT::Result verify() const;

  /**
   * @brief Write the XML of this tree document to a string.
   * @return String representing this document in XML format.
   */
  std::string writeToString() const;

  /**
   * @brief Write the XML of this tree document to a file.
   * @param path Path to the output file.
   */
  void writeToFile(const std::string & path) const;

  /**
   * @brief Clear this document and reset it to its initial state.
   * @return Modified tree document.
   */
  TreeDocument & reset();

private:
  template <typename ReturnT, typename DocumentT>
  static ReturnT getXMLElementForTreeWithNameImpl(DocumentT & doc, const std::string & tree_name);

  const XMLElement * getXMLElementForTreeWithName(const std::string & tree_name) const;

  XMLElement * getXMLElementForTreeWithName(const std::string & tree_name);

  /**
   * @brief Internal implementation of mergeFile with circular include detection.
   * @param path Path to an XML file specifying the tree document to be merged.
   * @param adopt_root_tree Set to `true` if you additionally want to update this document's root tree name.
   * @param include_stack Set of file paths currently being processed to detect circular includes.
   * @return Modified tree document.
   */
  TreeDocument & mergeFileImpl(const std::string & path, bool adopt_root_tree, std::set<std::string> & include_stack);

  /**
   * @brief Internal implementation of mergeTreeDocument with circular include detection.
   * @param other Low-level XML document containing the trees to be merged.
   * @param adopt_root_tree Set to `true` if you additionally want to update this document's root tree name.
   * @param include_stack Reference to include stack for circular detection.
   * @return Modified tree document.
   */
  TreeDocument & mergeTreeDocumentImpl(
    const XMLDocument & other, bool adopt_root_tree, std::set<std::string> & include_stack);

  const std::map<std::string, std::string> all_node_classes_package_map_;
  const std::set<std::string> native_node_names_;
  std::string format_version_;
  NodeRegistrationLoader::SharedPtr tree_node_loader_ptr_;

protected:
  NodeManifest registered_nodes_manifest_;
  BT::BehaviorTreeFactory factory_;
  rclcpp::Logger logger_;
  rclcpp::Node::WeakPtr ros_node_wptr_;
  rclcpp::CallbackGroup::WeakPtr tree_node_waitables_callback_group_wptr_;
  rclcpp::executors::SingleThreadedExecutor::WeakPtr tree_node_waitables_executor_wptr_;
  bool only_non_ros_nodes_ = false;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond INTERNAL
template <typename T, typename = void>
struct has_static_method_registrationOptions : std::false_type
{
};

template <typename T>
struct has_static_method_registrationOptions<
  T, typename std::enable_if_t<std::is_same_v<decltype(T::registrationOptions()), NodeRegistrationOptions>>>
: std::true_type
{
};
/// @endcond

template <class T>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, T> && !std::is_same_v<model::SubTree, T>, T>
TreeDocument::NodeElement::insertNode(const NodeElement * before_this)
{
  // Determine if overload for automatically trying to register the node can be used. Models for native nodes don't
  // implement the required static method, thus the other overload must be used.
  if constexpr (has_static_method_registrationOptions<T>::value) {
    NodeElement ele = insertNode(T::name(), T::registrationOptions(), before_this);
    return T(ele.doc_ptr_, ele.ele_ptr_);
  } else {
    NodeElement ele = insertNode(T::name(), before_this);
    return T(ele.doc_ptr_, ele.ele_ptr_);
  }
}

template <class T>
inline typename std::enable_if_t<std::is_same_v<model::SubTree, T>, model::SubTree>
TreeDocument::NodeElement::insertNode(const std::string & tree_name, const NodeElement * before_this)
{
  return insertSubTreeNode(tree_name, before_this);
}

template <class T>
inline typename std::enable_if_t<std::is_same_v<model::SubTree, T>, model::SubTree>
TreeDocument::NodeElement::insertNode(const TreeElement & tree, const NodeElement * before_this)
{
  return insertSubTreeNode(tree, before_this);
}

template <class T>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, T>, T> core::TreeDocument::NodeElement::getFirstNode(
  const std::string & instance_name, bool deep_search) const
{
  const NodeElement ele = getFirstNode(T::name(), instance_name, deep_search);
  return T(ele.doc_ptr_, ele.ele_ptr_);
}

template <class T>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, T>, TreeDocument::NodeElement &>
core::TreeDocument::NodeElement::removeFirstChild(const std::string & instance_name, bool deep_search)
{
  return removeFirstChild(T::name(), instance_name, deep_search);
}

template <class T>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, T>, TreeDocument::TreeElement &>
TreeDocument::TreeElement::removeFirstChild(const std::string & instance_name, bool deep_search)
{
  NodeElement::removeFirstChild<T>(instance_name, deep_search);
  return *this;
}

}  // namespace auto_apms_behavior_tree::core

// Include after TreeDocument is fully defined to provide complete type for model::SubTree
// which is forward declared above but used as return type in insertSubTreeNode methods.
#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"