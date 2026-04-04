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

#include <set>
#include <string>
#include <vector>

#include "auto_apms_behavior_tree_core/behavior.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Struct that encapsulates the identity string for a registered behavior tree.
 *
 * Its only purpose is to create the corresponding instance of TreeResource.
 *
 * The identity string is formatted like `<package_name>::<tree_file_stem>::<tree_name>`. Since TreeResource uses the
 * same identity resolution as BehaviorResource (where `<behavior_alias>` = `<tree_file_stem>::<tree_name>`), all
 * short forms documented in BehaviorResourceIdentity apply. Both `<tree_file_stem>` and `<tree_name>` must always be
 * provided.
 *
 * @note Because the behavior alias for tree resources contains `::`, the bare alias form
 * `<tree_file_stem>::<tree_name>` is **ambiguous** with the `<package_name>::<behavior_alias>` form of
 * BehaviorResourceIdentity. You must use at least `<package_name>::<tree_file_stem>::<tree_name>` when constructing
 * a TreeResourceIdentity from a string (`<package_name>` can be empty though). For partial lookups, use
 * TreeResource::findByTreeName() or TreeResource::findByFileStem().
 */
struct TreeResourceIdentity : public BehaviorResourceIdentity
{
  /**
   * @brief Constructor of a tree resource identity object.
   *
   * @p identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param identity Identity string for a specific behavior tree resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  TreeResourceIdentity(const std::string & identity);

  /**
   * @brief Constructor of a tree resource identity object.
   *
   * @p identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param identity C-style identity string for a specific behavior tree resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  TreeResourceIdentity(const char * identity);

  /**
   * @brief Constructor of an empty behavior tree resource identity object.
   *
   * The user must manually populate the member fields.
   */
  TreeResourceIdentity() = default;

  /// Name of the file (without extension) that contains the resource's tree document.
  std::string file_stem;
  /// Name of a specific tree inside the resource's tree document.
  std::string tree_name;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Class containing behavior tree resource data
 *
 * Behavior tree resources are registered by calling the CMake macro `auto_apms_behavior_tree_register_trees` in the
 * CMakeLists.txt of a package. They can be discovered once the corresponding package has been installed to the ROS 2
 * workspace.
 *
 * @note `auto_apms_behavior_tree_register_trees` replaces `auto_apms_behavior_tree_register_behavior` when registering
 * behavior trees and the user should only invoke the former. Registering the corresponding behavior resource
 * information is handled fully automatically for tree resources.
 *
 * TreeResource inherits from BehaviorResource and does not define any additional identity resolution logic. The
 * underlying behavior alias for a tree resource is `<tree_file_stem>::<tree_name>`, so all identity formats supported
 * by BehaviorResource apply uniformly:
 *
 * - `<category_name>/<package_name>::<tree_file_stem>::<tree_name>` — Fully qualified.
 *
 * - `<package_name>::<tree_file_stem>::<tree_name>` — Omit category (searched in all categories).
 *
 * - `::<tree_file_stem>::<tree_name>` — Omit both category and package (alias-only search). The leading `::` MUST be
 * kept to avoid misinterpreting `<tree_file_stem>` as `<package_name>`.
 *
 * @note When using identity strings, both `<tree_file_stem>` and `<tree_name>` must always be specified as part of the
 * behavior alias. For convenience, the static methods findByTreeName() and findByFileStem() allow searching by just
 * one component without requiring the full alias.
 *
 * ## Usage
 *
 * Given the user has specified a behavior tree named `MyBehaviorTree` inside the XML file `behavior/my_tree_file.xml`,
 * the CMake macro `auto_apms_behavior_tree_register_trees` must be called in the CMakeLists.txt of the parent package
 * (for example `my_package`) like this:
 *
 * ```cmake
 * auto_apms_behavior_tree_register_trees(
 *     "behavior/my_tree_file.xml"
 * )
 * ```
 *
 * The macro automatically parses the given files and detects the names of the trees inside. In the C++ source code, one
 * may use this resource, after the parent package `my_package` has been installed, like this:
 *
 * ```cpp
 * #include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
 *
 * using namespace auto_apms_behavior_tree;
 *
 * // For example, use the fully qualified tree resource identity signature
 * const std::string identity_string = "my_package::my_tree_file::MyBehaviorTree";
 *
 * // You may use the proxy class for a tree resource identity
 * core::TreeResourceIdentity identity(identity_string);
 * core::TreeResource resource(identity);
 *
 * // Or instantiate the resource object directly from the corresponding identity string
 * core::TreeResource resource(identity_string);
 *
 * // The resource object may for example be used with TreeDocument
 * core::TreeDocument doc;
 * doc.mergeResource(resource);  // Add "MyBehaviorTree" to the document
 *
 * // This also works, so creating a resource object is not strictly necessary
 * doc.mergeResource(identity_string)
 *
 * // The simplest approach is this
 * doc.mergeResource("my_package::my_tree_file::MyBehaviorTree");
 * ```
 *
 * @sa BehaviorResource
 */
class TreeResource : public BehaviorResourceTemplate<TreeResourceIdentity>
{
  friend class TreeDocument;
  friend class TreeBuilder;

public:
  /**
   * @brief Assemble a behavior tree resource using a TreeResourceIdentity.
   * @param search_identity Tree resource identity object used for searching the corresponding resource.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given
   * identity.
   */
  TreeResource(const TreeResourceIdentity & search_identity);

  /**
   * @brief Assemble a behavior tree resource identified by a string.
   *
   * @p search_identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param search_identity Tree resource identity string used for searching the corresponding resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity string.
   */
  TreeResource(const std::string & search_identity);

  /**
   * @brief Assemble a behavior tree resource identified by a string.
   *
   * @p search_identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param search_identity C-style tree resource identity string used for searching the corresponding resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity string.
   */
  TreeResource(const char * search_identity);

  /**
   * @brief Determine if this behavior tree resource specifies a root tree.
   *
   * The name of the root tree is determined as follows:
   *
   * - If the `<tree_name>` token was present in the resource identity when this instance was created, this is
   * considered the root tree name.
   *
   * - Otherwise, the XML of the associated tree document is parsed to determine the root tree.
   *
   * @return `true` if the root tree of this resource can be determined, `false` otherwise.
   */
  bool hasRootTreeName() const;

  /**
   * @brief Get the name of the root tree of this behavior tree resource.
   *
   * The name of the root tree is determined as follows:
   *
   * - If the `<tree_name>` token was present in the resource identity when this instance was created, this is
   * considered the root tree name.
   *
   * - Otherwise, the XML of the associated tree document is parsed to determine the root tree.
   *
   * @return Name of this resource's root tree.
   * @throw auto_apms_util::exceptions::ResourceError if the name of the root tree cannot be determined.
   */
  std::string getRootTreeName() const;

  /**
   * @brief Create a valid identity string for a specific behavior tree of this resource.
   * @param tree_name Name of one of the trees inside this resource's tree document. If empty, do not refer to a
   * specific behavior tree (identity won't be fully qualified).
   * @return Tree resource identity string.
   */
  TreeResourceIdentity createIdentityForTree(const std::string & tree_name = "") const;

  /**
   * @brief Find an installed behavior tree resource using a specific behavior tree name.
   *
   * This method searches the ament resource index for a tree resource whose `<tree_name>` component matches
   * @p tree_name. Unlike the identity-string constructor, this accepts just the tree name without requiring
   * `<tree_file_stem>`.
   *
   * @param tree_name The name of the behavior tree.
   * @param package_name Optional package name to narrow the search. If empty, all packages are searched.
   * @return Corresponding tree resource object.
   * @throws auto_apms_util::exceptions::ResourceError if no matching resource is found or if the match is ambiguous.
   */
  static TreeResource findByTreeName(const std::string & tree_name, const std::string & package_name = "");

  /**
   * @brief Find an installed behavior tree resource using an XML file stem.
   *
   * This method searches the ament resource index for a tree resource whose `<tree_file_stem>` component matches
   * @p file_stem. Unlike the identity-string constructor, this accepts just the file stem without requiring
   * `<tree_name>`.
   *
   * @param file_stem The stem (filename without extension) of the behavior tree XML file.
   * @param package_name Optional package name to narrow the search. If empty, all packages are searched.
   * @return Corresponding tree resource object.
   * @throws auto_apms_util::exceptions::ResourceError if no matching resource is found or if the match is ambiguous.
   */
  static TreeResource findByFileStem(const std::string & file_stem, const std::string & package_name = "");

private:
  std::string doc_root_tree_name_;
};

}  // namespace auto_apms_behavior_tree::core

/// @cond INTERNAL
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::core::TreeResourceIdentity>
{
  using Identity = auto_apms_behavior_tree::core::TreeResourceIdentity;
  static Node encode(const Identity & rhs)
  {
    Node node;
    node = rhs.str();
    return node;
  }
  static bool decode(const Node & node, Identity & rhs)
  {
    if (!node.IsScalar()) return false;
    rhs = Identity(node.Scalar());
    return true;
  }
};
}  // namespace YAML
/// @endcond
