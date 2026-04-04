// Copyright 2025 Robin Müller
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

#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_behavior_tree::core
{

// clang-format off

/**
 * @brief Struct that encapsulates the identity string for a registered behavior.
 *
 * A behavior resource identity string consists of three optional tokens:
 *
 * - `<category_name>` — The category of the behavior resource (e.g. `"tree"`, `"default"`).
 *
 * - `<package_name>` — The ROS 2 package that registered the behavior resource.
 *
 * - `<behavior_alias>` — The alias that uniquely identifies the behavior within its package.
 *
 * ## Supported identity formats
 *
 * Several short forms are supported to allow omitting tokens that are not needed for disambiguation. The `::` separator
 * is always used between `<package_name>` and `<behavior_alias>`, and `/` is used between `<category_name>` and the
 * rest of the identity.
 *
 * | Format | Description |
 * |---|---|
 * | `<category_name>/<package_name>::<behavior_alias>` | Fully qualified: category, package, and alias. |
 * | `<package_name>::<behavior_alias>` | Omit category — searches across all categories. Equivalent to `/<package_name>::<behavior_alias>`. |
 * | `::<behavior_alias>` | Omit both category and package — searches across all packages and categories. |
 * | `<behavior_alias>` | Same as `::<behavior_alias>`, but only when the alias itself does not contain `::`. |
 * | `<category_name>/::<behavior_alias>` | With category, but without package — narrows search to a specific category across all packages. |
 *
 * @note When the `<behavior_alias>` itself contains `::` (as is the case with TreeResourceIdentity where the alias is
 * `<tree_file_stem>::<tree_name>`), the bare alias form is **ambiguous** with the `<package_name>::<behavior_alias>`
 * form. In that case, you **must** provide the **first** `::` which separates the package name (which can be empty though) to avoid misinterpretation.
 *
 * ## Parsing rules
 *
 * 1. If the identity contains `/`, the substring before the **first** `/` is the `<category_name>` and everything
 *    after is processed as `<package_name>::<behavior_alias>`.
 *
 * 2. If the remaining string contains `::`, the substring before the **first** `::` is the `<package_name>` and
 *    everything after is the `<behavior_alias>`.
 *
 * 3. If no `::` is found, the entire remaining string is treated as the `<behavior_alias>` (package is empty).
 *
 * 4. A `default_category` may be provided to fill `<category_name>` when it is omitted or set to `"default"`.
 */
struct BehaviorResourceIdentity
// clang-format on
{
  /**
   * @brief Constructor of a behavior resource identity object.
   *
   * @p identity must be formatted like `<category_name>/<package_name>::<behavior_alias>`.
   * @param identity Identity string for a specific behavior resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  BehaviorResourceIdentity(const std::string & identity);

  /**
   * @brief Constructor of a behavior resource identity object.
   *
   * @p identity must be formatted like `<category_name>/<package_name>::<behavior_alias>`.
   * @param identity Identity string for a specific behavior resource.
   * @param default_category Name of the default category assigned if none is provided with the identity string.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  BehaviorResourceIdentity(const std::string & identity, const std::string & default_category);

  /**
   * @brief Constructor of a behavior resource identity object.
   *
   * @p identity must be formatted like `<category_name>/<package_name>::<behavior_alias>`.
   * @param identity C-style identity string for a specific behavior resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  BehaviorResourceIdentity(const char * identity);

  /**
   * @brief Constructor of an empty behavior resource identity object.
   *
   * The user must manually populate the member fields.
   */
  BehaviorResourceIdentity() = default;

  virtual ~BehaviorResourceIdentity() = default;

  bool operator==(const BehaviorResourceIdentity & other) const;

  bool operator<(const BehaviorResourceIdentity & other) const;

  /**
   * @brief Create the corresponding identity string.
   * @return Identity string for a behavior resource.
   */
  std::string str() const;

  /**
   * @brief Determine whether this behavior resource identity object is considered empty.
   * @return `true` if none of the member fields are set, `false` otherwise.
   */
  bool empty() const;

  /// Name of the category this behavior resource belongs to.
  std::string category_name;
  /// Name of the package that registers the behavior resource.
  std::string package_name;
  /// Alias for a single registered behavior.
  std::string behavior_alias;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Get all registered behavior resource identities.
 * @param include_categories Optional set of categories to include in the results. If empty, all categories are
 * included.
 * @param include_internal Optional flag whether to include behaviors marked as internal.
 * @param exclude_packages Optional set of package names to exclude from the search. If empty, all packages are
 * included.
 * @return Set of `BehaviorResourceIdentity` objects representing all registered behavior resources.
 */
std::set<BehaviorResourceIdentity> getBehaviorResourceIdentities(
  const std::set<std::string> & include_categories = {}, bool include_internal = false,
  const std::set<std::string> & exclude_packages = {});

/**
 * @brief Helper temlpate class for creating behavior resource abstractions.
 * @tparam T Resource identity type.
 */
template <class T, typename = std::enable_if_t<std::is_base_of_v<BehaviorResourceIdentity, T>>>
class BehaviorResourceTemplate
{
public:
  using Identity = T;

  /**
   * @brief Assemble a behavior resource using a BehaviorResourceIdentity.
   * @param search_identity Behavior resource identity object used for searching the corresponding resource.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given
   * identity.
   */
  BehaviorResourceTemplate(const Identity & search_identity);

  /**
   * @brief Assemble a behavior resource identified by a string.
   *
   * @p search_identity must be formatted like `<category_name>/<package_name>::<behavior_alias>`.
   *
   * @param search_identity Behavior resource identity string used for searching the corresponding resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity
   string.
   */
  BehaviorResourceTemplate(const std::string & search_identity);

  /**
   * @brief Assemble a behavior tree resource identified by a string.
   *
   * @p search_identity must be formatted like `<category_name>/<package_name>::<behavior_alias>`.
   *
   * @param search_identity C-style behavior resource identity string used for searching the corresponding resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity
   string.
   */
  BehaviorResourceTemplate(const char * search_identity);

  virtual ~BehaviorResourceTemplate() = default;

  /**
   * @brief Find an installed behavior resource using a specific behavior tree name.
   *
   * This method can be used as an alternative to passing an identity string to the constructor.
   *
   * @param behavior_alias Name of the desired resource. Is determined when registering the behavior with
   * `auto_apms_behavior_tree_register_behavior`.
   * @param package_name Optional package name provided to narrow down the search. If empty, search in all installed
   * packages.
   * @param category_name Optional category name provided to narrow down the search. If empty, search in all installed
   * packages.
   * @return Corresponding behavior resource object.
   * @throws auto_apms_util::exceptions::ResourceError if the corresponding behavior resource cannot be found using the
   * given arguments.
   */
  static BehaviorResourceTemplate find(
    const std::string & behavior_alias, const std::string & package_name = "", const std::string & category_name = "");

  /**
   * @brief Get the unique identity for this resource.
   * @return Identity object.
   */
  const Identity & getIdentity() const;

  /**
   * @brief Get the behavior build request associated with this resource.
   * @return Build request as a string.
   */
  const std::string & getBuildRequest() const;

  /**
   * @brief Get the fully qualified class name of the default build handler associated with this behavior resource.
   * @return Name of the default build handler.
   */
  const std::string & getDefaultBuildHandlerName() const;

  /**
   * @brief Get the entry point of this behavior resource.
   * @return Entry point as a string.
   */
  const std::string & getEntryPoint() const;

  /**
   * @brief Get the node manifest associated with this resource.
   * @return Node manifest object.
   */
  const NodeManifest & getNodeManifest() const;

protected:
  Identity unique_identity_;
  std::string build_request_file_path_;
  std::string build_request_;
  std::string default_build_handler_;
  std::string entry_point_;
  NodeManifest node_manifest_;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Class containing behavior resource data
 *
 * Behavior resources are registered by calling the CMake macro `auto_apms_behavior_tree_register_behavior` in the
 * CMakeLists.txt of a package. They can be discovered once the corresponding package has been installed to the ROS 2
 * workspace.
 *
 * The user may refer to a specific resource using an identity string that may contain the tokens `<category_name>`,
 * `<package_name>`, and `<behavior_alias>`. Depending on the number of registered resources, it might be
 * convenient to use shorter, less precise signatures. The formatting is described below:
 *
 * - `<category_name>/<package_name>::<behavior_alias>`
 *
 *   Fully qualified identity string of a specific behavior tree.
 *
 * - `<package_name>::<behavior_alias>` or `/<package_name>::<behavior_alias>`
 *
 *   **The category name is always optional**. If omitted, the resource is searched in all categories.
 *
 * - `::<behavior_alias>` or `<behavior_alias>`
 *
 *   Just like the category name, the package name is optional as well. If omitted, the resource is searched in all
 *   packages.
 *
 * @note The token <behavior_alias> is usually the stem of the file given to
 * `auto_apms_behavior_tree_register_behavior`. However, it is also possible to provide raw strings as arguments
 * (e.g. for referring to another resource). In this case, <behavior_alias> is determined to be the given string.
 *
 * ## Usage
 *
 * Given the user has specified a behavior using a YAML file, the CMake macro
 * `auto_apms_behavior_tree_register_behavior` must be called in the CMakeLists.txt of the parent package (for example
 * `my_package`) like this:
 *
 * ```cmake
 * auto_apms_behavior_tree_register_behavior(
 *     "behavior/my_behavior.yaml"
 * )
 * ```
 *
 * The macro automatically installs the given file and makes it discoverable using the above mentioned identities. In
 * the C++ source code, one may use this resource like this after the parent package `my_package` has been
 * installed:
 *
 * ```cpp
 * #include "auto_apms_behavior_tree_core/behavior.hpp"
 *
 * using namespace auto_apms_behavior_tree;
 *
 * // For example, use the fully qualified behavior resource identity signature
 * const std::string identity_string = "default/my_package::my_behavior";
 *
 * // You may use the proxy class for a tree resource identity
 * core::BehaviorResourceIdentity identity(identity_string);
 * core::BehaviorResource resource(identity);
 *
 * // Or instantiate the resource object directly from the corresponding identity string
 * core::BehaviorResource resource(identity_string);
 * ```
 */
class BehaviorResource : public BehaviorResourceTemplate<BehaviorResourceIdentity>
{
public:
  using BehaviorResourceTemplate::BehaviorResourceTemplate;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class T, typename U>
inline BehaviorResourceTemplate<T, U>::BehaviorResourceTemplate(const Identity & search_identity)
{
  if (search_identity.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Cannot create behavior resource with empty identity.");
  }

  std::set<std::string> search_packages;
  if (!search_identity.package_name.empty()) {
    search_packages.insert(search_identity.package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR);
  }

  size_t matching_count = 0;
  for (const auto & p : search_packages) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(
          _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, p, content, &base_path)) {
      for (const auto & line :
           auto_apms_util::splitString(content, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP)) {
        const std::vector<std::string> parts = auto_apms_util::splitString(
          line, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP, false);
        if (parts.size() != 6) {
          throw auto_apms_util::exceptions::ResourceError(
            "Invalid behavior tree resource file (Package: '" + p + "'). Invalid line: " + line + ".");
        }

        // Determine if resource is matching
        std::string found_category = parts[0];
        std::string found_alias = parts[1];
        if (search_identity.category_name.empty()) {
          // Disregard the category if not provided with the identity
          if (found_alias != search_identity.behavior_alias) {
            continue;
          }
        } else if (found_category != search_identity.category_name || found_alias != search_identity.behavior_alias) {
          continue;
        }

        // Found matching resource - Increase counter
        matching_count++;

        // Now fill the other member variables in case the resource matches (if match is not unique, error is thrown
        // later and the object is discarded)

        // Store behavior category
        unique_identity_.category_name = found_category;

        // Store behavior alias
        unique_identity_.behavior_alias = found_alias;

        // Store package name
        unique_identity_.package_name = p;

        // Store default build handler
        default_build_handler_ = parts[2];

        // Store build request
        build_request_file_path_ = base_path + "/" + parts[3];
        if (std::filesystem::is_regular_file(build_request_file_path_)) {
          std::ifstream file(build_request_file_path_);
          if (!file) {
            throw auto_apms_util::exceptions::ResourceError(
              "Failed to open behavior resource file '" + build_request_file_path_ + "'");
          }
          build_request_ = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        } else {
          build_request_file_path_ = "";
          build_request_ = parts[3];
        }

        // Store entry_point
        entry_point_ = parts[4];

        // Store node manifest paths
        std::vector<std::string> node_manifest_paths;
        for (const std::string & path : auto_apms_util::splitString(parts[5], ";", true)) {
          node_manifest_paths.push_back(std::filesystem::path(path).is_absolute() ? path : (base_path + "/" + path));
        }
        node_manifest_ = NodeManifest::fromFiles(node_manifest_paths);
      }
    }
  }

  if (matching_count == 0) {
    throw auto_apms_util::exceptions::ResourceError(
      "No behavior resource with identity '" + search_identity.str() + "' was registered.");
  }
  if (matching_count > 1) {
    throw auto_apms_util::exceptions::ResourceError(
      "Behavior resource identity '" + search_identity.str() + "' is ambiguous. You must be more precise.");
  }
}

template <class T, typename U>
inline BehaviorResourceTemplate<T, U>::BehaviorResourceTemplate(const std::string & identity)
: BehaviorResourceTemplate(Identity(identity))
{
}

template <class T, typename U>
inline BehaviorResourceTemplate<T, U>::BehaviorResourceTemplate(const char * identity)
: BehaviorResourceTemplate(std::string(identity))
{
}

template <class T, typename U>
inline BehaviorResourceTemplate<T, U> BehaviorResourceTemplate<T, U>::find(
  const std::string & behavior_alias, const std::string & package_name, const std::string & category_name)
{
  return BehaviorResourceTemplate(
    category_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP + package_name +
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + behavior_alias);
}

template <class T, typename U>
inline const T & BehaviorResourceTemplate<T, U>::getIdentity() const
{
  return unique_identity_;
}

template <class T, typename U>
inline const std::string & BehaviorResourceTemplate<T, U>::getBuildRequest() const
{
  return build_request_;
}

template <class T, typename U>
inline const std::string & BehaviorResourceTemplate<T, U>::getDefaultBuildHandlerName() const
{
  return default_build_handler_;
}

template <class T, typename U>
inline const std::string & BehaviorResourceTemplate<T, U>::getEntryPoint() const
{
  return entry_point_;
}

template <class T, typename U>
inline const NodeManifest & BehaviorResourceTemplate<T, U>::getNodeManifest() const
{
  return node_manifest_;
}

}  // namespace auto_apms_behavior_tree::core

/// @cond INTERNAL
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::core::BehaviorResourceIdentity>
{
  using Identity = auto_apms_behavior_tree::core::BehaviorResourceIdentity;
  inline static Node encode(const Identity & rhs)
  {
    Node node;
    node = rhs.str();
    return node;
  }
  inline static bool decode(const Node & node, Identity & rhs)
  {
    if (!node.IsScalar()) return false;
    rhs = Identity(node.Scalar());
    return true;
  }
};
}  // namespace YAML
/// @endcond
