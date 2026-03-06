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

#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"

#include <tinyxml2.h>

#include <filesystem>

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_behavior_tree::core
{

TreeResourceIdentity::TreeResourceIdentity(const std::string & identity)
: BehaviorResourceIdentity(identity, _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE)
{
  std::vector<std::string> tokens =
    auto_apms_util::splitString(behavior_alias, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP, true);

  // If only a single token is given, assume it's file_stem
  if (tokens.size() > 1) {
    file_stem = tokens[0];
    tree_name = tokens[1];
  } else if (tokens.size() > 0) {
    file_stem = tokens[0];
    tree_name = "";
  }

  if (file_stem.empty() && tree_name.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior tree resource identity string '" + identity +
      "' is invalid. It's not allowed to omit both <tree_file_stem> and <tree_name>.");
  }
}

TreeResourceIdentity::TreeResourceIdentity(const char * identity) : TreeResourceIdentity(std::string(identity)) {}

TreeResource::TreeResource(const TreeResourceIdentity & search_identity) : BehaviorResourceTemplate(search_identity)
{
  // Fill the tree specific fields for the unique identity
  const std::vector<std::string> & tokens = auto_apms_util::splitString(
    unique_identity_.behavior_alias, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP, false);

  if (tokens.size() != 2) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Unique tree resource identity string '" + unique_identity_.str() +
      "' is invalid. Behavior alias must be <tree_file_stem>" +
      _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + "<tree_name>.");
  }

  unique_identity_.file_stem = tokens[0];
  unique_identity_.tree_name = tokens[1];

  // Verify that the file is ok
  TreeDocument doc;
  try {
    doc.mergeFile(build_request_file_path_, true);
  } catch (const std::exception & e) {
    throw auto_apms_util::exceptions::ResourceError(
      "Failed to create TreeResource with identity '" + unique_identity_.str() + "' because tree file " +
      build_request_file_path_ + " cannot be parsed: " + e.what());
  }

  // Verify that the tree <tree_name> specified by the identity string is actually present
  if (!unique_identity_.tree_name.empty()) {
    if (!auto_apms_util::contains(doc.getAllTreeNames(), unique_identity_.tree_name)) {
      throw auto_apms_util::exceptions::ResourceError(
        "Cannot create TreeResource with identity '" + unique_identity_.str() + "' because '" +
        unique_identity_.tree_name + "' does not exist in tree file " + build_request_file_path_ + ".");
    }
  }

  // Save the root tree name if available
  if (doc.hasRootTreeName()) {
    doc_root_tree_name_ = doc.getRootTreeName();
  }
}

TreeResource::TreeResource(const std::string & search_identity) : TreeResource(TreeResourceIdentity(search_identity)) {}

TreeResource::TreeResource(const char * search_identity) : TreeResource(std::string(search_identity)) {}

TreeResource TreeResource::findByTreeName(const std::string & tree_name, const std::string & package_name)
{
  return TreeResource(
    package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP +
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + tree_name);
}

TreeResource TreeResource::findByFileStem(const std::string & file_name, const std::string & package_name)
{
  return TreeResource(
    package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + file_name +
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP);
}

bool TreeResource::hasRootTreeName() const
{
  return !unique_identity_.tree_name.empty() || !doc_root_tree_name_.empty();
}

std::string TreeResource::getRootTreeName() const
{
  if (!unique_identity_.tree_name.empty()) return unique_identity_.tree_name;

  // If <tree_name> wasn't provided, look for root tree attribute in XML file
  if (!doc_root_tree_name_.empty()) return doc_root_tree_name_;

  // Root tree cannot be determined
  throw auto_apms_util::exceptions::ResourceError(
    "Cannot get root tree name of tree resource '" + unique_identity_.str() +
    "'. Since there is no XML attribute named '" + TreeDocument::ROOT_TREE_ATTRIBUTE_NAME +
    "' and the resource identity doesn't specify <tree_name>, the root tree is unknown.");
}

TreeResourceIdentity TreeResource::createIdentityForTree(const std::string & tree_name) const
{
  TreeResourceIdentity i;
  i.package_name = unique_identity_.package_name;
  i.file_stem = unique_identity_.file_stem;
  i.tree_name = tree_name;
  return i;
}

}  // namespace auto_apms_behavior_tree::core