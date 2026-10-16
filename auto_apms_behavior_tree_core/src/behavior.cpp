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

#include "auto_apms_behavior_tree_core/behavior.hpp"

namespace auto_apms_behavior_tree::core
{

bool isInternalBehaviorCategory(const std::string & category_name)
{
  const char * suffix = _AUTO_APMS_BEHAVIOR_TREE_CORE__INTERNAL_BEHAVIOR_CATEGORY_SUFFIX;
  size_t suffix_len = std::strlen(suffix);
  return category_name.size() >= suffix_len &&
         category_name.compare(category_name.size() - suffix_len, suffix_len, suffix) == 0;
}

BehaviorResourceIdentity::BehaviorResourceIdentity(const std::string & identity)
{
  if (identity.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior resource identity string must not be empty.");
  }
  std::string resource_part;
  if (std::size_t pos = identity.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP);
      pos == std::string::npos) {
    category_name = "";
    resource_part = identity;
  } else {
    category_name = identity.substr(0, pos);
    resource_part =
      identity.substr(pos + std::string(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP).size());
  }
  if (resource_part.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior resource identity string '" + identity + "' is invalid: You must specify more than just the category.");
  }
  if (std::size_t pos = resource_part.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP);
      pos == std::string::npos) {
    // If only a single token is given, assume it's behavior_alias
    package_name = "";
    behavior_alias = resource_part;
  } else {
    package_name = resource_part.substr(0, pos);
    behavior_alias =
      resource_part.substr(pos + std::string(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP).size());
  }
  if (package_name.empty() && behavior_alias.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior resource identity string '" + identity +
      "' is invalid. Package name and behavior alias must not be empty.");
  }
}

BehaviorResourceIdentity::BehaviorResourceIdentity(const std::string & identity, const std::string & default_category)
: BehaviorResourceIdentity(identity)
{
  // If no category is explicitly specified, use the given default one
  if (category_name.empty() || category_name == _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY) {
    category_name = default_category;
  }
}

BehaviorResourceIdentity::BehaviorResourceIdentity(const char * identity)
: BehaviorResourceIdentity(std::string(identity))
{
}

bool BehaviorResourceIdentity::operator==(const BehaviorResourceIdentity & other) const { return str() == other.str(); }

bool BehaviorResourceIdentity::operator<(const BehaviorResourceIdentity & other) const { return str() < other.str(); }

std::string BehaviorResourceIdentity::str() const
{
  std::string str;
  if (!category_name.empty()) {
    str += category_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP;
  }
  str += package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + behavior_alias;
  return str;
}

bool BehaviorResourceIdentity::empty() const { return package_name.empty() && behavior_alias.empty(); }

std::set<BehaviorResourceIdentity> getBehaviorResourceIdentities(
  const std::set<std::string> & include_categories, bool include_internal,
  const std::set<std::string> & exclude_packages)
{
  std::set<BehaviorResourceIdentity> identities;
  for (const auto & p : auto_apms_util::getPackagesWithResourceType(
         _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, exclude_packages)) {
    const ament_index_cpp::PathWithResource resource =
      ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, p);
    if (resource.resourcePath) {
      const std::string & content = resource.contents;
      for (const auto & line :
           auto_apms_util::splitString(content, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP)) {
        const std::vector<std::string> parts = auto_apms_util::splitString(
          line, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP, false);
        if (parts.size() > 1) {
          BehaviorResourceIdentity i;
          i.category_name = parts[0];
          i.package_name = p;
          i.behavior_alias = parts[1];
          if (include_categories.empty()) {
            if (!include_internal && isInternalBehaviorCategory(i.category_name))
              continue;  // Skip identities in the internal category if not requested otherwise
          } else {
            if (include_categories.find(i.category_name) == include_categories.end())
              continue;  // Skip identities not in the specified include categories
          }
          identities.insert(i);
        }
      }
    }
  }
  return identities;
}

}  // namespace auto_apms_behavior_tree::core
