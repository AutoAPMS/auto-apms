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

#include "auto_apms_util/resource.hpp"

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/string.hpp"
#include "pluginlib/class_loader.hpp"

namespace auto_apms_util
{

const std::string PLUGIN_RESOURCE_TYPE = _AUTO_APMS_UTIL__RESOURCE_TYPE_NAME__PLUGINLIB;

std::set<std::string> getPackagesWithResourceType(
  const std::string & resource_type, const std::set<std::string> & exclude_packages)
{
  std::set<std::string> packages;
  for (const auto & [package, _] : ament_index_cpp::get_resources_by_name(resource_type)) {
    packages.insert(package);
  }
  if (const std::set<std::string> common = getCommonElements(packages, exclude_packages); !common.empty()) {
    for (const std::string & package_to_exclude : common) packages.erase(package_to_exclude);
  }
  return packages;
}

std::set<std::string> getPackagesWithPluginResources(const std::set<std::string> & exclude_packages)
{
  return getPackagesWithResourceType(PLUGIN_RESOURCE_TYPE, exclude_packages);
}

std::string getPluginXMLPath(const std::string & package)
{
  const ament_index_cpp::PathWithResource resource = ament_index_cpp::get_resource(PLUGIN_RESOURCE_TYPE, package);
  if (resource.resourcePath) {
    std::vector<std::string> paths = splitString(resource.contents, "\n");
    if (paths.size() != 1) {
      throw exceptions::ResourceError(
        "Invalid '" + PLUGIN_RESOURCE_TYPE + "' resource marker file installed by package '" + package +
        "'. Must contain a single line with a relative path to the plugins.xml "
        "manifest file with respect to the package's install prefix.");
    }
    return resource.resourcePath->string() + '/' + paths[0];
  }
  throw exceptions::ResourceError(
    "Cannot find a plugin.xml file in package '" + package + "' (Plugin resource type is: '" + PLUGIN_RESOURCE_TYPE +
    "').");
}

std::vector<std::string> collectPluginXMLPaths(const std::set<std::string> & exclude_packages)
{
  std::vector<std::string> xml_paths;
  for (const std::string & package : getPackagesWithResourceType(PLUGIN_RESOURCE_TYPE, exclude_packages)) {
    xml_paths.push_back(getPluginXMLPath(package));
  }
  return xml_paths;
}

}  // namespace auto_apms_util
