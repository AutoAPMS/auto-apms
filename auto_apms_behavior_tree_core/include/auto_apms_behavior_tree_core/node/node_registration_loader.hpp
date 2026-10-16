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

#include "auto_apms_behavior_tree_core/node/node_registration_interface.hpp"
#include "auto_apms_util/resource.hpp"
#include "rclcpp/macros.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief A `pluginlib::ClassLoader` specifically for loading installed behavior tree node plugins.
 */
class NodeRegistrationLoader : public auto_apms_util::PluginClassLoader<NodeRegistrationInterface>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NodeRegistrationLoader)

  /// Name of the package that contains the base class for this plugin loader.
  static const std::string BASE_PACKAGE_NAME;
  /// Name of the base class of all plugins to be loaded.
  static const std::string BASE_CLASS_NAME;

  /**
   * @brief NodeRegistrationLoader constructor.
   *
   * Parses the workspace for behavior tree node plugins and performs an ambiguity check that is supposed to detect
   * duplicate class names. If any duplicate class names are found, it throws an error.
   * @param exclude_packages Packages to exclude when searching for associated plugin resources.
   * @throw See PluginClassLoader::PluginClassLoader and PluginClassLoader::verifyUnambiguousClassNames.
   */
  NodeRegistrationLoader(const std::set<std::string> & exclude_packages = {});
};

}  // namespace auto_apms_behavior_tree::core