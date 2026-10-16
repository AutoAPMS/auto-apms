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

#include "auto_apms_behavior_tree/build_handler/build_handler_loader.hpp"

namespace auto_apms_behavior_tree
{

const std::string TreeBuildHandlerLoader::BASE_PACKAGE_NAME = "auto_apms_behavior_tree";
const std::string TreeBuildHandlerLoader::BASE_CLASS_NAME = "auto_apms_behavior_tree::TreeBuildHandlerFactoryInterface";

TreeBuildHandlerLoader::TreeBuildHandlerLoader(const std::set<std::string> & exclude_packages)
: PluginClassLoader(BASE_PACKAGE_NAME, BASE_CLASS_NAME, exclude_packages)
{
  verifyUnambiguousClassNames(BASE_PACKAGE_NAME, BASE_CLASS_NAME, exclude_packages);
}

}  // namespace auto_apms_behavior_tree
