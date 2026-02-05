// Copyright 2026 Robin Müller
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

#include <iostream>

#include "auto_apms_behavior_tree_core/behavior.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
  std::cout << "Getting all behavior resource identities (including internal)..." << std::endl;

  auto identities = auto_apms_behavior_tree::core::getBehaviorResourceIdentities({}, true);

  std::cout << "\nFound " << identities.size() << " behavior resource identities:\n" << std::endl;

  for (const auto & identity : identities) {
    std::cout << "  - Category: '" << identity.category_name << "', Package: '" << identity.package_name
              << "', Alias: '" << identity.behavior_alias << "', Full: '" << identity.str() << "'" << std::endl;
  }

  std::cout << "\n\nGetting non-internal behavior resource identities only..." << std::endl;

  auto non_internal_identities = auto_apms_behavior_tree::core::getBehaviorResourceIdentities({}, false);

  std::cout << "\nFound " << non_internal_identities.size() << " non-internal behavior resource identities:\n"
            << std::endl;

  for (const auto & identity : non_internal_identities) {
    std::cout << "  - Category: '" << identity.category_name << "', Package: '" << identity.package_name
              << "', Alias: '" << identity.behavior_alias << "', Full: '" << identity.str() << "'" << std::endl;
  }

  return 0;
}
