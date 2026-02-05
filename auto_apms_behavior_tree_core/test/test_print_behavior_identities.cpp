// Quick test to print behavior resource identities

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
