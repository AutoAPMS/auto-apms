// Quick test to print node manifest resource identities

#include <iostream>

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
  std::cout << "Getting all node manifest resource identities..." << std::endl;

  auto identities = auto_apms_behavior_tree::core::getNodeManifestResourceIdentities();

  std::cout << "\nFound " << identities.size() << " node manifest resource identities:\n" << std::endl;

  for (const auto & identity : identities) {
    std::cout << "  - Package: '" << identity.package_name << "', Metadata ID: '" << identity.metadata_id
              << "', Full: '" << identity.str() << "'" << std::endl;
  }

  return 0;
}
