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

#include <filesystem>
#include <iostream>
#include <set>

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cerr << "create_node_manifest: Missing inputs! The program requires: \n\t1.) the yaml "
                 "node manifest files (separated by ';').\n\t2.) Build information for nodes supposed to be "
                 "registered during build time (List of '<class_name>@<library_build_path>@<registration_type>' "
                 "separated by ';').\n\t3.) The name of the package that provides the build targets.\n\t4.) Output "
                 "file for the complete node plugin manifest.\n\t";
    std::cerr << "Usage: create_node_manifest <manifest_files> <build_infos> <build_package_name> "
                 "<output_file>\n";
    return EXIT_FAILURE;
  }

  try {
    std::vector<std::string> manifest_files;
    for (const auto & path : auto_apms_util::splitString(argv[1], ";")) {
      manifest_files.push_back(std::filesystem::absolute(path).string());
    }
    const std::vector<std::string> build_infos = auto_apms_util::splitString(argv[2], ";");
    const std::string build_package_name = argv[3];
    const std::filesystem::path output_file = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[4]));

    // Ensure that arguments are not empty
    if (manifest_files.empty()) {
      throw std::runtime_error("Argument manifest_files must not be empty");
    }
    if (output_file.empty()) {
      throw std::runtime_error("Argument output_file must not be empty.");
    }

    // Ensure correct extensions
    if (output_file.extension() != ".yaml") {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.yaml'.");
    }

    // Interpret build info
    std::map<std::string, std::string> library_paths_build_package;
    std::map<std::string, std::string> registration_types_build_package;
    std::map<std::string, std::string> reserved_names;
    for (const auto & build_info : build_infos) {
      std::vector<std::string> parts = auto_apms_util::splitString(build_info, "@");
      if (parts.size() != 3) {
        throw std::runtime_error(
          "Invalid build info entry ('" + build_info +
          "'). Expected format: '<class_name>@<library_path>@<registration_type>'.");
      }
      const std::string & class_name = parts[0];
      const std::string & build_path = parts[1];
      const std::string & registration_type = parts[2];
      if (library_paths_build_package.find(class_name) != library_paths_build_package.end()) {
        throw std::runtime_error("Node class '" + class_name + "' is specified multiple times in build infos.");
      }
      library_paths_build_package[class_name] = build_path;              // {class_name: build_path}
      registration_types_build_package[class_name] = registration_type;  // {class_name: registration_type}
      reserved_names[class_name] = build_package_name;                   // Additional names for the ambiguity check
    }

    // Construct manifest object from input files
    core::NodeManifest output_manifest = core::NodeManifest::fromFiles(manifest_files);

    /**
     * Trying to construct NodeRegistrationLoader will work completely fine during build time for all packages
     * EXCEPT the original auto_apms_behavior_tree_core package. The constructor will throw
     * pluginlib::ClassLoaderException if auto_apms_behavior_tree_core is being built for the first time, because the
     * pluginlib::ClassLoader constructor initially checks wether the package containing the base class is installed.
     * Therefore we MUST avoid triggering this script during build time of auto_apms_behavior_tree_core (Theoretically
     * we could use this script in auto_apms_behavior_tree_core if we only used locally built node plugins, but all
     * standard nodes are located in auto_apms_behavior_tree).
     */

    using Loader = auto_apms_util::PluginClassLoader<core::NodeRegistrationInterface>;

    std::set<std::string> exclude_build_package{build_package_name};
    std::set<std::string> other_packages_with_node_plugins;
    try {
      // Exclude the build package from the list of packages to be searched for resources, because resources from this
      // package are not installed yet. Instead we hold additional build information in the build_infos variable.
      other_packages_with_node_plugins = auto_apms_util::getPackagesWithPluginResources(exclude_build_package);
    } catch (const auto_apms_util::exceptions::ResourceError & e) {
      // Allow assembling library paths also if no other packages register node resources (Only using resources of the
      // build package).
      other_packages_with_node_plugins = {};
    }
    std::unique_ptr<Loader> loader_ptr = nullptr;
    if (!other_packages_with_node_plugins.empty()) {
      // When creating the node plugin loader, reserve the class names this package is building when checking for
      // ambiguous declarations. This forces us to use the auto_apms_util::PluginClassLoader constructor.
      Loader * ptr = new Loader(
        Loader::makeUnambiguousPluginClassLoader(
          core::NodeRegistrationLoader::BASE_PACKAGE_NAME, core::NodeRegistrationLoader::BASE_CLASS_NAME,
          exclude_build_package, reserved_names));
      loader_ptr = std::unique_ptr<Loader>(ptr);
    }

    // Determine shared libraries associated with the node classes required by the manifest files
    std::set<std::string> library_paths;
    std::map<std::string, std::string> registration_types;  // class_name -> registration_type
    for (const auto & [node_name, params] : output_manifest.map()) {
      // Look for class in the build package
      if (library_paths_build_package.find(params.class_name) != library_paths_build_package.end()) {
        library_paths.insert(library_paths_build_package[params.class_name]);
        registration_types[params.class_name] = registration_types_build_package[params.class_name];
        continue;
      }

      // Look for class in other already installed packages if it is not provided by the build package
      if (loader_ptr != nullptr && loader_ptr->isClassAvailable(params.class_name)) {
        library_paths.insert(loader_ptr->getClassLibraryPath(params.class_name));
        registration_types[params.class_name] = loader_ptr->getClassType(params.class_name);
        continue;
      }

      // There is no shared library associated with the class name we're looking for
      throw std::runtime_error(
        "Class '" + params.class_name + "' (associated with node '" + node_name +
        "') cannot be found. It's not being built by this package (" + build_package_name +
        ") and is also not provided by any other package installed on your machine. For a node to be discoverable, "
        "one must register it using auto_apms_behavior_tree_register_nodes() in the "
        "CMakeLists.txt of a ROS 2 package.");
    }

    // Save the manifest (Merged multiple files into s single one)
    output_manifest.toFile(output_file);

    // Print two lines to stdout:
    // Line 1: shared library paths (semicolon-separated)
    // Line 2: registration type mappings (semicolon-separated class_name=registration_type)
    bool first = true;
    for (const auto & path : library_paths) {
      if (!first) std::cout << ';';
      std::cout << path;
      first = false;
    }
    std::cout << '\n';
    first = true;
    for (const auto & [class_name, reg_type] : registration_types) {
      if (!first) std::cout << ';';
      std::cout << class_name << '=' << reg_type;
      first = false;
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR (create_node_manifest): " << e.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
