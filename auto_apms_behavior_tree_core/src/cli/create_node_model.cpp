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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_interface.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_util/logging.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "class_loader/class_loader.hpp"

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cerr << "create_node_model: Missing inputs! The program requires: \n\t1.) The path to the node plugin "
                 "manifest.\n\t2. The exhaustive list of libraries to be loaded by ClassLoader (Separated by "
                 "';').\n\t3.) Registration type mappings (Separated by ';', format: "
                 "'class_name=registration_type').\n\t4.) The xml file to store the model.\n";
    std::cerr << "Usage: create_node_model <manifest_file> <library_paths> <registration_types> <output_file>.\n";
    return EXIT_FAILURE;
  }

  try {
    const std::filesystem::path manifest_file = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[1]));
    const std::vector<std::string> library_paths = auto_apms_util::splitString(argv[2], ";", true);
    const std::vector<std::string> registration_type_entries = auto_apms_util::splitString(argv[3], ";", true);
    const std::filesystem::path output_file = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[4]));

    // Build class_name -> registration_type mapping
    std::map<std::string, std::string> registration_type_map;
    for (const auto & entry : registration_type_entries) {
      const auto eq_pos = entry.find('=');
      if (eq_pos == std::string::npos) {
        throw std::runtime_error(
          "Invalid registration type entry '" + entry + "'. Expected format: 'class_name=registration_type'.");
      }
      registration_type_map[entry.substr(0, eq_pos)] = entry.substr(eq_pos + 1);
    }

    if (!std::filesystem::exists(manifest_file)) {
      throw std::runtime_error("File manifest_file must exist.");
    }
    if (library_paths.empty()) {
      throw std::runtime_error("Argument library_paths must not be empty.");
    }
    if (!output_file.has_filename()) {
      throw std::runtime_error("Output file path must include a filename.");
    }

    // Ensure correct extensions
    if (output_file.extension() != ".xml") {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'.");
    }

    const rclcpp::Logger logger = rclcpp::get_logger("create_node_model__" + output_file.stem().string());

    BT::BehaviorTreeFactory factory;
    const auto manifest = core::NodeManifest::fromFile(manifest_file.string());

    /**
     * NOTE: We have to use the low level class loader here because the pluginlib::ClassLoader API doesn't allow
     * customizing the internal node/library allocation map.
     */

    // Instantiate loaders for all libraries in library_paths (We don't use class_loader::MultiLibraryClassLoader
    // because we want to keep track of the libraries that the nodes come from for debugging purposes)
    std::vector<std::unique_ptr<class_loader::ClassLoader>> class_loaders;
    for (const auto & path : library_paths) {
      try {
        class_loaders.push_back(std::make_unique<class_loader::ClassLoader>(path));
      } catch (const std::exception & e) {
        throw std::runtime_error("Failed to load library '" + path + "': " + e.what());
      }
    }

    // Walk manifest and register all plugins with BT::BehaviorTreeFactory
    for (const auto & [node_name, params] : manifest.map()) {
      // Look up the registration type from the mapping provided by create_node_manifest
      const auto reg_type_it = registration_type_map.find(params.class_name);
      if (reg_type_it == registration_type_map.end()) {
        throw std::runtime_error(
          "Node '" + node_name + "' (Class: " + params.class_name +
          ") has no registration type mapping. Please rebuild the package.");
      }
      const std::string & registration_type = reg_type_it->second;

      class_loader::ClassLoader * loader = nullptr;
      for (const auto & l : class_loaders) {
        if (l->isClassAvailable<core::NodeRegistrationInterface>(registration_type)) {
          loader = l.get();
        }
      }

      if (!loader) {
        throw std::runtime_error(
          "Node '" + node_name + "' (Class: " + params.class_name +
          ") cannot be registered, because the registration class '" + registration_type +
          "' could not be found in any loaded library. Check that the class name is spelled correctly and "
          "the node is registered by calling auto_apms_behavior_tree_register_nodes() in the CMakeLists.txt of the "
          "corresponding package. Also make sure that you called the "
          "AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE macro or PLUGINLIB_EXPORT_CLASS in the source file.");
      }

      RCLCPP_DEBUG(
        logger, "Registering behavior tree node '%s' (Class: %s) from library %s.", node_name.c_str(),
        params.class_name.c_str(), loader->getLibraryPath().c_str());

      try {
        const auto plugin_instance = loader->createUniqueInstance<core::NodeRegistrationInterface>(registration_type);
        rclcpp::Node::SharedPtr node = nullptr;
        rclcpp::CallbackGroup::SharedPtr group = nullptr;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor = nullptr;
        core::RosNodeContext ros_node_context(
          node, group, executor, params);  // Values don't matter when not instantiating it
        plugin_instance->registerWithBehaviorTreeFactory(factory, node_name, &ros_node_context);
      } catch (const std::exception & e) {
        throw std::runtime_error(
          "Failed to register node '" + node_name + " (Class: " + params.class_name + ")': " + e.what());
      }
    }

    // Generate node model XML from the factory
    const std::string model_xml = BT::writeTreeNodesModelXML(factory);

    // Parse the XML into a document to extract the NodeModelMap later
    tinyxml2::XMLDocument model_doc;
    if (model_doc.Parse(model_xml.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
      throw std::runtime_error("Error parsing the generated node model XML: " + std::string(model_doc.ErrorStr()));
    }

    // Convert to NodeModelMap with hidden ports applied
    auto model_map = core::TreeDocument::getNodeModel(model_doc, manifest);

    // Create a TreeDocument and add the filtered node model
    core::TreeDocument doc;
    doc.addNodeModel(model_map);

    // Write to file
    doc.writeToFile(output_file.string());
  } catch (const std::exception & e) {
    std::cerr << "ERROR (create_node_model): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
