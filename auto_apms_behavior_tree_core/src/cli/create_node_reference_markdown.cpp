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

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/basic_types.h"

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  bool print_help = false;
  if (argc > 1) {
    const std::string arg(argv[1]);
    print_help = "-h" == arg || "--help" == arg;
  }
  if (print_help || argc < 2) {
    std::cerr << "create_node_reference_markdown: The program accepts: \n\t1.) Path to the markdown file to write "
                 "to.\n\t2.) Node manifest identities of installed resources used to "
                 "create the documentation. All "
                 "arguments after the first one are interpreted as resource identities.\n";
    std::cerr << "Usage: create_node_reference_markdown <output_file> [<node_manifest_identity> ...]\n";
    return EXIT_FAILURE;
  }

  try {
    const std::filesystem::path output_file = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[1]));
    if (output_file.empty()) {
      throw std::runtime_error("Argument output_file must not be empty.");
    }

    // Ensure correct extensions
    if (output_file.extension() != ".md") {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.md'.");
    }

    core::NodeManifest node_manifest;
    std::vector<std::string> input_packages;
    for (int i = 2; i < argc; ++i) {
      const std::string package_name = auto_apms_util::splitString(argv[i], "::")[0];
      input_packages.push_back(package_name);
      if (package_name == "include_native") continue;

      // Throws if there are registration names which exist multiple times
      node_manifest.merge(core::NodeManifest::fromResource(auto_apms_util::trimWhitespaces(argv[i])), false);
    }
    const bool include_native = auto_apms_util::contains(input_packages, std::string("include_native"));
    const std::string native_package_name = "auto_apms_behavior_tree (BehaviorTree.CPP)";
    for (std::string & ele : input_packages) {
      if (ele == "include_native") ele = native_package_name;
    }

    // Search all packages
    core::NodeRegistrationLoader::SharedPtr node_loader_ptr = core::NodeRegistrationLoader::make_shared();
    core::TreeDocument doc(core::TreeDocument::BTCPP_FORMAT_DEFAULT_VERSION, node_loader_ptr);
    doc.registerNodes(node_manifest);
    std::map<std::string, std::string> package_for_class = node_loader_ptr->getClassPackageMap();
    const NodeModelMap model_map = doc.getNodeModel(include_native);
    core::NodeRegistrationOptions native_node_options;
    native_node_options.class_name = "None";

    // Verify that package information is available
    const std::set<std::string> native_node_names = BT::BehaviorTreeFactory().builtinNodes();
    for (const auto & [registration_name, _] : model_map) {
      if (!node_manifest.contains(registration_name)) {
        if (native_node_names.find(registration_name) != native_node_names.end()) {
          node_manifest.add(registration_name, native_node_options);
          package_for_class[node_manifest[registration_name].class_name] = native_package_name;
        } else {
          throw std::runtime_error("Package for node '" + registration_name + "' is unknown.");
        }
      }
    }

    auto lowerize = [](const std::string & str) {
      std::string ret(str);
      std::transform(ret.begin(), ret.end(), ret.begin(), [](unsigned char c) { return std::tolower(c); });
      return ret;
    };

    std::ostringstream content;
    content << R"(<!-- markdownlint-disable MD024 MD041 MD060 -->
| Registration Name | Class Name | Package |
| :--- | :---: | :---: |)";
    for (const std::string & package_name : input_packages) {
      for (const auto & [registration_name, model] : model_map) {
        const core::NodeRegistrationOptions & options = node_manifest[registration_name];
        if (package_name != package_for_class[options.class_name]) continue;
        content << "\n| [" << registration_name << "](#" << lowerize(registration_name) << ") | `" << options.class_name
                << "` | " << package_for_class[options.class_name] << " |";
      }
    }
    content << "\n";
    for (const std::string & package_name : input_packages) {
      content << "\n## " << package_name << "\n";
      for (const auto & [registration_name, model] : model_map) {
        const core::NodeRegistrationOptions & options = node_manifest[registration_name];
        if (package_name != package_for_class[options.class_name]) continue;
        // clang-format off
        content << R"(
)" << "### " << registration_name << R"(
)";
        if (options.class_name != "None") {
          content << R"(
**Plugin Class:** `)" << options.class_name << R"(`
)";
        }
        content << R"(
**C++ Model:** `)" << auto_apms_util::splitString(package_name, " ")[0] << "::" << registration_name << R"(`

**Node Type:** `)" << BT::toStr(model.type) << R"(`

**Description:** )" << (options.description.empty() ? "No description available." : options.description) << R"(
)";
        if (model.port_infos.empty()) {
          content << "\n*This node doesn't have any ports.*\n";
          continue;
        }
        std::vector<NodePortInfo> inputs;
        std::vector<NodePortInfo> outputs;
        std::vector<NodePortInfo> inouts;
        for (const NodePortInfo & port_info : model.port_infos) {
          switch (port_info.port_direction) {
            case BT::PortDirection::INPUT:
              inputs.push_back(port_info);
              break;
            case BT::PortDirection::OUTPUT:
              outputs.push_back(port_info);
              break;
            case BT::PortDirection::INOUT:
              inouts.push_back(port_info);
              break;
          }
        }
        if (!inputs.empty()) {
          content << R"(
#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
)";
          for (const NodePortInfo & port_info : inputs) {
            content << "| **" << port_info.port_name << "** | `" << port_info.port_type << "` | " << (port_info.port_has_default ? port_info.port_default : "❌") << " | " << port_info.port_description << " |\n";
          }
        }
        if (!outputs.empty()) {
          content << R"(
#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
)";
          for (const NodePortInfo & port_info : outputs) {
            content << "| **" << port_info.port_name << "** | `" << port_info.port_type << "` | " << (port_info.port_has_default ? port_info.port_default : "❌") << " | " << port_info.port_description << " |\n";
          }
        }
        if (!inouts.empty()) {
          content << R"(
#### Bidirectional Ports

| Port Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
)";
          for (const NodePortInfo & port_info : inouts) {
            content << "| **" << port_info.port_name << "** | `" << port_info.port_type << "` | " << (port_info.port_has_default ? port_info.port_default : "❌") << " | " << port_info.port_description << " |\n";
          }
        }
        // clang-format on
      }
    }

    std::ofstream out_stream(output_file);
    if (out_stream.is_open()) {
      out_stream << content.str();
      out_stream.close();
    } else {
      throw std::runtime_error("Error opening markdown output file '" + output_file.string() + "'");
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR (create_node_reference_markdown): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}