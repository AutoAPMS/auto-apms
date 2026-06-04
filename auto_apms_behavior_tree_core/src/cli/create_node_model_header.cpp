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

#include <tinyxml2.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/xml_parsing.h"

using namespace auto_apms_behavior_tree;

const std::regex no_typed_setter_getter_types(
  "std::shared_ptr|std::unique_ptr|std::weak_ptr|BT::Any|BT::AnyTypeAllowed|std::string");
const std::vector<BT::NodeType> leaf_node_types{BT::NodeType::ACTION, BT::NodeType::SUBTREE};

int main(int argc, char ** argv)
{
  bool include_native_nodes = false;
  if (argc < 5) {
    std::cerr << "create_node_model_header: Missing inputs! The program requires: \n\t1.) The path to the node plugin "
                 "manifest YAML file.\n\t2.) The path to the nodes model XML file.\n\t3.) The name of the package "
                 "building the node model header.\n\t4.) The path of the output .hpp file.\n";
    std::cerr << "Usage: create_node_model_header <manifest_file> <model_file> <build_package_name> <header_path>.\n";
    return EXIT_FAILURE;
  }

  try {
    const std::filesystem::path manifest_file = std::filesystem::absolute(argv[1]);
    const std::filesystem::path model_file = std::filesystem::absolute(argv[2]);
    const std::string build_package_name = auto_apms_util::trimWhitespaces(argv[3]);
    const std::filesystem::path header_path = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[4]));
    if (build_package_name == "auto_apms_behavior_tree") include_native_nodes = true;

    // Ensure that arguments are not empty
    if (manifest_file.empty()) {
      throw std::runtime_error("Argument manifest_file must not be empty.");
    }
    if (model_file.empty()) {
      throw std::runtime_error("Argument model_file must not be empty.");
    }
    if (header_path.empty()) {
      throw std::runtime_error("Argument header_path must not be empty.");
    }

    // Ensure correct extensions
    if (header_path.extension() != ".hpp") {
      throw std::runtime_error("Output file '" + header_path.string() + "' has wrong extension. Must be '.hpp'.");
    }

    core::NodeManifest manifest;
    tinyxml2::XMLDocument model_doc;
    NodeModelMap model_map;

    const std::set<std::string> native_node_names = BT::BehaviorTreeFactory().builtinNodes();
    if (include_native_nodes) {
      if (
        model_doc.Parse(BT::writeTreeNodesModelXML(BT::BehaviorTreeFactory(), true).c_str()) !=
        tinyxml2::XMLError::XML_SUCCESS) {
        throw std::runtime_error(model_doc.ErrorStr());
      }
      model_map = core::TreeDocument::getNodeModel(model_doc, core::NodeManifest());
      for (const auto & [name, _] : model_map) {
        core::NodeRegistrationOptions opt;
        opt.class_name = "empty";
        manifest.add(name, opt);
      }
      manifest.remove(core::TreeDocument::SUBTREE_ELEMENT_NAME);
      model_doc.Clear();
    }

    manifest.merge(core::NodeManifest::fromFile(manifest_file));
    if (model_doc.LoadFile(model_file.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
      throw std::runtime_error(model_doc.ErrorStr());
    }
    model_map.merge(core::TreeDocument::getNodeModel(model_doc, manifest));

    // clang-format off
    std::ostringstream content;
    content << R"(// This header has been generated automatically. DO NOT CHANGE!

#pragma once

#include "behaviortree_cpp/basic_types.h"
#include "auto_apms_behavior_tree_core/convert.hpp"
#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"

namespace )" << build_package_name << R"(::model
{
)";
    // clang-format on
    for (const auto & [node_name, options] : manifest.map()) {
      NodeModelMap::const_iterator it = model_map.find(node_name);
      if (it == model_map.end()) continue;
      const NodeModel & model = it->second;
      const bool is_native_node = native_node_names.find(node_name) != native_node_names.end();
      const bool is_leaf = auto_apms_util::contains(leaf_node_types, model.type);
      const std::string base_class_name = is_leaf ? "LeafNodeModelType" : "NodeModelType";
      // clang-format off
      content << R"(
/// @brief )" << options.description << R"(
class )" << node_name << R"( : public auto_apms_behavior_tree::core::)" << base_class_name << R"(
{
friend class auto_apms_behavior_tree::core::TreeDocument::NodeElement;

using )" << base_class_name << "::" << base_class_name << R"(;

public:
/// @brief Information about the implemented ports.
static PortInfos ports()
{
PortInfos port_infos;
)";
      for (const NodePortInfo & info : model.port_infos) {
        content << "port_infos.insert(BT::CreatePort<"<< info.port_type <<">(BT::convertFromString<BT::PortDirection>(\"" << info.port_direction << "\"), \"" << info.port_name << "\", \"" << info.port_description << "\"));\n";
        if (info.port_has_default && !BT::TreeNode::isBlackboardPointer(info.port_default)) {
          content << "port_infos[\"" << info.port_name << "\"].setDefaultValue(BT::convertFromString<" << info.port_type << ">(\"" << info.port_default << "\"));\n";
        }
      }
      content << R"(return port_infos;
}

/// @brief Type of the behavior tree node.
static BT::NodeType type()
{
return BT::convertFromString<BT::NodeType>(")" << model.type << R"(");
}

/// @brief Name of the behavior tree node given during registration.
static std::string name()
{
return ")" << node_name << R"(";
}

/// @brief Name of the behavior tree node given during registration.
std::string getRegistrationName() const override final
{
return name();
}

)";
      if (!is_native_node) {
        content << R"(/// @brief Registration options for this node.
static RegistrationOptions registrationOptions()
{
return RegistrationOptions::decode(R"()" << options.encode() << ")\");" << R"(
}

)";
      }
      if (!is_leaf) {
        content << "AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_NON_LEAF_THISREF_METHODS(" << node_name << ")\n";
      }
      content << "AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_LEAF_THISREF_METHODS(" << node_name << ")\n";
      for (const NodePortInfo & info : model.port_infos) {
        content << R"(
/// @brief Setter for port ')" << info.port_name << "' (" << BT::toStr(info.port_direction) << R"().
///
/// )" << info.port_description << R"(
)" << node_name << " & set_" << info.port_name << "(const std::string & str";
        if (info.port_has_default) {
          content << " = \"" << info.port_default << "\")";
        } else {
          content << ")";
        }
        content << R"(
{
return setPorts({{")" << info.port_name << R"(", str}});
}

/// @brief Getter for port ')" << info.port_name << "' (" << BT::toStr(info.port_direction) << R"().
///
/// )" << info.port_description << R"(
std::string get_)" << info.port_name << R"(_str() const
{
return getPorts().at(")" << info.port_name << R"(");
}
)";
        if (std::regex_search(info.port_type, no_typed_setter_getter_types)) continue;
        content << R"(
/// @brief Setter for port ')" << info.port_name << "' (" << BT::toStr(info.port_direction) << R"().
///
/// )" << info.port_description << R"(
)" << node_name << " & set_" << info.port_name << "(const " << info.port_type << " & val)" << R"(
{
return setPorts({{")" << info.port_name << R"(", BT::toStr(val)}});
}

/// @brief Getter for port ')" << info.port_name << "' (" << BT::toStr(info.port_direction) << R"().
///
/// )" << info.port_description << R"(
)" << info.port_type << " get_" << info.port_name << R"(() const
{
return BT::convertFromString<)" << info.port_type << ">(get_" << info.port_name << R"(_str());
}
)";
        // clang-format on
      }
      content << "};\n";
    }
    content << "\n}";

    std::ofstream out_stream(header_path);
    if (out_stream.is_open()) {
      out_stream << content.str();
      out_stream.close();
    } else {
      throw std::runtime_error("Error opening node model header file '" + header_path.string() + "'");
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR (create_node_model_header): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
