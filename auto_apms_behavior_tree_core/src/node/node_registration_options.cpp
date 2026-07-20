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

#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"

namespace auto_apms_behavior_tree::core
{

bool NodeRegistrationOptions::valid() const { return !class_name.empty(); }

bool NodeRegistrationOptions::operator==(const NodeRegistrationOptions & other) const
{
  // Compare by canonical YAML serialization. This is the single source of truth for registration-options equality:
  // it covers every field (including the free-form 'extra' node) without requiring per-field comparison and keeps the
  // semantics identical on both sides of the comparison.
  const auto fingerprint = [](const NodeRegistrationOptions & options) {
    YAML::Emitter emitter;
    emitter << YAML::Flow << YAML::convert<NodeRegistrationOptions>::encode(options);
    return std::string(emitter.c_str());
  };
  return fingerprint(*this) == fingerprint(other);
}

bool NodeRegistrationOptions::operator!=(const NodeRegistrationOptions & other) const { return !(*this == other); }

}  // namespace auto_apms_behavior_tree::core

/// @cond INTERNAL
namespace YAML
{

Node convert<auto_apms_behavior_tree::core::NodeRegistrationOptions>::encode(const Options & rhs)
{
  Node node(NodeType::Map);
  // Note: We intentionally do not encode the 'parent' field, since we directly evaluate it during decoding
  node[Options::PARAM_NAME_CLASS] = rhs.class_name;
  node[Options::PARAM_NAME_DESCRIPTION] = rhs.description;
  node[Options::PARAM_NAME_ROS2TOPIC] = rhs.topic;
  node[Options::PARAM_NAME_PORT_ALIAS] = rhs.port_alias;
  node[Options::PARAM_NAME_PORT_DEFAULT] = rhs.port_default;
  node[Options::PARAM_NAME_HIDDEN_PORTS] = rhs.hidden_ports;
  node[Options::PARAM_NAME_WAIT_TIMEOUT] = rhs.wait_timeout.count();
  node[Options::PARAM_NAME_REQUEST_TIMEOUT] = rhs.request_timeout.count();
  node[Options::PARAM_NAME_ALLOW_UNREACHABLE] = rhs.allow_unreachable;
  node[Options::PARAM_NAME_LOGGER_LEVEL] = rhs.logger_level;
  node[Options::PARAM_NAME_EXTRA] = rhs.extra;
  return node;
}

bool convert<auto_apms_behavior_tree::core::NodeRegistrationOptions>::decode(
  const Node & node, auto_apms_behavior_tree::core::NodeRegistrationOptions & rhs)
{
  using namespace auto_apms_behavior_tree;
  using namespace auto_apms_behavior_tree::core;
  using Options = NodeRegistrationOptions;

  if (!node.IsMap())
    throw auto_apms_util::exceptions::YAMLFormatError(
      "YAML::Node for auto_apms_behavior_tree::core::NodeRegistrationOptions must be map but is type " +
      std::to_string(node.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

  // Resolve parent field first (if present) — sets rhs to the inherited parent options as a base.
  // This avoids the need to track overrides separately: we simply apply the remaining fields on top afterward.
  if (const Node parent_val = node[Options::PARAM_NAME_PARENT]) {
    if (!parent_val.IsScalar()) {
      throw auto_apms_util::exceptions::YAMLFormatError(
        "Value for key '" + std::string(Options::PARAM_NAME_PARENT) + "' must be scalar but is type " +
        std::to_string(parent_val.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
    }
    const std::string parent_str = parent_val.as<std::string>();

    const std::string id_sep = _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP;
    const std::string node_sep = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_NAMESPACE_DEFAULT_SEP;

    // Find the first '.' that follows the '::' separator to split manifest identity from node name
    const std::size_t id_sep_pos = parent_str.find(id_sep);
    const std::size_t dot_pos =
      parent_str.find(node_sep, id_sep_pos != std::string::npos ? id_sep_pos + id_sep.size() : 0);

    if (dot_pos == std::string::npos) {
      throw exceptions::NodeManifestError(
        "Invalid parent '" + parent_str + "'. Expected format: '<package_name>" + id_sep + "<manifest_alias>" +
        node_sep + "<node_name>'.");
    }

    const std::string manifest_identity_str = parent_str.substr(0, dot_pos);
    const std::string parent_node_name = parent_str.substr(dot_pos + node_sep.size());

    if (parent_node_name.empty()) {
      throw exceptions::NodeManifestError("Invalid parent '" + parent_str + "'. Parent node name must not be empty.");
    }

    try {
      const NodeManifest parent_manifest =
        NodeManifest::fromResource(NodeManifestResourceIdentity(manifest_identity_str));
      if (!parent_manifest.contains(parent_node_name)) {
        throw exceptions::NodeManifestError(
          "Parent node '" + parent_node_name + "' does not exist in manifest '" + manifest_identity_str + "'.");
      }
      rhs = parent_manifest[parent_node_name];
    } catch (const exceptions::NodeManifestError &) {
      throw;
    } catch (const std::exception & e) {
      throw exceptions::NodeManifestError(
        "Failed to load parent manifest '" + manifest_identity_str + "': " + e.what());
    }
  }

  // Apply all non-parent fields from the YAML node onto rhs.
  // For the parent case these act as overrides; for the no-parent case they are the full definition.
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    const Node val = it->second;

    if (key == Options::PARAM_NAME_PARENT) continue;  // already handled above

    if (key == Options::PARAM_NAME_EXTRA) {
      rhs.extra = val;
      continue;
    }

    if (key == Options::PARAM_NAME_PORT_ALIAS) {
      if (!val.IsMap()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      for (const auto & [k, v] : val.as<std::map<std::string, std::string>>()) rhs.port_alias[k] = v;
      continue;
    }

    if (key == Options::PARAM_NAME_PORT_DEFAULT) {
      if (!val.IsMap()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      for (const auto & [k, v] : val.as<std::map<std::string, std::string>>()) rhs.port_default[k] = v;
      continue;
    }

    if (key == Options::PARAM_NAME_HIDDEN_PORTS) {
      if (!val.IsSequence()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      for (const auto & entry : val.as<std::vector<std::string>>()) {
        if (std::find(rhs.hidden_ports.begin(), rhs.hidden_ports.end(), entry) == rhs.hidden_ports.end())
          rhs.hidden_ports.push_back(entry);
      }
      continue;
    }

    if (key == Options::PARAM_NAME_CLASS) {
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.class_name = val.as<std::string>();
      continue;
    }
    if (key == Options::PARAM_NAME_DESCRIPTION) {
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.description = val.as<std::string>();
      continue;
    }
    if (key == Options::PARAM_NAME_ROS2TOPIC) {
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.topic = val.as<std::string>();
      continue;
    }
    if (key == Options::PARAM_NAME_WAIT_TIMEOUT) {
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.wait_timeout = std::chrono::duration<double>(val.as<double>());
      continue;
    }
    if (key == Options::PARAM_NAME_REQUEST_TIMEOUT) {
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.request_timeout = std::chrono::duration<double>(val.as<double>());
      continue;
    }
    if (key == Options::PARAM_NAME_ALLOW_UNREACHABLE) {
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.allow_unreachable = val.as<bool>();
      continue;
    }
    if (key == Options::PARAM_NAME_LOGGER_LEVEL) {
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.logger_level = val.as<std::string>();
      continue;
    }

    // Unknown parameter
    throw auto_apms_util::exceptions::YAMLFormatError("Unknown parameter name '" + key + "'.");
  }
  return true;
}

}  // namespace YAML
/// @endcond