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

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"

#include <fstream>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_behavior_tree::core
{

NodeManifestResourceIdentity::NodeManifestResourceIdentity(const std::string & identity)
{
  if (identity.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Cannot create node manifest resource identity from empty string. Must be <package_name>" +
      std::string(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP) + "<metadata_id>.");
  }
  if (std::size_t pos = identity.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP);
      pos == std::string::npos) {
    // If only a single token is given, assume it's metadata_id
    package_name = "";
    metadata_id = metadata_id;
  } else {
    package_name = identity.substr(0, pos);
    metadata_id = identity.substr(pos + std::string(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP).size());
  }
  if (metadata_id.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Node manifest resource identity string '" + identity + "' is invalid. Metadata ID must not be empty.");
  }
}

NodeManifestResourceIdentity::NodeManifestResourceIdentity(const char * identity)
: NodeManifestResourceIdentity(std::string(identity))
{
}

bool NodeManifestResourceIdentity::operator==(const NodeManifestResourceIdentity & other) const
{
  return str() == other.str();
}

bool NodeManifestResourceIdentity::operator<(const NodeManifestResourceIdentity & other) const
{
  return str() < other.str();
}

std::string NodeManifestResourceIdentity::str() const
{
  return package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + metadata_id;
}

bool NodeManifestResourceIdentity::empty() const { return package_name.empty() && metadata_id.empty(); }

std::set<NodeManifestResourceIdentity> getNodeManifestResourceIdentities(const std::set<std::string> & exclude_packages)
{
  std::set<NodeManifestResourceIdentity> identities;
  for (const auto & p : auto_apms_util::getPackagesWithResourceType(
         _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST, exclude_packages)) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(
          _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST, p, content, &base_path)) {
      for (const auto & line :
           auto_apms_util::splitString(content, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP)) {
        const std::vector<std::string> parts = auto_apms_util::splitString(
          line, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP, false);
        if (parts.size() > 0) {
          NodeManifestResourceIdentity i;
          i.package_name = p;
          i.metadata_id = parts[0];
          identities.insert(i);
        }
      }
    }
  }
  return identities;
}

NodeModelMap getNativeNodeModel() { return TreeDocument().getNodeModel(true); }

NodeManifest::NodeManifest(const Map & map) : map_{map} {}

NodeManifest NodeManifest::fromFiles(const std::vector<std::string> & paths)
{
  NodeManifest manifest;
  for (const auto & path : paths) {
    try {
      manifest.merge(fromFile(path));
    } catch (const std::exception & e) {
      throw exceptions::NodeManifestError("Error creating node manifest from multiple files: " + std::string(e.what()));
    }
  }
  return manifest;
}

NodeManifest NodeManifest::fromResource(const NodeManifestResourceIdentity & search_identity)
{
  return NodeManifestResource(search_identity).getNodeManifest();
}

void NodeManifest::toFile(const std::string & file_path) const
{
  std::ofstream out_stream(file_path);
  if (out_stream.is_open()) {
    out_stream << this->encode();
    out_stream.close();
  } else {
    throw exceptions::NodeManifestError("Error opening node manifest output file '" + file_path + "'.");
  }
}

bool NodeManifest::contains(const std::string & node_name) const { return map_.find(node_name) != map_.end(); }

NodeManifest::RegistrationOptions & NodeManifest::operator[](const std::string & node_name)
{
  if (contains(node_name)) return map_[node_name];
  throw std::out_of_range{
    "Node '" + node_name + "' doesn't exist in node manifest (Size: " + std::to_string(map_.size()) +
    "). Use the add() method to add an entry."};
}

const NodeManifest::RegistrationOptions & NodeManifest::operator[](const std::string & node_name) const
{
  if (contains(node_name)) return map_.at(node_name);
  throw std::out_of_range{
    "Node '" + node_name + "' doesn't exist in node manifest (Size: " + std::to_string(map_.size()) + ")."};
}

NodeManifest & NodeManifest::add(const std::string & node_name, const RegistrationOptions & opt)
{
  if (contains(node_name)) {
    throw exceptions::NodeManifestError{
      "Node '" + node_name + "' already exists in node manifest (Size: " + std::to_string(map_.size()) + ")."};
  }
  if (!opt.valid()) {
    throw exceptions::NodeManifestError(
      "Cannot add node '" + node_name + "' to manifest. Parameter class_name must not be empty.");
  }
  map_[node_name] = opt;
  return *this;
}

NodeManifest & NodeManifest::remove(const std::string & node_name)
{
  if (!contains(node_name)) {
    throw std::out_of_range{
      "Node '" + node_name + "' doesn't exist in node manifest, so the corresponding entry cannot be removed."};
  }
  map_.erase(node_name);
  return *this;
}

NodeManifest & NodeManifest::merge(const NodeManifest & other, bool replace)
{
  for (const auto & [node_name, params] : other.map()) {
    if (contains(node_name)) {
      if (replace) {
        map_.erase(node_name);
      } else {
        throw exceptions::NodeManifestError(
          "Cannot merge node manifests, because node '" + node_name +
          "' already exists and argument replace is false (Won't replace existing entries).");
      }
    }
    add(node_name, params);
  }
  return *this;
}

NodeManifest & NodeManifest::mergeWithNamespace(
  const NodeManifest & other, const std::string & with_namespace, const std::string & sep)
{
  for (const auto & [node_name, params] : other.map()) {
    // Apply namespace prefix
    const std::string final_name = with_namespace + sep + node_name;
    if (contains(final_name)) {
      throw exceptions::NodeManifestError(
        "Cannot merge node manifests, because node '" + final_name + "' already exists.");
    }
    add(final_name, params);
  }
  return *this;
}

NodeManifest & NodeManifest::applyNodeNamespace(const std::string & ns, const std::string & sep)
{
  const NodeManifest::Map old_map = map_;
  map_.clear();
  for (const auto & [node_name, params] : old_map) {
    const std::string final_name = ns + sep + node_name;
    map_[final_name] = params;
  }
  return *this;
}

std::vector<std::string> NodeManifest::getNodeNames()
{
  std::vector<std::string> names;
  names.reserve(map_.size());
  for (const auto & [name, _] : map_) names.push_back(name);
  return names;
}

size_t NodeManifest::size() const { return map_.size(); }

bool NodeManifest::empty() const { return map_.empty(); }

const NodeManifest::Map & NodeManifest::map() const { return map_; }

NodeManifestResource::NodeManifestResource(const Identity & search_identity)
{
  std::set<std::string> search_packages;
  if (!search_identity.package_name.empty()) {
    search_packages.insert(search_identity.package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST);
  }

  size_t matching_count = 0;
  for (const auto & p : search_packages) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(
          _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST, p, content, &base_path)) {
      std::vector<std::string> lines =
        auto_apms_util::splitString(content, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP);
      for (const std::string & line : lines) {
        std::vector<std::string> parts = auto_apms_util::splitString(
          line, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP, false);
        if (parts.size() != 3) {
          throw auto_apms_util::exceptions::ResourceError(
            "Invalid node manifest resource file (Package: '" + p + "'). Invalid line: " + line + ".");
        }

        // Determine if resource is matching
        std::string found_metadata_id = parts[0];
        if (found_metadata_id != search_identity.metadata_id) continue;

        // Found matching resource - Increase counter
        matching_count++;

        // Now fill the other member variables in case the resource matches (if match is not unique, error is thrown
        // later and the object is discarded)

        unique_identity_.package_name = p;
        unique_identity_.metadata_id = found_metadata_id;
        node_manifest_file_path_ = base_path + "/" + parts[1];
        node_model_file_path_ = base_path + "/" + parts[2];
        node_manifest_ = NodeManifest::fromFile(node_manifest_file_path_);

        tinyxml2::XMLDocument model_doc;
        if (model_doc.LoadFile(node_model_file_path_.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
          throw exceptions::TreeDocumentError(
            "Error parsing the node model associated with node manifest " + unique_identity_.str());
        }
        node_model_ = TreeDocument::getNodeModel(model_doc, node_manifest_);
      }
    }
  }

  if (matching_count == 0) {
    throw auto_apms_util::exceptions::ResourceError(
      "No node manifest resource was found using identity '" + search_identity.str() + "'.");
  }
  if (matching_count > 1) {
    throw auto_apms_util::exceptions::ResourceError(
      "There are multiple node manifest resources with metadata ID '" + search_identity.str() + "'.");
  }
}

NodeManifestResource::NodeManifestResource(const std::string & search_identity)
: NodeManifestResource(NodeManifestResourceIdentity(search_identity))
{
}

NodeManifestResource::NodeManifestResource(const char * search_identity)
: NodeManifestResource(std::string(search_identity))
{
}

const NodeManifestResource::Identity & NodeManifestResource::getIdentity() const { return unique_identity_; }

const NodeManifest & NodeManifestResource::getNodeManifest() const { return node_manifest_; }

const NodeModelMap & NodeManifestResource::getNodeModel() const { return node_model_; }

}  // namespace auto_apms_behavior_tree::core
