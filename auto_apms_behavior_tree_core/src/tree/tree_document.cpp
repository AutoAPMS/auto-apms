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

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

#include <algorithm>
#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/logging.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/xml_parsing.h"

namespace auto_apms_behavior_tree::core
{

std::vector<std::string> getAllTreeNamesImpl(const tinyxml2::XMLDocument & doc)
{
  std::vector<std::string> names;
  if (const tinyxml2::XMLElement * root = doc.RootElement()) {
    if (strcmp(root->Name(), TreeDocument::ROOT_ELEMENT_NAME) == 0) {
      // Found root element
      for (const tinyxml2::XMLElement * child = root->FirstChildElement(); child != nullptr;
           child = child->NextSiblingElement()) {
        if (strcmp(TreeDocument::TREE_ELEMENT_NAME, child->Name()) == 0) {
          if (const char * name = child->Attribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME)) {
            names.push_back(name);
          } else {
            throw exceptions::TreeDocumentError(
              "Cannot get tree name, because required attribute '" +
              std::string(TreeDocument::TREE_NAME_ATTRIBUTE_NAME) + "' is missing.");
          }
        }
      }
    } else if (strcmp(root->Name(), TreeDocument::TREE_ELEMENT_NAME) == 0) {
      // Found behavior tree element as root
      if (const char * name = root->Attribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME)) names.push_back(name);
    }
  }
  return names;
}

TreeDocument::NodeElement::NodeElement(TreeDocument * doc_ptr, XMLElement * ele_ptr)
: doc_ptr_(doc_ptr), ele_ptr_(ele_ptr)
{
  if (!doc_ptr) {
    throw exceptions::TreeDocumentError("Cannot create an instance of NodeElement with doc_ptr=nullptr.");
  }
  if (!ele_ptr) {
    throw exceptions::TreeDocumentError("Cannot create an instance of NodeElement with ele_ptr=nullptr.");
  }
  const NodeModelMap model = doc_ptr_->getNodeModel(true);
  NodeModelMap::const_iterator it = model.find(ele_ptr_->Name());
  std::vector<NodePortInfo> port_infos_str_vec;
  if (it != model.end()) port_infos_str_vec = it->second.port_infos;

  // There should always be a corresponding element in the nodes model. However, if the constructor is called e.g.
  // before the document registered the respective node, that's not the case and setPorts() won't work as expected.
  // Therefore, we must ensure that all factory methods for a NodeElement verify that the node is known to the document
  // before creating an instance. NOTE: We must not throw, since the constructor must also succeed for the derived
  // TreeElement class for which there is no node model and setPorts() is deleted anyways.
  if (!port_infos_str_vec.empty()) {
    for (const NodePortInfo & port_info : port_infos_str_vec) {
      port_names_.push_back(port_info.port_name);
      if (!port_info.port_default.empty()) port_default_values_[port_info.port_name] = port_info.port_default;
    }

    // Set the node's attributes according to the default values for all the ports that have a default value and have
    // not been set previously.
    PortValues existing_port_values = getPorts();
    for (const auto & [name, val] : port_default_values_) {
      if (existing_port_values.find(name) == existing_port_values.end()) {
        ele_ptr_->SetAttribute(name.c_str(), val.c_str());
      }
    }
  }
}

TreeDocument::NodeElement & TreeDocument::NodeElement::operator=(const NodeElement & other)
{
  XMLElement * other_ele = other.ele_ptr_;
  if (ele_ptr_ != other_ele) {
    // Copy the other element and all its children
    other_ele = other_ele->DeepClone(doc_ptr_)->ToElement();
    // Insert new element in place of this
    XMLElement * prev = ele_ptr_->PreviousSiblingElement();
    if (prev) {
      ele_ptr_->Parent()->InsertAfterChild(ele_ptr_->PreviousSibling(), other_ele);
    } else {
      ele_ptr_->Parent()->InsertFirstChild(other_ele);
    }
    // Delete this element
    doc_ptr_->DeleteNode(ele_ptr_);
    // Update the internal pointer
    ele_ptr_ = other_ele;
  }
  port_names_ = other.port_names_;
  port_default_values_ = other.port_default_values_;
  return *this;
}

bool TreeDocument::NodeElement::operator==(const NodeElement & other) const { return ele_ptr_ == other.ele_ptr_; }

bool TreeDocument::NodeElement::operator!=(const NodeElement & other) const { return !this->operator==(other); }

TreeDocument::NodeElement TreeDocument::NodeElement::insertNode(
  const std::string & name, const NodeElement * before_this)
{
  if (const std::set<std::string> names = doc_ptr_->getRegisteredNodeNames(true); names.find(name) == names.end()) {
    throw exceptions::TreeDocumentError(
      "Cannot insert unkown node <" + name +
      ">. Before inserting a new node, the associated document must register the corresponding behavior tree "
      "node. Consider using a signature of insertNode() that does this automatically.");
  }
  XMLElement * ele = doc_ptr_->NewElement(name.c_str());
  return insertBeforeImpl(before_this, ele);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertNode(
  const std::string & name, const NodeRegistrationOptions & registration_options, const NodeElement * before_this)
{
  // Check if the node name is known. If the user tries to register a node with the same name as one of the native
  // nodes, we want registerNodes to throw since those names are reserved (Therefore we set include_native to false).
  if (const std::set<std::string> names = doc_ptr_->getRegisteredNodeNames(false); names.find(name) == names.end()) {
    doc_ptr_->registerNodes(NodeManifest({{name, registration_options}}));
  }
  return insertNode(name, before_this);
}

model::SubTree TreeDocument::NodeElement::insertSubTreeNode(
  const std::string & tree_name, const NodeElement * before_this)
{
  if (!doc_ptr_->hasTreeName(tree_name)) {
    throw exceptions::TreeDocumentError(
      "Cannot insert subtree node for tree '" + tree_name + "' because no tree with that name exists.");
  }
  NodeElement ele = insertNode(SUBTREE_ELEMENT_NAME, before_this);
  ele.ele_ptr_->SetAttribute(TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str());
  return model::SubTree(ele.doc_ptr_, ele.ele_ptr_);
}

model::SubTree TreeDocument::NodeElement::insertSubTreeNode(const TreeElement & tree, const NodeElement * before_this)
{
  const std::string tree_name = tree.getName();
  if (doc_ptr_->hasTreeName(tree_name)) {
    // A tree with this name already exists in the document.
    // If the provided tree element is from THIS document, we can compare directly using pointer equality.
    // If it's from an external document, we compare the XML content to detect conflicting trees.
    const TreeElement existing_tree = doc_ptr_->getTree(tree_name);
    if (tree.doc_ptr_ == doc_ptr_) {
      // Tree element is from this document - they must be the same tree element
      if (existing_tree != tree) {
        throw exceptions::TreeDocumentError(
          "Cannot insert subtree node using the provided tree element, because another tree with name '" + tree_name +
          "' already exists.");
      }
    } else {
      // Tree element is from an external document - compare XML content
      const std::string existing_xml = existing_tree.writeToString();
      const std::string incoming_xml = tree.writeToString();
      if (existing_xml != incoming_xml) {
        throw exceptions::TreeDocumentError(
          "Cannot insert subtree node using the provided tree element, because another tree with name '" + tree_name +
          "' already exists with different content.");
      }
    }
    // If we reach here, the trees are equivalent - proceed to insert the subtree node
  } else {
    // Add the tree provided as argument to the document
    doc_ptr_->mergeTree(tree, false);
  }
  return insertSubTreeNode(tree_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTree(
  const TreeElement & tree, const NodeElement * before_this)
{
  const XMLElement * root_child = tree.ele_ptr_->FirstChildElement();
  if (!root_child) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree element '" + tree.getFullyQualifiedName() + "' because it has no child nodes.");
  }
  if (root_child->NextSibling()) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree element '" + tree.getFullyQualifiedName() + "' because it has more than one child node.");
  }
  doc_ptr_->registerNodes(tree.getRequiredNodeManifest());
  return insertTreeFromDocument(*tree.doc_ptr_, tree.getName(), before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromDocument(
  const TreeDocument & doc, const std::string & tree_name, const NodeElement * before_this)
{
  const std::vector<std::string> other_tree_names = doc.getAllTreeNames();
  if (other_tree_names.empty()) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree '" + tree_name + "' because document has no <" + TREE_ELEMENT_NAME + "> elements.");
  }
  if (!auto_apms_util::contains(other_tree_names, tree_name)) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree '" + tree_name + "' because document doesn't specify a tree with that name.");
  }

  // List including the target tree name and the names of its dependencies (Trees required by SubTree nodes)
  std::set<std::string> required_tree_names = {tree_name};

  // Find all subtree nodes and push back the associated tree to required_tree_names.
  // NOTE: All tree elements within a TreeDocument instance always have exactly one child, so we don't have to verify
  // that here again
  TreeDocument temp_doc(doc_ptr_->format_version_, doc_ptr_->tree_node_loader_ptr_);  // Temporary working document
  doc.DeepCopy(&temp_doc);
  const TreeElement other_tree(&temp_doc, temp_doc.getXMLElementForTreeWithName(tree_name));
  ConstDeepApplyCallback collect_dependency_tree_names;
  collect_dependency_tree_names = [&required_tree_names, &collect_dependency_tree_names](const NodeElement & ele) {
    if (ele.getRegistrationName() == SUBTREE_ELEMENT_NAME) {
      if (const char * name = ele.ele_ptr_->Attribute(TREE_NAME_ATTRIBUTE_NAME)) {
        // Add the tree name to the list
        required_tree_names.insert(name);

        // Search for more tree dependencies under the tree pointed by this subtree node
        ele.doc_ptr_->getTree(name).deepApplyConst(collect_dependency_tree_names);
      } else {
        throw exceptions::TreeDocumentError("Subtree element has no name attribute.");
      };
    }
    // The return value doesn't matter here
    return false;
  };
  other_tree.deepApplyConst(collect_dependency_tree_names);

  // Verify that all child nodes of the trees listed in required_tree_names are known to the builder
  ConstDeepApplyCallback apply = [available_names = doc_ptr_->getRegisteredNodeNames(true)](const NodeElement & ele) {
    return available_names.find(ele.getRegistrationName()) == available_names.end();
  };
  for (const std::string & name : required_tree_names) {
    const NodeElement ele(&temp_doc, temp_doc.getXMLElementForTreeWithName(name)->FirstChildElement());
    if (const std::vector<NodeElement> found = ele.deepApplyConst(apply); !found.empty()) {
      std::vector<std::string> names;
      for (const NodeElement & ele : found) names.push_back(ele.getFullyQualifiedName());
      throw exceptions::TreeDocumentError(
        "Cannot insert tree '" + tree_name + "' because the following nodes found in tree '" + name +
        "' are unkown to the builder:\n\t- " + auto_apms_util::join(names, "\n\t- "));
    }
  }

  // Remove the data from each tree that the target tree doesn't directly depend on and merge the working document. This
  // approach ensures, that the verification logic inside merge() is applied to the working document.
  required_tree_names.erase(tree_name);  // Don't add the target tree twice
  for (const std::string & name : other_tree_names) {
    if (required_tree_names.find(name) == required_tree_names.end()) {
      // Remove if not required
      temp_doc.removeTree(name);
    }
  }
  doc_ptr_->mergeTreeDocument(static_cast<const XMLDocument &>(temp_doc), false);

  // Clone and insert the target tree
  return insertBeforeImpl(
    before_this, doc.getXMLElementForTreeWithName(tree_name)->FirstChildElement()->DeepClone(doc_ptr_)->ToElement());
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromDocument(
  const TreeDocument & doc, const NodeElement * before_this)
{
  return insertTreeFromDocument(doc, doc.getRootTreeName(), before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromString(
  const std::string & tree_str, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc(doc_ptr_->format_version_, doc_ptr_->tree_node_loader_ptr_);
  insert_doc.mergeString(tree_str);
  return insertTreeFromDocument(insert_doc, tree_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromString(
  const std::string & tree_str, const NodeElement * before_this)
{
  TreeDocument insert_doc(doc_ptr_->format_version_, doc_ptr_->tree_node_loader_ptr_);
  insert_doc.mergeString(tree_str);
  return insertTreeFromDocument(insert_doc, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromFile(
  const std::string & path, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc(doc_ptr_->format_version_, doc_ptr_->tree_node_loader_ptr_);
  insert_doc.mergeFile(path);
  return insertTreeFromDocument(insert_doc, tree_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromFile(
  const std::string & path, const NodeElement * before_this)
{
  TreeDocument insert_doc(doc_ptr_->format_version_, doc_ptr_->tree_node_loader_ptr_);
  insert_doc.mergeFile(path);
  return insertTreeFromDocument(insert_doc, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromResource(
  const TreeResource & resource, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc(doc_ptr_->format_version_, doc_ptr_->tree_node_loader_ptr_);
  insert_doc.mergeFile(resource.build_request_file_path_);

  // We register all associated node plugins beforehand, so that the user doesn't have to do that manually. This means,
  // that also potentially unused nodes are available and registered with the factory. This seems unnecessary, but it's
  // very convenient and the performance probably doesn't suffer too much.
  doc_ptr_->registerNodes(resource.getNodeManifest(), false);

  return insertTreeFromDocument(insert_doc, tree_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromResource(
  const TreeResource & resource, const NodeElement * before_this)
{
  return insertTreeFromResource(resource, resource.getRootTreeName(), before_this);
}

bool TreeDocument::NodeElement::hasChildren() const { return ele_ptr_->FirstChild() == nullptr ? false : true; }

TreeDocument::NodeElement TreeDocument::NodeElement::getFirstNode(
  const std::string & registration_name, const std::string & instance_name, bool deep_search) const
{
  if (registration_name.empty() && instance_name.empty()) return NodeElement(doc_ptr_, ele_ptr_->FirstChildElement());

  ConstDeepApplyCallback apply = [&registration_name, &instance_name](const NodeElement & ele) {
    if (registration_name.empty()) return ele.getName() == instance_name;
    if (instance_name.empty()) return ele.getRegistrationName() == registration_name;
    return ele.getRegistrationName() == registration_name && ele.getName() == instance_name;
  };

  if (deep_search) {
    if (const std::vector<NodeElement> found = deepApplyConst(apply); !found.empty()) return found[0];
  } else {
    for (auto child : *this) {
      if (apply(child)) return child;
    }
  }

  // Cannot find node in children of this
  throw exceptions::TreeDocumentError(
    "Cannot find a child node that matches the search arguments (registration_name: '" + registration_name +
    "' - instance_name: '" + instance_name + "') in parent element '" + getFullyQualifiedName() + "'.");
}

TreeDocument::NodeElement & TreeDocument::NodeElement::removeFirstChild(
  const std::string & registration_name, const std::string & instance_name, bool deep_search)
{
  XMLElement * found = getFirstNode(registration_name, instance_name, deep_search).ele_ptr_;
  found->Parent()->DeleteChild(found);
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::removeChildren()
{
  ele_ptr_->DeleteChildren();
  return *this;
}

const std::vector<std::string> & TreeDocument::NodeElement::getPortNames() const { return port_names_; }

TreeDocument::NodeElement::PortValues TreeDocument::NodeElement::getPorts() const
{
  PortValues values;
  for (const tinyxml2::XMLAttribute * attr = ele_ptr_->FirstAttribute(); attr != nullptr; attr = attr->Next()) {
    if (const std::string attr_name = attr->Name(); auto_apms_util::contains(port_names_, attr_name)) {
      values[attr_name] = attr->Value();
    }
  }
  return values;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::setPorts(const PortValues & port_values)
{
  // Verify port_values
  std::vector<std::string> unkown_keys;
  for (const auto & [key, _] : port_values) {
    if (!auto_apms_util::contains(port_names_, key)) unkown_keys.push_back(key);
  }
  if (!unkown_keys.empty()) {
    throw exceptions::TreeDocumentError(
      "Cannot set ports. According to the node model, the following ports are not implemented by '" +
      std::string(ele_ptr_->Name()) + "': [ " + auto_apms_util::join(unkown_keys, ", ") + " ].");
  }

  // Populate attributes according to the content of port_values
  for (const auto & [key, val] : port_values) {
    ele_ptr_->SetAttribute(key.c_str(), val.c_str());
  }
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::resetPorts()
{
  for (const std::string & name : port_names_) {
    PortValues::const_iterator it = port_default_values_.find(name);
    if (it != port_default_values_.end()) {
      ele_ptr_->SetAttribute(it->first.c_str(), it->second.c_str());
    } else {
      ele_ptr_->DeleteAttribute(name.c_str());
    }
  }
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::setConditionalScript(BT::PreCond type, const Script & script)
{
  ele_ptr_->SetAttribute(BT::toStr(type).c_str(), script.str().c_str());
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::setConditionalScript(BT::PostCond type, const Script & script)
{
  ele_ptr_->SetAttribute(BT::toStr(type).c_str(), script.str().c_str());
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::setName(const std::string & instance_name)
{
  ele_ptr_->SetAttribute(NODE_INSTANCE_NAME_ATTRIBUTE_NAME, instance_name.c_str());
  return *this;
}

std::string TreeDocument::NodeElement::getRegistrationName() const { return ele_ptr_->Name(); }

std::string TreeDocument::NodeElement::getName() const
{
  if (const char * name = ele_ptr_->Attribute(NODE_INSTANCE_NAME_ATTRIBUTE_NAME)) return name;
  return getRegistrationName();
}

std::string TreeDocument::NodeElement::getFullyQualifiedName() const
{
  std::string registration_name = getRegistrationName();
  std::string instance_name = getName();
  if (registration_name == instance_name) return registration_name;
  return instance_name + " (" + registration_name + ")";
}

const TreeDocument & TreeDocument::NodeElement::getParentDocument() const { return *doc_ptr_; }

const std::vector<TreeDocument::NodeElement> TreeDocument::NodeElement::deepApplyConst(
  ConstDeepApplyCallback apply_callback) const
{
  std::vector<NodeElement> found;
  deepApplyImpl(*this, apply_callback, found);
  return found;
}

std::vector<TreeDocument::NodeElement> TreeDocument::NodeElement::deepApply(DeepApplyCallback apply_callback)
{
  std::vector<NodeElement> found;
  deepApplyImpl(*this, apply_callback, found);
  return found;
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertBeforeImpl(
  const NodeElement * before_this, XMLElement * add_this)
{
  if (before_this) {
    XMLElement * prev = nullptr;
    XMLElement * curr = ele_ptr_->FirstChildElement();

    // Traverse through siblings of first child to find the before_this node.
    // If there are no siblings, add the first child to this;
    bool found = false;
    while (curr) {
      if (curr == before_this->ele_ptr_) {
        found = true;
        break;
      }
      prev = curr;
      curr = curr->NextSiblingElement();
    }
    if (prev) {
      if (found) {
        ele_ptr_->InsertAfterChild(prev, add_this);
      } else {
        throw exceptions::TreeDocumentError(
          "NodeElement before_this (" + before_this->getFullyQualifiedName() + ") is not a child of " +
          getFullyQualifiedName() + ".");
      }
    } else {
      ele_ptr_->InsertFirstChild(add_this);
    }
  } else {
    ele_ptr_->InsertEndChild(add_this);
  }
  return NodeElement(doc_ptr_, add_this);
}

void TreeDocument::NodeElement::deepApplyImpl(
  const NodeElement & parent, ConstDeepApplyCallback apply_callback, std::vector<NodeElement> & vec)
{
  for (XMLElement * child = parent.ele_ptr_->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement()) {
    const NodeElement child_ele(parent.doc_ptr_, child);

    // Apply on current child element
    if (apply_callback(child_ele)) vec.push_back(child_ele);

    // Search children of child before evaluating the siblings
    deepApplyImpl(child_ele, apply_callback, vec);
  }
}

void TreeDocument::NodeElement::deepApplyImpl(
  NodeElement & parent, DeepApplyCallback apply_callback, std::vector<NodeElement> & vec)
{
  for (XMLElement * child = parent.ele_ptr_->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement()) {
    NodeElement child_ele(parent.doc_ptr_, child);

    // Apply on current child element
    if (apply_callback(child_ele)) vec.push_back(child_ele);

    // Search children of child before evaluating the siblings
    deepApplyImpl(child_ele, apply_callback, vec);
  }
}

TreeDocument::NodeElement::ChildIterator::ChildIterator() : doc_ptr_(nullptr), current_(nullptr) {}

TreeDocument::NodeElement::ChildIterator::ChildIterator(TreeDocument * doc_ptr, tinyxml2::XMLElement * current)
: doc_ptr_(doc_ptr), current_(current)
{
}

TreeDocument::NodeElement::ChildIterator::value_type TreeDocument::NodeElement::ChildIterator::operator*() const
{
  return NodeElement(doc_ptr_, current_);
}

TreeDocument::NodeElement::ChildIterator & TreeDocument::NodeElement::ChildIterator::operator++()
{
  if (current_) current_ = current_->NextSiblingElement();
  return *this;
}

TreeDocument::NodeElement::ChildIterator TreeDocument::NodeElement::ChildIterator::operator++(int)
{
  ChildIterator tmp = *this;
  ++(*this);
  return tmp;
}

bool TreeDocument::NodeElement::ChildIterator::operator==(const ChildIterator & other) const
{
  return current_ == other.current_;
}

bool TreeDocument::NodeElement::ChildIterator::operator!=(const ChildIterator & other) const
{
  return current_ != other.current_;
}

TreeDocument::NodeElement::ChildIterator TreeDocument::NodeElement::begin() const
{
  return ChildIterator(doc_ptr_, ele_ptr_->FirstChildElement());
}

TreeDocument::NodeElement::ChildIterator TreeDocument::NodeElement::end() const { return ChildIterator(); }

TreeDocument::TreeElement::TreeElement(TreeDocument * doc_ptr, XMLElement * ele_ptr) : NodeElement(doc_ptr, ele_ptr)
{
  if (!ele_ptr->Attribute(TREE_NAME_ATTRIBUTE_NAME)) {
    throw exceptions::TreeDocumentError(
      "Cannot create tree element without a '" + std::string(TREE_NAME_ATTRIBUTE_NAME) + "' attribute.");
  }
}

TreeDocument::TreeElement & TreeDocument::TreeElement::operator=(const TreeElement & other)
{
  const std::string other_tree_name = other.getName();
  // For a tree replacement to be allowed, the other tree name must be the same or at least not already taken by another
  // tree inside this document.
  if (getName() != other_tree_name && doc_ptr_->hasTreeName(other.getName())) {
    throw exceptions::TreeDocumentError(
      "Cannot copy tree '" + other.getName() + "' because another tree with this name already exists.");
  }
  NodeElement::operator=(other);
  return *this;
}

TreeDocument::TreeElement & TreeDocument::TreeElement::setName(const std::string & tree_name)
{
  ele_ptr_->SetAttribute(TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str());
  return *this;
}

std::string TreeDocument::TreeElement::getName() const
{
  if (const char * name = ele_ptr_->Attribute(TREE_NAME_ATTRIBUTE_NAME)) return name;
  return "unkown";
}

TreeDocument::TreeElement & TreeDocument::TreeElement::makeRoot()
{
  doc_ptr_->setRootTreeName(getName());
  return *this;
}

NodeManifest TreeDocument::TreeElement::getRequiredNodeManifest() const
{
  NodeManifest m;
  deepApplyConst([this, &m](const NodeElement & node) {
    const std::string name = node.getRegistrationName();
    const bool is_native_node = doc_ptr_->native_node_names_.find(name) != doc_ptr_->native_node_names_.end();
    if (!is_native_node && !m.contains(name)) {
      if (!doc_ptr_->registered_nodes_manifest_.contains(name)) {
        throw exceptions::NodeManifestError(
          "Cannot assemble the required node manifest for tree '" + getName() +
          "' since there are no registration options for node '" + name + "'.");
      }
      m.add(name, doc_ptr_->registered_nodes_manifest_[name]);
    }
    return false;
  });
  return m;
}

BT::Result TreeDocument::TreeElement::verify() const
{
  TreeDocument doc(doc_ptr_->format_version_, doc_ptr_->tree_node_loader_ptr_);
  return doc.mergeTree(*this, true).verify();
}

std::string TreeDocument::TreeElement::writeToString() const
{
  XMLDocument tree_doc;
  tree_doc.InsertEndChild(ele_ptr_->DeepClone(&tree_doc));
  tinyxml2::XMLPrinter printer;
  tree_doc.Print(&printer);
  return printer.CStr();
}

TreeDocument::TreeElement & TreeDocument::TreeElement::removeFirstChild(
  const std::string & registration_name, const std::string & instance_name, bool deep_search)
{
  NodeElement::removeFirstChild(registration_name, instance_name, deep_search);
  return *this;
}

TreeDocument::TreeElement & TreeDocument::TreeElement::removeChildren()
{
  NodeElement::removeChildren();
  return *this;
}

TreeDocument::TreeDocument(const std::string & format_version, NodeRegistrationLoader::SharedPtr tree_node_loader)
// It's important to initialize XMLDocument using PRESERVE_WHITESPACE, since encoded port data (like
// NodeRegistrationOptions) may be sensitive to changes in the whitespaces (think of the YAML format).
: XMLDocument(true, tinyxml2::PRESERVE_WHITESPACE),
  all_node_classes_package_map_(auto_apms_behavior_tree::core::NodeRegistrationLoader().getClassPackageMap()),
  native_node_names_(BT::BehaviorTreeFactory().builtinNodes()),
  format_version_(format_version),
  tree_node_loader_ptr_(tree_node_loader),
  registered_nodes_manifest_(),
  factory_(),
  logger_(rclcpp::get_logger(LOGGER_NAME))
{
  reset();
}

TreeDocument & TreeDocument::mergeTreeDocument(const XMLDocument & other, bool adopt_root_tree)
{
  std::set<std::string> include_stack;
  return mergeTreeDocumentImpl(other, adopt_root_tree, include_stack);
}

TreeDocument & TreeDocument::mergeTreeDocumentImpl(
  const XMLDocument & other, bool adopt_root_tree, std::set<std::string> & include_stack)
{
  const XMLElement * other_root = other.RootElement();
  if (!other_root) {
    throw exceptions::TreeDocumentError("Cannot merge tree documents: other_root is nullptr.");
  };

  auto verify_tree_structure = [](const XMLElement * tree_ele) {
    const char * tree_id = tree_ele->Attribute(TREE_NAME_ATTRIBUTE_NAME);
    if (!tree_id) {
      throw exceptions::TreeDocumentError(
        "Cannot merge tree document: Found a <" + std::string(TREE_ELEMENT_NAME) +
        "> element that doesn't specify the required attribute '" + TREE_NAME_ATTRIBUTE_NAME + "'.");
    }
    const XMLElement * tree_root_child = tree_ele->FirstChildElement();
    if (!tree_root_child) {
      throw exceptions::TreeDocumentError(
        "Cannot merge tree document: Tree '" + std::string(tree_id) + "' has no child nodes.");
    }
    if (tree_root_child->NextSibling()) {
      throw exceptions::TreeDocumentError(
        "Cannot merge tree document: Tree '" + std::string(tree_id) + "' has more than one child node.");
    }
  };

  // Helper lambda to check for duplicate tree names and throw with a descriptive message
  auto check_duplicates = [this](const std::vector<std::string> & new_tree_names, const std::string & source) {
    const auto common = auto_apms_util::getCommonElements(getAllTreeNames(), new_tree_names);
    if (!common.empty()) {
      throw exceptions::TreeDocumentError(
        "Cannot merge tree document: The following trees from " + source + " are already defined: [ " +
        auto_apms_util::join(common, ", ") + " ].");
    }
  };

  // Get tree names from the other document (used for duplicate check and adopt_root_tree logic)
  const std::vector<std::string> other_tree_names = getAllTreeNamesImpl(other);

  if (strcmp(other_root->Name(), ROOT_ELEMENT_NAME) == 0) {
    // Verify format
    if (const char * ver = other_root->Attribute(BTCPP_FORMAT_ATTRIBUTE_NAME)) {
      if (std::string(ver) != format_version_) {
        throw exceptions::TreeDocumentError(
          "Cannot merge tree document: Format of other document (" + std::string(BTCPP_FORMAT_ATTRIBUTE_NAME) + ": " +
          ver + ") is not compatible with this document (" + std::string(BTCPP_FORMAT_ATTRIBUTE_NAME) + ": " +
          format_version_ + ").");
      }
    } else {
      throw exceptions::TreeDocumentError(
        "Cannot merge tree document: Root element of other document doesn't have required attribute '" +
        std::string(BTCPP_FORMAT_ATTRIBUTE_NAME) + "'.");
    }

    // Iterate over all include elements and store the associated content to a temporary buffer document
    TreeDocument include_doc(format_version_, tree_node_loader_ptr_);
    for (const XMLElement * include_ele = other_root->FirstChildElement(INCLUDE_ELEMENT_NAME); include_ele != nullptr;
         include_ele = include_ele->NextSiblingElement(INCLUDE_ELEMENT_NAME)) {
      if (const char * path = include_ele->Attribute(INCLUDE_PATH_ATTRIBUTE_NAME)) {
        if (std::string(path).empty()) {
          throw exceptions::TreeDocumentError(
            "Cannot merge tree document: Found an <" + std::string(INCLUDE_ELEMENT_NAME) +
            "> element that specifies an empty path in attribute '" + INCLUDE_PATH_ATTRIBUTE_NAME + "'.");
        }
        std::string absolute_path = path;
        if (const char * ros_pkg = include_ele->Attribute(INCLUDE_ROS_PKG_ATTRIBUTE_NAME)) {
          if (std::string(ros_pkg).empty()) {
            throw exceptions::TreeDocumentError(
              "Cannot merge tree document: Found an <" + std::string(INCLUDE_ELEMENT_NAME) +
              "> element that specifies an empty ROS package name in attribute '" + INCLUDE_ROS_PKG_ATTRIBUTE_NAME +
              "'.");
          }
          if (std::filesystem::path(path).is_absolute()) {
            throw exceptions::TreeDocumentError(
              "Cannot merge tree document: Found an <" + std::string(INCLUDE_ELEMENT_NAME) +
              "> element that specifies an absolute path '" + std::string(path) + "' in attribute '" +
              INCLUDE_PATH_ATTRIBUTE_NAME + "' together with a ROS package name in attribute '" +
              INCLUDE_ROS_PKG_ATTRIBUTE_NAME + "'. Please remove the attribute " + INCLUDE_ROS_PKG_ATTRIBUTE_NAME +
              " or use a relative path if you want to refer to a location "
              "relative to the package's share directory.");
          }
          try {
            absolute_path =
              (std::filesystem::path(ament_index_cpp::get_package_share_directory(ros_pkg)) / path).string();
          } catch (const ament_index_cpp::PackageNotFoundError & e) {
            throw exceptions::TreeDocumentError(
              "Cannot merge tree document: Found an <" + std::string(INCLUDE_ELEMENT_NAME) +
              "> element that specifies a non-existing ROS package '" + std::string(ros_pkg) + "' in attribute '" +
              INCLUDE_ROS_PKG_ATTRIBUTE_NAME + "'.");
          }
        }

        // Recursively merge the included file to the buffer document, passing the include stack for circular detection
        try {
          include_doc.mergeFileImpl(absolute_path, false, include_stack);
        } catch (const std::exception & e) {
          throw exceptions::TreeDocumentError(
            "Cannot merge tree document: Failed to include file '" + absolute_path + "': " + e.what());
        }
      } else {
        throw exceptions::TreeDocumentError(
          "Cannot merge tree document: Found an <" + std::string(INCLUDE_ELEMENT_NAME) +
          "> element that doesn't specify the required attribute '" + INCLUDE_PATH_ATTRIBUTE_NAME + "'.");
      }
    }

    // Check for duplicates from included files before inserting
    check_duplicates(include_doc.getAllTreeNames(), "included files");

    // Iterate over all the tree elements of the include buffer and include them in this document
    for (const XMLElement * child = include_doc.RootElement()->FirstChildElement(TREE_ELEMENT_NAME); child != nullptr;
         child = child->NextSiblingElement(TREE_ELEMENT_NAME)) {
      // We do not need to verify the tree structure here, since it has already been verified during the recursive
      // mergeFile() calls by the for loop below

      // If tree element is valid, append to this document
      RootElement()->InsertEndChild(child->DeepClone(this));
    }

    // Check for duplicates from other document's direct trees before inserting
    check_duplicates(other_tree_names, "the merged document");

    // Iterate over all the tree elements of other root and verify them
    for (const XMLElement * child = other_root->FirstChildElement(TREE_ELEMENT_NAME); child != nullptr;
         child = child->NextSiblingElement(TREE_ELEMENT_NAME)) {
      verify_tree_structure(child);

      // If tree element is valid, append to this document
      RootElement()->InsertEndChild(child->DeepClone(this));
    }

    if (adopt_root_tree) {
      // After (!) all tree elements have been inserted, adopt the name specified in the corresponding attribute of the
      // root element
      if (const char * name = other_root->Attribute(ROOT_TREE_ATTRIBUTE_NAME)) setRootTreeName(name);
    }
  } else if (strcmp(other_root->Name(), TREE_ELEMENT_NAME) == 0) {
    // Allow a single behavior tree without <root> element.
    // We assume that the the format complies with the version configured at construction time
    verify_tree_structure(other_root);

    // Check for duplicates before inserting
    check_duplicates(other_tree_names, "the merged document");

    // If tree element is valid, append to this document
    RootElement()->InsertEndChild(other_root->DeepClone(this));
  } else {
    throw exceptions::TreeDocumentError(
      "Cannot merge tree document: Root element of other document must either be <" + std::string(ROOT_ELEMENT_NAME) +
      "> or <" + TREE_ELEMENT_NAME + ">.");
  }

  if (adopt_root_tree && other_tree_names.size() == 1) {
    // If there is only one tree element, we change the root tree to the respective tree after (!) all tree elements
    // have been inserted
    setRootTreeName(other_tree_names[0]);
  }

  return *this;
}

TreeDocument & TreeDocument::mergeTreeDocument(const TreeDocument & other, bool adopt_root_tree)
{
  registerNodes(other.getRequiredNodeManifest(), false);
  std::set<std::string> include_stack;
  return mergeTreeDocumentImpl(static_cast<const XMLDocument &>(other), adopt_root_tree, include_stack);
}

TreeDocument & TreeDocument::mergeString(const std::string & tree_str, bool adopt_root_tree)
{
  XMLDocument other_doc;
  if (other_doc.Parse(tree_str.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeDocumentError("Cannot merge tree document from string: " + std::string(other_doc.ErrorStr()));
  }
  std::set<std::string> include_stack;
  return mergeTreeDocumentImpl(other_doc, adopt_root_tree, include_stack);
}

TreeDocument & TreeDocument::mergeFile(const std::string & path, bool adopt_root_tree)
{
  std::set<std::string> include_stack;
  return mergeFileImpl(path, adopt_root_tree, include_stack);
}

TreeDocument & TreeDocument::mergeFileImpl(
  const std::string & path, bool adopt_root_tree, std::set<std::string> & include_stack)
{
  // Get canonical path for consistent circular include detection
  std::string canonical_path;
  try {
    canonical_path = std::filesystem::canonical(path).string();
  } catch (const std::filesystem::filesystem_error &) {
    // If canonical fails (e.g., file doesn't exist), use the original path
    // The error will be caught later when loading the file
    canonical_path = path;
  }

  // Check for circular includes
  if (include_stack.count(canonical_path) > 0) {
    throw exceptions::TreeDocumentError(
      "Cannot merge tree document: Circular include detected for file '" + path + "'.");
  }

  // Add to include stack
  include_stack.insert(canonical_path);

  XMLDocument other_doc;
  if (other_doc.LoadFile(path.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeDocumentError("Cannot create tree document from file " + path + ": " + other_doc.ErrorStr());
  }

  mergeTreeDocumentImpl(other_doc, adopt_root_tree, include_stack);

  // Remove from include stack (for recursive calls that continue after this one)
  include_stack.erase(canonical_path);
  return *this;
}

TreeDocument & TreeDocument::mergeResource(const TreeResource & resource, bool adopt_root_tree)
{
  registerNodes(resource.getNodeManifest(), false);
  return mergeFile(resource.build_request_file_path_, adopt_root_tree);
}

TreeDocument & TreeDocument::mergeTree(const TreeElement & tree, bool make_root_tree)
{
  XMLDocument tree_doc;
  tree_doc.InsertEndChild(tree.ele_ptr_->DeepClone(&tree_doc));
  registerNodes(tree.getRequiredNodeManifest(), false);
  mergeTreeDocument(tree_doc, make_root_tree);
  return *this;
}

TreeDocument::TreeElement TreeDocument::newTree(const std::string & tree_name)
{
  if (tree_name.empty()) {
    throw exceptions::TreeDocumentError("Cannot create a new tree with an empty name");
  }
  if (hasTreeName(tree_name)) {
    throw exceptions::TreeDocumentError(
      "Cannot create a new tree with name '" + tree_name + "' because it already exists.");
  }
  TreeDocument::XMLElement * new_ele = RootElement()->InsertNewChildElement(TreeDocument::TREE_ELEMENT_NAME);
  new_ele->SetAttribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str());
  return TreeElement(this, new_ele);
}

TreeDocument::TreeElement TreeDocument::newTree(const TreeElement & other_tree)
{
  return mergeTree(other_tree).getTree(other_tree.getName());
}

TreeDocument::TreeElement TreeDocument::newTreeFromDocument(const TreeDocument & other, const std::string & tree_name)
{
  std::string name(tree_name);
  if (name.empty()) {
    if (other.hasRootTreeName()) {
      name = other.getRootTreeName();
    } else if (const std::vector<std::string> names = other.getAllTreeNames(); names.size() == 1) {
      name = names[0];
    } else {
      throw exceptions::TreeDocumentError(
        "Failed to create new tree element from another document because argument tree_name was omitted and it was not "
        "possible to determine the root tree automatically.");
    }
  }
  TreeElement tree_ele = newTree(name);
  tree_ele.insertTreeFromDocument(other, name);
  return tree_ele;
}

TreeDocument::TreeElement TreeDocument::newTreeFromString(const std::string & tree_str, const std::string & tree_name)
{
  TreeDocument new_doc(format_version_, tree_node_loader_ptr_);
  new_doc.mergeString(tree_str, true);
  return newTreeFromDocument(new_doc, tree_name);
}

TreeDocument::TreeElement TreeDocument::newTreeFromFile(const std::string & path, const std::string & tree_name)
{
  TreeDocument new_doc(format_version_, tree_node_loader_ptr_);
  new_doc.mergeFile(path, true);
  return newTreeFromDocument(new_doc, tree_name);
}

TreeDocument::TreeElement TreeDocument::newTreeFromResource(
  const TreeResource & resource, const std::string & tree_name)
{
  TreeDocument new_doc(format_version_, tree_node_loader_ptr_);
  new_doc.mergeFile(resource.build_request_file_path_);
  if (resource.hasRootTreeName()) new_doc.setRootTreeName(resource.getRootTreeName());
  registerNodes(resource.getNodeManifest(), false);
  return newTreeFromDocument(new_doc, tree_name);
}

bool TreeDocument::hasTreeName(const std::string & tree_name) const
{
  if (auto_apms_util::contains(getAllTreeNames(), tree_name)) return true;
  return false;
}

TreeDocument::TreeElement TreeDocument::getTree(const std::string & tree_name)
{
  return TreeElement(this, getXMLElementForTreeWithName(tree_name));
}

TreeDocument & TreeDocument::setRootTreeName(const std::string & tree_name)
{
  if (tree_name.empty()) {
    throw exceptions::TreeDocumentError("Cannot set root tree name with empty string.");
  }
  if (!hasTreeName(tree_name)) {
    throw exceptions::TreeDocumentError(
      "Cannot make tree with name '" + tree_name + "' the root tree because it doesn't exist.");
  }
  RootElement()->SetAttribute(ROOT_TREE_ATTRIBUTE_NAME, tree_name.c_str());
  return *this;
}

bool TreeDocument::hasRootTreeName() const
{
  if (RootElement()->Attribute(ROOT_TREE_ATTRIBUTE_NAME)) return true;
  return false;
}

std::string TreeDocument::getRootTreeName() const
{
  if (const auto tree_name = RootElement()->Attribute(ROOT_TREE_ATTRIBUTE_NAME)) return tree_name;
  throw exceptions::TreeDocumentError(
    "Cannot get root tree name because the document's root element has no attribute '" +
    std::string(ROOT_TREE_ATTRIBUTE_NAME) + "'.");
}

TreeDocument::TreeElement TreeDocument::getRootTree() { return getTree(getRootTreeName()); }

TreeDocument & TreeDocument::removeTree(const std::string & tree_name)
{
  RootElement()->DeleteChild(getXMLElementForTreeWithName(tree_name));
  return *this;
}

TreeDocument & TreeDocument::removeTree(const TreeElement & tree) { return removeTree(tree.getName()); }

std::vector<std::string> TreeDocument::getAllTreeNames() const { return getAllTreeNamesImpl(*this); }

TreeDocument & TreeDocument::applyNodeNamespace(const std::string & node_namespace, const std::string & sep)
{
  // Get the old node names before applying namespace
  std::set<std::string> old_node_names;
  for (const auto & [name, _] : registered_nodes_manifest_.map()) {
    old_node_names.insert(name);
  }

  // Apply namespace to the registered nodes manifest
  registered_nodes_manifest_.applyNodeNamespace(node_namespace, sep);

  // Helper to recursively rename nodes in the XML
  auto rename_recursive = [&old_node_names, &node_namespace, &sep](XMLElement * parent, auto & self) -> void {
    for (XMLElement * child = parent->FirstChildElement(); child != nullptr; child = child->NextSiblingElement()) {
      const char * name = child->Name();
      // Rename if the node name is in the old registered nodes - we don't want to rename native nodes.
      if (name && old_node_names.count(name) > 0) {
        child->SetName((node_namespace + sep + name).c_str());
      }
      self(child, self);
    }
  };

  // Process all trees
  for (XMLElement * tree_ele = RootElement()->FirstChildElement(TREE_ELEMENT_NAME); tree_ele != nullptr;
       tree_ele = tree_ele->NextSiblingElement(TREE_ELEMENT_NAME)) {
    rename_recursive(tree_ele, rename_recursive);
  }

  // Also update the BehaviorTreeFactory registrations so that the factory
  // recognizes the new (namespaced) node IDs used in the XML.
  // We must copy the manifest and builder data BEFORE unregistering, as unregisterBuilder
  // invalidates any references to the internal maps.
  std::vector<std::pair<std::string, std::pair<BT::TreeNodeManifest, BT::NodeBuilder>>> registrations_to_update;
  const auto & factory_manifests = factory_.manifests();
  const auto & factory_builders = factory_.builders();
  for (const auto & old_name : old_node_names) {
    const auto manifest_it = factory_manifests.find(old_name);
    const auto builder_it = factory_builders.find(old_name);
    if (manifest_it == factory_manifests.end() || builder_it == factory_builders.end()) {
      // This should never happen since we derived old_node_names from registered_nodes_manifest_
      throw exceptions::TreeDocumentError(
        "Internal error while applying node namespace: Node '" + old_name +
        "' not found in BehaviorTreeFactory registrations.");
    }
    // Copy the data before we modify the factory
    registrations_to_update.emplace_back(old_name, std::make_pair(manifest_it->second, builder_it->second));
  }

  // Now perform the unregister/register operations using the copied data
  for (const auto & [old_name, manifest_builder_pair] : registrations_to_update) {
    factory_.unregisterBuilder(old_name);
    BT::TreeNodeManifest new_manifest = manifest_builder_pair.first;
    new_manifest.registration_ID = node_namespace + sep + old_name;
    factory_.registerBuilder(new_manifest, manifest_builder_pair.second);
  }

  return *this;
}

TreeDocument & TreeDocument::registerNodes(const NodeManifest & tree_node_manifest, bool override)
{
  // Make sure that there are no node names that are reserved for native nodes
  std::set<std::string> all_registration_names;
  for (const auto & [name, _] : tree_node_manifest.map()) all_registration_names.insert(name);
  if (const std::set<std::string> common =
        auto_apms_util::getCommonElements(all_registration_names, native_node_names_);
      !common.empty()) {
    throw exceptions::TreeDocumentError(
      "Found reserved node registration names in the node manifest. The following names are not allowed, because "
      "they refer to native behavior tree nodes: [ " +
      auto_apms_util::join(std::vector<std::string>(common.begin(), common.end()), ", ") + " ].");
  }

  for (const auto & [node_name, params] : tree_node_manifest.map()) {
    // By design, we require the user to maintain unique registration names for nodes. Theoretically, we could tolerate
    // multiple registrations of the same node name if both registration options are identical, but this would
    // complicate the implementation (comparing every option one by one) and indicates bad structuring of node manifests
    // anyways. So we discourage this practice by throwing an error.
    if (registered_nodes_manifest_.contains(node_name)) {
      if (override) {
        // If override is true, register the new node plugin instead of the current one
        factory_.unregisterBuilder(node_name);
        registered_nodes_manifest_.remove(node_name);
      } else {
        // If overriding is not explicitly wanted, we must throw
        throw exceptions::TreeDocumentError(
          "Tried to register node '" + node_name + "' (Class: " + params.class_name +
          ") which is already known. You must make sure that the registration names are unique or explicitly allow "
          "overriding previously registered nodes with the same name by setting override=true.");
      }
    }

    // Check if the class we search for is actually available with the loader.
    if (!tree_node_loader_ptr_->isClassAvailable(params.class_name)) {
      if (all_node_classes_package_map_.find(params.class_name) == all_node_classes_package_map_.end()) {
        throw exceptions::TreeDocumentError(
          "Node '" + node_name + " (" + params.class_name +
          ")' cannot be registered, because the class name is not known to the class loader. "
          "Make sure that it's spelled correctly and registered by calling "
          "auto_apms_behavior_tree_register_nodes() in the CMakeLists.txt of the "
          "corresponding package.");
      }
      throw exceptions::TreeDocumentError(
        "Node '" + node_name + " (" + params.class_name +
        ")' cannot be registered, because the corresponding resource belongs to excluded package '" +
        all_node_classes_package_map_.at(params.class_name) + "'.");
    }

    pluginlib::UniquePtr<NodeRegistrationInterface> plugin_instance;
    try {
      plugin_instance = tree_node_loader_ptr_->createUniqueInstance(params.class_name);
    } catch (const pluginlib::CreateClassException & e) {
      throw pluginlib::CreateClassException(
        "Failed to create an instance of node '" + node_name + " (" + params.class_name +
        ")'. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE "
        "macro must be called in the source file for the node class to be discoverable. "
        "Error message: " +
        e.what() + ".");
    }

    try {
      if (plugin_instance->requiresRosNodeContext()) {
        if (only_non_ros_nodes_) {
          throw exceptions::NodeRegistrationError(
            "Node '" + node_name +
            "' relies on ROS 2 functionality but this instance only allows to use non-ROS nodes.");
        }
        RosNodeContext ros_node_context(
          ros_node_wptr_.lock(), tree_node_waitables_callback_group_wptr_.lock(),
          tree_node_waitables_executor_wptr_.lock(), params);
        plugin_instance->registerWithBehaviorTreeFactory(factory_, node_name, &ros_node_context);
      } else {
        // Create a dummy object of RosNodeContext to allow for parsing the registration params nevertheless
        RosNodeContext ros_node_context(nullptr, nullptr, nullptr, params);
        plugin_instance->registerWithBehaviorTreeFactory(factory_, node_name, &ros_node_context);
      }
    } catch (const std::exception & e) {
      throw exceptions::NodeRegistrationError(
        "Cannot register node '" + node_name + " (" + params.class_name + ")': " + e.what() + ".");
    }
    registered_nodes_manifest_.add(node_name, params);
  }
  return *this;
}

std::set<std::string> TreeDocument::getRegisteredNodeNames(bool include_native) const
{
  std::set<std::string> names;
  if (include_native) names = native_node_names_;
  for (const auto & [name, _] : registered_nodes_manifest_.map()) names.insert(name);
  return names;
}

NodeManifest TreeDocument::getRequiredNodeManifest() const
{
  NodeManifest m;
  TreeDocument * doc = const_cast<TreeDocument *>(this);
  for (const std::string & tree_name : getAllTreeNames()) {
    XMLElement * ptr = const_cast<XMLElement *>(getXMLElementForTreeWithName(tree_name));
    const TreeElement ele(doc, ptr);
    m.merge(ele.getRequiredNodeManifest(), true);
  }
  return m;
}

TreeDocument & TreeDocument::addNodeModel(NodeModelMap model_map)
{
  // Create or get the TreeNodesModel element
  tinyxml2::XMLElement * model_root = RootElement()->FirstChildElement(TREE_NODE_MODEL_ELEMENT_NAME);

  // If no TreeNodesModel element exists, create one
  if (!model_root) {
    model_root = NewElement(TREE_NODE_MODEL_ELEMENT_NAME);
    RootElement()->InsertEndChild(model_root);
  }

  // Iterate through the model_map and create XML elements for each node
  for (const auto & [node_name, model] : model_map) {
    // Create the element with the node type as the tag name
    tinyxml2::XMLElement * node_element = NewElement(BT::toStr(model.type).c_str());
    node_element->SetAttribute("ID", node_name.c_str());

    // Add port information
    for (const auto & port_info : model.port_infos) {
      tinyxml2::XMLElement * port_element = nullptr;

      // Create the appropriate port element based on direction
      switch (port_info.port_direction) {
        case BT::PortDirection::INPUT:
          port_element = NewElement("input_port");
          break;
        case BT::PortDirection::OUTPUT:
          port_element = NewElement("output_port");
          break;
        case BT::PortDirection::INOUT:
          port_element = NewElement("inout_port");
          break;
      }

      // Set port attributes
      port_element->SetAttribute("name", port_info.port_name.c_str());

      if (!port_info.port_type.empty()) {
        port_element->SetAttribute("type", port_info.port_type.c_str());
      }

      if (port_info.port_has_default) {
        port_element->SetAttribute("default", port_info.port_default.c_str());
      }

      // Set port description as text content
      if (!port_info.port_description.empty()) {
        port_element->SetText(port_info.port_description.c_str());
      }

      node_element->InsertEndChild(port_element);
    }

    model_root->InsertEndChild(node_element);
  }

  return *this;
}

NodeModelMap TreeDocument::getNodeModel(tinyxml2::XMLDocument & doc, const NodeManifest & manifest)
{
  const tinyxml2::XMLElement * root = doc.RootElement();
  if (!root) {
    throw exceptions::TreeDocumentError("Node model document has no root element.");
  }
  if (const char * ver = root->Attribute(TreeDocument::BTCPP_FORMAT_ATTRIBUTE_NAME)) {
    const std::string expected_format = TreeDocument::BTCPP_FORMAT_DEFAULT_VERSION;
    if (std::string(ver) != expected_format) {
      throw exceptions::TreeDocumentError(
        "Cannot parse node model document: Format of model document (" +
        std::string(TreeDocument::BTCPP_FORMAT_ATTRIBUTE_NAME) + ": " + ver +
        ") doesn't comply with the expected format (" + std::string(TreeDocument::BTCPP_FORMAT_ATTRIBUTE_NAME) + ": " +
        expected_format + ").");
    }
  } else {
    throw exceptions::TreeDocumentError(
      "Cannot parse node model document: Root element of model document doesn't have required attribute '" +
      std::string(TreeDocument::BTCPP_FORMAT_ATTRIBUTE_NAME) + "'.");
  }
  tinyxml2::XMLElement * model_ele = doc.RootElement()->FirstChildElement(TreeDocument::TREE_NODE_MODEL_ELEMENT_NAME);
  if (!model_ele) {
    throw exceptions::TreeDocumentError(
      "Element <" + std::string(TreeDocument::TREE_NODE_MODEL_ELEMENT_NAME) +
      "> doesn't exist in node model document.");
  }

  NodeModelMap model_map;
  for (tinyxml2::XMLElement * ele = model_ele->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement()) {
    const char * node_name = ele->Attribute("ID");
    if (!node_name) {
      throw exceptions::TreeDocumentError(
        "Element '" + std::string(ele->Name()) + "' in node model document is missing the required attribute 'ID'");
    }
    NodeModel & model = model_map[node_name];
    model.type = BT::convertFromString<BT::NodeType>(ele->Name());
    for (const tinyxml2::XMLElement * port_ele = ele->FirstChildElement(); port_ele != nullptr;
         port_ele = port_ele->NextSiblingElement()) {
      const std::string direction = port_ele->Name();
      NodePortInfo port_info;
      if (direction == "input_port") {
        port_info.port_direction = BT::PortDirection::INPUT;
      } else if (direction == "output_port") {
        port_info.port_direction = BT::PortDirection::OUTPUT;
      } else if (direction == "inout_port") {
        port_info.port_direction = BT::PortDirection::INOUT;
      } else {
        throw exceptions::TreeDocumentError(
          "Unkown port direction in node model for '" + std::string(node_name) + "': " + direction);
      }
      if (const char * c = port_ele->Attribute("name")) {
        port_info.port_name = c;
      }
      if (const char * c = port_ele->Attribute("type")) {
        port_info.port_type = c;
      }
      if (const char * c = port_ele->Attribute("default")) {
        port_info.port_has_default = true;
        port_info.port_default = c;
      } else {
        port_info.port_has_default = false;
      }
      if (const char * c = port_ele->GetText()) {
        port_info.port_description = c;
      }
      model.port_infos.push_back(std::move(port_info));
    }
    // Port infos may be empty if there are no ports
  }

  // Collect hidden ports from the node manifest
  std::map<std::string, std::vector<std::string>> hidden_ports;
  for (const auto & [node_name, registration_options] : manifest.map()) {
    // Extract hidden_ports from registration options
    for (const std::string & port_name : registration_options.hidden_ports) {
      hidden_ports[node_name].push_back(port_name);
    }

    // Implicitly hide ports that are specified in port_alias
    for (const auto & [port_name, _] : registration_options.port_alias) {
      hidden_ports[node_name].push_back(port_name);
    }
  }

  // Apply port hiding
  for (const auto & [node_name, ports_to_hide] : hidden_ports) {
    auto it = model_map.find(node_name);
    if (it == model_map.end()) {
      continue;
    }
    NodeModel & model = it->second;
    model.port_infos.erase(
      std::remove_if(
        model.port_infos.begin(), model.port_infos.end(),
        [&ports_to_hide](const NodePortInfo & port_info) {
          return std::find(ports_to_hide.begin(), ports_to_hide.end(), port_info.port_name) != ports_to_hide.end();
        }),
      model.port_infos.end());
  }

  return model_map;
}

NodeModelMap TreeDocument::getNodeModel(bool include_native) const
{
  // Generate XML from the factory
  std::string model_xml;
  try {
    model_xml = BT::writeTreeNodesModelXML(factory_, include_native);
  } catch (const std::exception & e) {
    throw exceptions::TreeDocumentError("Error generating node model XML from factory: " + std::string(e.what()));
  }

  // Parse the XML
  tinyxml2::XMLDocument model_doc;
  if (model_doc.Parse(model_xml.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeDocumentError(
      "Error parsing the model of the currently registered nodes: " + std::string(model_doc.ErrorStr()));
  }

  // Get the node model with hidden ports applied
  return getNodeModel(model_doc, registered_nodes_manifest_);
}

BT::Result TreeDocument::verify() const
{
  NodeModelMap model_map = getNodeModel(true);
  std::unordered_map<std::string, BT::NodeType> registered_nodes;
  for (const auto & [node_name, model] : model_map) {
    registered_nodes[node_name] = model.type;
  }
  try {
    BT::VerifyXML(writeToString(), registered_nodes);
  } catch (const BT::RuntimeError & e) {
    return nonstd::make_unexpected(e.what());
  }
  return {};
}

std::string TreeDocument::writeToString() const
{
  tinyxml2::XMLPrinter printer;
  Print(&printer);
  return printer.CStr();
}

void TreeDocument::writeToFile(const std::string & path) const
{
  XMLDocument doc;
  DeepCopy(&doc);
  tinyxml2::XMLError result = doc.SaveFile(path.c_str());
  if (result != tinyxml2::XML_SUCCESS) {
    throw exceptions::TreeDocumentError(
      "Failed to write tree document to file. Error ID: " + std::string(doc.ErrorIDToName(result)));
  }
}

TreeDocument & TreeDocument::reset()
{
  Clear();
  tinyxml2::XMLElement * root_ele = NewElement(TreeDocument::ROOT_ELEMENT_NAME);
  root_ele->SetAttribute(TreeDocument::BTCPP_FORMAT_ATTRIBUTE_NAME, format_version_.c_str());
  InsertFirstChild(root_ele);
  return *this;
}

const TreeDocument::XMLElement * TreeDocument::getXMLElementForTreeWithName(const std::string & tree_name) const
{
  return getXMLElementForTreeWithNameImpl<const XMLElement *>(*this, tree_name);
}

TreeDocument::XMLElement * TreeDocument::getXMLElementForTreeWithName(const std::string & tree_name)
{
  return getXMLElementForTreeWithNameImpl<XMLElement *>(*this, tree_name);
}

template <typename ReturnT, typename DocumentT>
ReturnT TreeDocument::getXMLElementForTreeWithNameImpl(DocumentT & doc, const std::string & tree_name)
{
  if (tree_name.empty()) {
    throw exceptions::TreeDocumentError("Cannot get tree with an empty name.");
  }
  if (!doc.hasTreeName(tree_name)) {
    throw exceptions::TreeDocumentError("Cannot get tree with name '" + tree_name + "' because it doesn't exist.");
  }
  auto child = doc.RootElement()->FirstChildElement(TreeDocument::TREE_ELEMENT_NAME);
  while (child && !child->Attribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str())) {
    child = child->NextSiblingElement();
  }
  if (!child) {
    throw std::logic_error(
      "Unexpected error trying to get tree element with name '" + tree_name +
      "'. Since hasTreeName() returned true, there MUST be a corresponding element.");
  }
  return child;
}

}  // namespace auto_apms_behavior_tree::core