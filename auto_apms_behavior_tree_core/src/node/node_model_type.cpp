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

#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace auto_apms_behavior_tree
{
namespace core
{

TreeDocument::NodeElement NodeModelType::toNodeElement() { return NodeElement(*this); }

}  // namespace core

namespace model
{

BT::NodeType SubTree::type() { return BT::NodeType::SUBTREE; }

std::string SubTree::name() { return core::TreeDocument::SUBTREE_ELEMENT_NAME; }

std::string SubTree::getRegistrationName() const { return name(); }

SubTree & SubTree::setBlackboardRemapping(const std::map<std::string, std::string> & remapping)
{
  for (const auto & [key, val] : remapping) {
    if (!BT::TreeNode::isBlackboardPointer(val)) {
      throw exceptions::TreeDocumentError(
        "When setting the blackboard remapping for a subtree, you must refer to the parent blackboard's entry to remap "
        "to using curly brackets (Got: '" +
        val + "').");
    }
    ele_ptr_->SetAttribute(key.c_str(), val.c_str());
  }
  return *this;
}

std::map<std::string, std::string> SubTree::getBlackboardRemapping() const
{
  std::map<std::string, std::string> remapping;
  for (const tinyxml2::XMLAttribute * attr = ele_ptr_->FirstAttribute(); attr != nullptr; attr = attr->Next()) {
    if (BT::TreeNode::isBlackboardPointer(attr->Value())) {
      remapping[attr->Name()] = attr->Value();
    }
  }
  return remapping;
}

SubTree & SubTree::set_auto_remap(bool val) { return setPorts({{"_autoremap", BT::toStr(val)}}); }

bool SubTree::get_auto_remap() const { return BT::convertFromString<bool>(getPorts().at("_autoremap")); }

}  // namespace model
}  // namespace auto_apms_behavior_tree
