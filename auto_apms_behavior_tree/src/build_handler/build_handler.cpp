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

#include "auto_apms_behavior_tree/build_handler/build_handler.hpp"

#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

TreeBasedEntryPoint::TreeBasedEntryPoint(const std::string & encoded_str)
{
  if (encoded_str.empty()) {
    throw exceptions::TreeBuildHandlerError("Entry point string must not be empty.");
  }

  // Find the opening parenthesis
  const auto open_paren = encoded_str.find('(');
  if (open_paren == std::string::npos) {
    // No parentheses: the entire string is the tree name
    root_tree_name = encoded_str;
    return;
  }

  // Validate closing parenthesis
  if (encoded_str.back() != ')') {
    throw exceptions::TreeBuildHandlerError(
      "Invalid entry point format: missing closing parenthesis in '" + encoded_str + "'.");
  }

  root_tree_name = encoded_str.substr(0, open_paren);
  if (root_tree_name.empty()) {
    throw exceptions::TreeBuildHandlerError(
      "Invalid entry point format: tree name must not be empty in '" + encoded_str + "'.");
  }

  // Extract the content between the parentheses
  const std::string params_str = encoded_str.substr(open_paren + 1, encoded_str.size() - open_paren - 2);
  if (params_str.empty()) {
    return;
  }

  // Parse comma-separated key=value pairs
  std::string::size_type pos = 0;
  while (pos < params_str.size()) {
    const auto comma = params_str.find(',', pos);
    const std::string token = params_str.substr(pos, comma - pos);

    const auto eq = token.find('=');
    if (eq == std::string::npos) {
      throw exceptions::TreeBuildHandlerError(
        "Invalid entry point format: expected 'key=value' but got '" + token + "' in '" + encoded_str + "'.");
    }

    const std::string key = token.substr(0, eq);
    const std::string value = token.substr(eq + 1);
    if (key.empty()) {
      throw exceptions::TreeBuildHandlerError(
        "Invalid entry point format: blackboard key must not be empty in '" + encoded_str + "'.");
    }
    inital_blackboard[key] = value;

    if (comma == std::string::npos) {
      break;
    }
    pos = comma + 1;
  }
}

TreeBuildHandler::TreeBuildHandler(
  const std::string & name, rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
: logger_(ros_node_ptr->get_logger().get_child(name)),
  ros_node_wptr_(ros_node_ptr),
  tree_node_loader_ptr(tree_node_loader_ptr)
{
}

TreeBuildHandler::TreeBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
: TreeBuildHandler("tree_build_handler", ros_node_ptr, tree_node_loader_ptr)
{
}

bool TreeBuildHandler::setBuildRequest(
  const std::string & /*build_request*/, const std::string & /*entry_point*/, const NodeManifest & /*node_manifest*/)
{
  return true;
}

rclcpp::Node::SharedPtr TreeBuildHandler::getRosNodePtr() const
{
  if (ros_node_wptr_.expired()) {
    throw std::runtime_error("TreeBuildHandler: Weak pointer to rclcpp::Node expired.");
  }
  return ros_node_wptr_.lock();
}

TreeBuildHandler::NodeLoader::SharedPtr TreeBuildHandler::getNodeLoaderPtr() const { return tree_node_loader_ptr; }

}  // namespace auto_apms_behavior_tree