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

#pragma once

#include <memory>

#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Core API for AutoAPMS's behavior tree implementation.
 */
namespace auto_apms_behavior_tree::core
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Class for configuring and instantiating behavior trees.
 *
 * This class extends the functionality provided by `BT::BehaviorTreeFactory` of
 * [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and offers a user-friendly
 * [builder](https://refactoring.guru/design-patterns/builder/cpp/example#lang-features) API for configuring behavior
 * trees. In contrast to the original API, the user doesn't need to manually register behavior tree node entities with
 * the factory in order to create an instance of `BT::Tree`. Instead, this class conveniently automates this process for
 * you by querying the `ament_index`.
 *
 * ## Usage
 *
 * To create an executable behavior tree object, one must configure the tree using the inherited TreeDocument API.
 * Afterwards, the behavior tree may be instantiated using the factory method TreeBuilder::instantiate.
 *
 * The most convenient way of building and executing behavior trees is by using the TreeBuildHandler plugins that come
 * with AutoAPMS and are fully integrated with the TreeExecutorNode concept.
 *
 * @sa <a href="https://autoapms.github.io/auto-apms-guide/tutorial/deploying-behaviors>Tutorial:
 * Deploying Behaviors</a>
 */
class TreeBuilder : public TreeDocument
{
  inline static const std::string LOGGER_NAME = "tree_builder";

public:
  RCLCPP_SMART_PTR_DEFINITIONS(TreeBuilder)

  /**
   * @brief Constructor.
   *
   * Arguments @p ros_node, @p tree_node_waitables_callback_group and @p tree_node_waitables_executor are passed to
   * behavior tree node plugins which utilize ROS 2 communication interfaces or waitables in general, that is, all nodes
   * inheriting from RosPublisherNode, RosSubscriberNode, RosServiceNode or RosActionNode.
   *
   * @param ros_node Weak pointer to the ROS 2 node instance to be associate the behavior tree node with.
   * @param tree_node_waitables_callback_group The ROS 2 callback group to be used within tree nodes when adding
   * waitables.
   * @param tree_node_waitables_executor The ROS 2 executor instance that may be used for executing work provided by
   * the node's waitables.
   * @param tree_node_loader Shared pointer to the behavior tree node plugin loader instance.
   */
  explicit TreeBuilder(
    rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group,
    rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
    NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

  /**
   * @brief Constructor.
   *
   * Using this signature you'll only be able to instantiate behavior trees tha only contain nodes which don't require
   * an instance of RosNodeContext during construction time (non-ROS nodes).
   *
   * @param tree_node_loader Shared pointer to the behavior tree node plugin loader instance.
   */
  explicit TreeBuilder(NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

  virtual ~TreeBuilder() override = default;

  /// @copydoc TreeDocument::registerNodes(const NodeManifest & tree_node_manifest, bool override)
  TreeBuilder & registerNodes(const NodeManifest & tree_node_manifest, bool override = false) override;

  /**
   * @brief Define enums that may be used by any scripts inside the behavior tree.
   * @param enum_name Name that is used to refer to the given value.
   * @param val Value of the enum.
   * @return Modified tree builder.
   */
  TreeBuilder & setScriptingEnum(const std::string & enum_name, int val);

  /**
   * @brief Define enums that may be used by any scripts inside the behavior tree.
   *
   * This function automatically defines the scripting enums by inspecting @p EnumT using
   * https://github.com/Neargye/magic_enum.
   *
   * @warning Refer to [this page](https://github.com/Neargye/magic_enum/blob/master/doc/limitations.md) to learn about
   * the limitations when using this approach.
   * @tparam EnumT Type of the enumeration.
   * @return Modified tree builder.
   */
  template <typename EnumT>
  TreeBuilder & setScriptingEnumsFromType();

  /**
   * @brief Create the behavior tree.
   *
   * This creates an instance of `BT::Tree` which holds the memory of all node callbacks and enables the user to
   * actually execute the behavior tree. The entry point is determined by @p root_tree_name.
   * @param root_tree_name Name of an existing tree that should be the root tree.
   * @param bb_ptr Optional pointer to the parent blackboard.
   * @return Instance of `BT::Tree` representing the configured behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeBuildError if there's no tree named @p root_tree_name.
   * @throw auto_apms_behavior_tree::exceptions::TreeBuildError if the tree cannot be instantiated.
   */
  Tree instantiate(const std::string & root_tree_name, TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

  /**
   * @brief Create the behavior tree.
   *
   * This creates an instance of `BT::Tree` which holds the memory of all node callbacks and enables the user to
   * actually execute the behavior tree. The entry point will be the underlying document's root tree.
   * @param bb_ptr Optional pointer to the parent blackboard.
   * @return Instance of `BT::Tree` representing the configured behavior tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeBuildError if it's not defined which one of the existing trees is
   * the root tree.
   * @throw auto_apms_behavior_tree::exceptions::TreeBuildError if the tree cannot be instantiated.
   */
  Tree instantiate(TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename EnumT>
TreeBuilder & TreeBuilder::setScriptingEnumsFromType()
{
  factory_.registerScriptingEnums<EnumT>();
  return *this;
}

}  // namespace auto_apms_behavior_tree::core