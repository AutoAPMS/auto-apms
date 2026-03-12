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

#include <map>

#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

// Include all built in node models and helpers for convenience
#include "auto_apms_behavior_tree/behavior_tree_nodes.hpp"
#include "auto_apms_behavior_tree/util/node.hpp"

// Include exceptions if derived build handlers need to throw a TreeBuildHandlerError
#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Decoded entry point information for low-level tree building.
 *
 * The expected format of the encoded string is:
 *
 * **MyTreeName(var1=value1,var2=value2,...)**
 *
 * It follows the syntax used by a lot of programming languages for instantiating classes or calling funtions. The part
 * before the parantheses is the **tree name** and the part within defines its **inital blackboard**. The initial
 * blackboard values are given as comma-separated key-value pairs, where the key and value are separated by an equals
 * sign. The value is expected to be a string that can be parsed into the respective type of the blackboard entry.
 */
struct TreeBasedEntryPoint
{
  std::string root_tree_name;
  std::map<std::string, std::string> inital_blackboard;

  /**
   * @brief Default constructor.
   */
  TreeBasedEntryPoint() = default;

  /**
   * @brief Constructor that decodes the given string into the expected format.
   * @param encoded_str String that encodes the entry point.
   */
  TreeBasedEntryPoint(const std::string & encoded_str);
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Base class for plugins that implement patterns for creating behavior trees using a standardized interface.
 *
 * Inheriting classes must implement TreeBuildHandler::buildTree. Additionally, the user is given the possibility to
 * define specific rules for when to accept a request and what to do when one arrives. This can be achieved by
 * overriding TreeBuildHandler::setBuildRequest. By default, all requests are accepted and the arguments of the build
 * request are ignored.
 *
 * A tree build handler allows TreeExecutorNode to create a behavior tree at runtime when an execution
 * request is received. The user may change the the way how a behavior tree is created by simply switching to the
 * desired build handler implementation. To make those implementations available at runtime, the user must register them
 * using the CMake macro `auto_apms_behavior_tree_register_build_handlers` in the CMakeLists.txt of the parent package.
 *
 * ## Usage
 *
 * Behavior tree build handler plugins are created like this:
 *
 * ```cpp
 * // src/my_build_handler.cpp
 *
 * #include "auto_apms_behavior_tree/build_handler.hpp"
 *
 * namespace my_namespace
 * {
 *
 * class MyCustomBuildHandler : public auto_apms_behavior_tree::TreeBuildHandler
 * {
 * public:
 *   using TreeBuildHandler::TreeBuildHandler;
 *
 *   bool setBuildRequest(const std::string & build_request,
 *                        const std::string & entry_point,
 *                        const NodeManifest & node_manifest) override final
 *   {
 *     // Do something when a build request arrives. If this isn't overridden,
 *     // all requests are accepted regardless of the arguments given above.
 *
 *     // ...
 *
 *     // Returning true means accepting the request, false means rejecting it
 *     return true;
 *   }
 *
 *   TreeDocument::TreeElement buildTree(TreeDocument & doc,
 *                                       TreeBlackboard & bb) override final
 *   {
 *     // Configure the behavior tree
 *     TreeDocument::TreeElement tree = doc.newTree("MyTreeName");
 *
 *     // ...
 *
 *     // The returned tree element must point to a behavior tree within doc
 *     // and will be used as the root tree.
 *     return tree;
 *   }
 * };
 *
 * }  // namespace my_namespace
 *
 * // Make sure the plugin class is discoverable
 * AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(my_namespace::MyCustomBuildHandler)
 * ```
 *
 * Given this implementation, the CMake macro `auto_apms_behavior_tree_register_build_handlers` must be called in the
 * CMakeLists.txt of the parent package like this:
 *
 * ```cmake
 * find_package(ament_cmake REQUIRED)
 * find_package(auto_apms_behavior_tree REQUIRED)
 *
 * # Create a shared library
 * add_library(my_build_handler_library_target SHARED
 *   "src/my_build_handler.cpp"  # Add the source file
 * )
 * target_link_libraries(my_build_handler_library_target PUBLIC
 *   auto_apms_behavior_tree::auto_apms_behavior_tree  # Link against the auto_apms_behavior_tree library
 * )
 *
 * # Add the plugin to this package's ament_index resources
 * auto_apms_behavior_tree_register_build_handlers(my_build_handler_library_target
 *   "my_namespace::MyCustomBuildHandler"
 * )
 *
 * # Install the shared library to the standard directory
 * install(
 *   TARGETS
 *   my_build_handler_library_target
 *   LIBRARY DESTINATION lib
 *   ARCHIVE DESTINATION lib
 *   RUNTIME DESTINATION bin
 * )
 *
 * ament_package()
 * ```
 *
 * Once the parent package has been installed, the build handler plugin may be loaded using TreeBuildHandlerLoader. With
 * a behavior tree executor node, the user may simply set its parameter called `build_handler` to
 * "my_namespace::MyCustomBuildHandler" (the fully qualified class name). All build requests that the node receives from
 * this point on are forwarded to this specific build handler implementation.
 *
 * @sa <a
 *href="https://autoapms.github.io/auto-apms-guide/tutorial/building-behavior-trees#using-treebuildhandler">
 * Tutorial: Using Behavior Tree Build Handlers</a>
 */
class TreeBuildHandler
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(TreeBuildHandler)

  /* Convenience aliases for deriving classes */

  using NodeLoader = core::NodeRegistrationLoader;
  using NodeManifest = core::NodeManifest;
  using TreeResource = core::TreeResource;
  using TreeDocument = core::TreeDocument;
  using TreeBlackboard = auto_apms_behavior_tree::TreeBlackboard;

  /**
   * @brief Constructor allowing to give the build handler a specific name.
   * @param name Name of the build handler.
   * @param ros_node_ptr Shared pointer to the associated ROS 2 node.
   * @param tree_node_loader_ptr  Shared pointer to an existing behavior tree node loader that the build handler may
   * work with.
   */
  TreeBuildHandler(
    const std::string & name, rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr);

  /**
   * @brief Constructor.
   *
   * This signature is used when creating an instance of this build handler.
   *
   * The build handler is given a generic name. To provide a user defined name, reimplement the constructor and
   * delegate construction to the overload above. This may look something like this:
   *
   * ```cpp
   * MyCustomBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
   * : TreeBuildHandler("my_custom_name", ros_node_ptr, tree_node_loader_ptr)
   * {
   * }
   * ```
   * @param ros_node_ptr Shared pointer to the associated ROS 2 node.
   * @param tree_node_loader_ptr  Shared pointer to an existing behavior tree node loader that the build handler may
   * work with.
   */
  TreeBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr);

  virtual ~TreeBuildHandler() = default;

  /**
   * @brief Specify the behavior tree build request encoded in a string.
   *
   * Additionally, you may provide an associated node manifest and a specific entry point. When
   * using TreeExecutorNode, all arguments are populated using the respective parameters of the incoming
   * `StartTreeExecutor` action goal. It's up to the specific implementation, if and how they are interpreted.
   *
   * The intention with the boolean return value is to indicate whether the respective behavior tree is
   * allowed to be built or not. Only if this returns `true`, TreeBuildHandler::buildTree is to be called afterwards.
   * The user must stick to this design pattern and implement this function accordingly.
   *
   * By default, this callback always returns `true`.
   *
   * @param build_request Request that specifies how to build the behavior tree encoded in a string.
   * @param entry_point Single point of entry for behavior execution.
   * @param node_manifest Behavior tree node manifest that specifies which nodes to use and how to load them.
   * @return `true` if the build handler accepts the request, `false` if it is rejected.
   */
  virtual bool setBuildRequest(
    const std::string & build_request, const std::string & entry_point, const NodeManifest & node_manifest);

  /**
   * @brief Build the behavior tree specified before.
   *
   * Typically, the build handler stores the information received when TreeBuildHandler::setBuildRequest was called. As
   * soon as this function is invoked, this information should be looked up and used to configure the corresponding
   * behavior tree with @p doc. Optionally, initial values for the global blackboard may be set using @p bb.
   * @param doc Reference to the behavior tree document that is used to instantiate the tree that is built by this
   * function.
   * @param bb Reference to the global blackboard that is used as a parent blackboard when instantiating the tree that
   * is built by this function.
   * @return Tree element representing one of the trees inside @p doc. It is used to determine the root tree.
   */
  virtual TreeDocument::TreeElement buildTree(TreeDocument & doc, TreeBlackboard & bb) = 0;

  /**
   * @brief Get a shared pointer to the parent `rclcpp::Node` of this build handler.
   * @return Shared pointer of the associated ROS 2 node.
   */
  rclcpp::Node::SharedPtr getRosNodePtr() const;

  /**
   * @brief Get a shared pointer to the class loader instance used for loading the required behavior tree nodes.
   * @return Shared pointer of the associated behavior tree node loader.
   */
  NodeLoader::SharedPtr getNodeLoaderPtr() const;

protected:
  /// ROS 2 logger initialized with the name of the build handler.
  const rclcpp::Logger logger_;

private:
  rclcpp::Node::WeakPtr ros_node_wptr_;
  NodeLoader::SharedPtr tree_node_loader_ptr;
};

}  // namespace auto_apms_behavior_tree