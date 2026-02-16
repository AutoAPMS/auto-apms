cmake_minimum_required(VERSION 3.12)
project(auto_apms_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)

#
# ------------ Configuration --------------
#

set(_AUTO_APMS_BEHAVIOR_TREE__BUILD_DIR_ABSOLUTE "${_AUTO_APMS_UTIL__THIS_PACKAGE_BUILD_DIR_ABSOLUTE}")

# Behavior tree executor defaults
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME "tree_executor")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX "/start")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX "/cmd")
set(_AUTO_APMS_BEHAVIOR_TREE__CLEAR_BLACKBOARD_SERVICE_NAME_SUFFIX "/clear_blackboard")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_OTHER_BUILD_HANDLERS "allow_other_build_handlers")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD "allow_dynamic_blackboard")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_SCRIPTING_ENUMS "allow_dynamic_scripting_enums")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_NODE "node_exclude_packages")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_BUILD_HANDLER "build_handler_exclude_packages")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER "build_handler")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE "tick_rate")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT "groot2_port")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER "state_change_logger")

# Builtin behavior tree node names
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_LOGGER "Logger")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_THROW_EXCEPTION "Error")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_START_EXECUTOR "StartExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_TERMINATE_EXECUTOR "TerminateExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_HALT_EXECUTOR "HaltExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_PAUSE_EXECUTOR "PauseExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_RESUME_EXECUTOR "ResumeExecutor")

set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER "SetParameter")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL "SetParameterBool")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT "SetParameterInt")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE "SetParameterDouble")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING "SetParameterString")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BYTE_VEC "SetParameterByteVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL_VEC "SetParameterBoolVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT_VEC "SetParameterIntVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE_VEC "SetParameterDoubleVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING_VEC "SetParameterStringVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER "GetParameter")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL "GetParameterBool")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT "GetParameterInt")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE "GetParameterDouble")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING "GetParameterString")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BYTE_VEC "GetParameterByteVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL_VEC "GetParameterBoolVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT_VEC "GetParameterIntVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE_VEC "GetParameterDoubleVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING_VEC "GetParameterStringVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_HAS_PARAMETER "HasParameter")

set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_BOOL "SetBool")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_EMPTY "SetEmpty")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_TRIGGER "SetTrigger")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_PUBLISH_POSE "PublishPose")

#
# ------------ Library --------------
#

# Generate parameter headers employing https://github.com/PickNikRobotics/generate_parameter_library
configure_file("config/executor_params.yaml.in" "${_AUTO_APMS_BEHAVIOR_TREE__BUILD_DIR_ABSOLUTE}/executor_params.yaml")
set(_temp ${CMAKE_CURRENT_SOURCE_DIR})  # WORKAROUND: temporarily change CMAKE_CURRENT_SOURCE_DIR since the function doesn't allow to load the parameter definition file from a different directory
set(CMAKE_CURRENT_SOURCE_DIR "${_AUTO_APMS_BEHAVIOR_TREE__BUILD_DIR_ABSOLUTE}")
generate_parameter_library(${PROJECT_NAME}_executor_params "executor_params.yaml")
set(CMAKE_CURRENT_SOURCE_DIR ${_temp})

add_library(${PROJECT_NAME} SHARED
  "src/build_handler/build_handler.cpp"
  "src/build_handler/build_handler_loader.cpp"
  "src/executor/state_observer.cpp"
  "src/executor/executor_base.cpp"
  "src/executor/executor_node.cpp"
  "src/util/parameter.cpp"
  "src/util/node.cpp"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PUBLIC
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX}"
  _AUTO_APMS_BEHAVIOR_TREE__CLEAR_BLACKBOARD_SERVICE_NAME_SUFFIX="${_AUTO_APMS_BEHAVIOR_TREE__CLEAR_BLACKBOARD_SERVICE_NAME_SUFFIX}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_OTHER_BUILD_HANDLERS="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_OTHER_BUILD_HANDLERS}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_SCRIPTING_ENUMS="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_SCRIPTING_ENUMS}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_NODE="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_NODE}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_BUILD_HANDLER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_BUILD_HANDLER}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT}"
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER}"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS}
  auto_apms_util::auto_apms_util
  ${auto_apms_interfaces_TARGETS}
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core

  ${PROJECT_NAME}_executor_params
)

# Install the version header to the autogenerated headers dir also used by rosidl generators
ament_generate_version_header(${PROJECT_NAME} HEADER_PATH "${PROJECT_NAME}/version.hpp")

# Default tree executor server node. Available as a typical ROS node component and a standalone executable.
# We create a separate shared library for that since it's best practice when using class_loader::ClassLoader
# for loading plugins as you can read here https://wiki.ros.org/class_loader under section 2.8.
add_library(${PROJECT_NAME}_executor_node_components SHARED
  "src/executor/executor_node_components.cpp"
)
target_link_libraries(${PROJECT_NAME}_executor_node_components PUBLIC
  rclcpp_components::component
  ${PROJECT_NAME}
)
rclcpp_components_register_node(${PROJECT_NAME}_executor_node_components
  PLUGIN "${PROJECT_NAME}::TreeExecutorNode"
  EXECUTABLE "tree_executor"
)
rclcpp_components_register_nodes(${PROJECT_NAME}_executor_node_components
  "${PROJECT_NAME}::NoUndeclaredParamsExecutorNode"
  "${PROJECT_NAME}::OnlyScriptingEnumParamsExecutorNode"
  "${PROJECT_NAME}::OnlyBlackboardParamsExecutorNode"
  "${PROJECT_NAME}::OnlyInitialScriptingEnumParamsExecutorNode"
  "${PROJECT_NAME}::OnlyInitialBlackboardParamsExecutorNode"
)

#
# ------------ Standard Behavior Tree Nodes --------------
#

add_library(${PROJECT_NAME}_behavior_tree_nodes SHARED
  "src/util/parameter.cpp"

  "src/node/logger.cpp"
  "src/node/throw_exception.cpp"
  "src/node/start_executor.cpp"
  "src/node/command_executor.cpp"
  "src/node/set_parameter.cpp"
  "src/node/get_parameter.cpp"
  "src/node/has_parameter.cpp"
  "src/node/set_bool.cpp"
  "src/node/set_empty.cpp"
  "src/node/set_trigger.cpp"
  "src/node/publish_pose.cpp"
)
# Make parameter util available for standard nodes too, but don't include anything that's not also been added to the sources
target_include_directories(${PROJECT_NAME}_behavior_tree_nodes PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_behavior_tree_nodes PUBLIC
  ${PROJECT_NAME}_executor_params
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${auto_apms_interfaces_TARGETS}
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
)
auto_apms_behavior_tree_register_nodes(${PROJECT_NAME}_behavior_tree_nodes
  "${PROJECT_NAME}::Logger"
  "${PROJECT_NAME}::ThrowException"
  "${PROJECT_NAME}::StartExecutor"
  "${PROJECT_NAME}::TerminateExecutor"
  "${PROJECT_NAME}::HaltExecutor"
  "${PROJECT_NAME}::PauseExecutor"
  "${PROJECT_NAME}::ResumeExecutor"

  "${PROJECT_NAME}::SetParameter"
  "${PROJECT_NAME}::SetParameterBool"
  "${PROJECT_NAME}::SetParameterInt"
  "${PROJECT_NAME}::SetParameterDouble"
  "${PROJECT_NAME}::SetParameterString"
  "${PROJECT_NAME}::SetParameterByteVec"
  "${PROJECT_NAME}::SetParameterBoolVec"
  "${PROJECT_NAME}::SetParameterIntVec"
  "${PROJECT_NAME}::SetParameterDoubleVec"
  "${PROJECT_NAME}::SetParameterStringVec"
  "${PROJECT_NAME}::GetParameter"
  "${PROJECT_NAME}::GetParameterBool"
  "${PROJECT_NAME}::GetParameterInt"
  "${PROJECT_NAME}::GetParameterDouble"
  "${PROJECT_NAME}::GetParameterString"
  "${PROJECT_NAME}::GetParameterByteVec"
  "${PROJECT_NAME}::GetParameterBoolVec"
  "${PROJECT_NAME}::GetParameterIntVec"
  "${PROJECT_NAME}::GetParameterDoubleVec"
  "${PROJECT_NAME}::GetParameterStringVec"
  "${PROJECT_NAME}::HasParameter"

  "${PROJECT_NAME}::SetBool"
  "${PROJECT_NAME}::SetEmpty"
  "${PROJECT_NAME}::SetTrigger"
  "${PROJECT_NAME}::PublishPose"
  NODE_MANIFEST
  "config/standard_nodes.yaml.in"
  NODE_MANIFEST_ALIAS
  "behavior_tree_nodes"
  NODE_MODEL_HEADER_TARGET
  ${PROJECT_NAME}
)

#
# ------------ Standard Behavior Tree Build Handlers --------------
#

add_library(${PROJECT_NAME}_standard_build_handlers SHARED
  "src/build_handler/impl/tree_from_string.cpp"
  "src/build_handler/impl/tree_from_resource.cpp"
)
target_link_libraries(${PROJECT_NAME}_standard_build_handlers PUBLIC
  ${PROJECT_NAME}
)

# In the original package, we need to manually include this before being able to call the macro.
# However, it is automatically available for all downstream packages.
include("cmake/register_build_handlers.cmake")
auto_apms_behavior_tree_register_build_handlers(${PROJECT_NAME}_standard_build_handlers
  "${PROJECT_NAME}::TreeFromStringBuildHandler"
  "${PROJECT_NAME}::TreeFromResourceBuildHandler"
)

#
# ------------ Utility Executables --------------
#

# Run a behavior tree registered with the ament resource index
add_executable(run_behavior "src/cli/run_behavior.cpp")
target_link_libraries(run_behavior ${PROJECT_NAME})

# Run a single behavior tree node registered with the ament resource index
add_executable(run_tree_node "src/cli/run_tree_node.cpp")
target_link_libraries(run_tree_node ${PROJECT_NAME})

# Create a new behavior tree file with optionally a node model already loaded
add_executable(new_tree "src/cli/new_tree.cpp")
target_link_libraries(new_tree ${PROJECT_NAME})

#
# ------------ Packaging --------------
#

# Python tools
ament_python_install_package(${PROJECT_NAME})

# Generate node model xml for all native nodes
add_custom_command(
  OUTPUT "${_AUTO_APMS_UTIL__THIS_PACKAGE_BUILD_DIR_ABSOLUTE}/node_model_native.xml"
  COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MODEL_FOR_MANIFEST_RESOURCE_CMD}"
          "${_AUTO_APMS_UTIL__THIS_PACKAGE_BUILD_DIR_ABSOLUTE}/node_model_native.xml"
  COMMENT "Generating native nodes model XML"
  VERBATIM
)
add_custom_target(generate_node_model_native ALL
  DEPENDS "${_AUTO_APMS_UTIL__THIS_PACKAGE_BUILD_DIR_ABSOLUTE}/node_model_native.xml"
)

# Install the generated node model xml for all native nodes
install(
  FILES "${_AUTO_APMS_UTIL__THIS_PACKAGE_BUILD_DIR_ABSOLUTE}/node_model_native.xml"
  DESTINATION "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}"
)


# Export libraries
install(
  TARGETS
  ${PROJECT_NAME}_executor_params
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_srvs
  auto_apms_util
  auto_apms_interfaces
  auto_apms_behavior_tree_core
)

#
# ------------ Testing --------------
#

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  ament_add_gtest(${PROJECT_NAME}_unit_tests
    "test/unit/node_registration.cpp"
    "test/unit/tree_document_model_templates.cpp"
  )
  target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME})
endif()

# Plugins
install(
  TARGETS
  ${PROJECT_NAME}_behavior_tree_nodes
  ${PROJECT_NAME}_standard_build_handlers
  ${PROJECT_NAME}_executor_node_components
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
  run_behavior
  run_tree_node
  new_tree
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "auto_apms_behavior_tree-extras.cmake.in")
