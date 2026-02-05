cmake_minimum_required(VERSION 3.12)
project(auto_apms_behavior_tree_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(auto_apms_util REQUIRED)

#
# ------------ Configuration --------------
#

set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP "\\n")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP "|")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP "/")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP "::")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_NAMESPACE_DEFAULT_SEP ".")

# Core resources
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR "auto_apms_behavior_tree_core__behavior")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST "auto_apms_behavior_tree_core__node_manifest")

set(_AUTO_APMS_BEHAVIOR_TREE_CORE__INTERNAL_BEHAVIOR_CATEGORY_SUFFIX "__internal")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY "default")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE "tree")

# Node manifest registration parameters
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_CLASS "class_name")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_DESCRIPTION "description")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_ROS2TOPIC "topic")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_PORT_ALIAS "port_alias")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_PORT_DEFAULT "port_default")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT "wait_timeout")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT "request_timeout")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_ALLOW_UNREACHABLE "allow_unreachable")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_LOGGER_LEVEL "logger_level")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_HIDDEN_PORTS "hidden_ports")
set(_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_EXTRA "extra")

#
# ------------ Library --------------
#

add_library(${PROJECT_NAME} SHARED
  "src/node/node_registration_options.cpp"
  "src/node/node_model_type.cpp"
  "src/node/node_manifest.cpp"
  "src/node/ros_node_context.cpp"
  "src/node/node_registration_loader.cpp"
  "src/tree/script.cpp"
  "src/tree/tree_resource.cpp"
  "src/tree/tree_document.cpp"
  "src/behavior.cpp"
  "src/builder.cpp"
  "src/convert.cpp"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST="${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__INTERNAL_BEHAVIOR_CATEGORY_SUFFIX="${_AUTO_APMS_BEHAVIOR_TREE_CORE__INTERNAL_BEHAVIOR_CATEGORY_SUFFIX}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY="${_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE="${_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_CLASS="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_CLASS}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_DESCRIPTION="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_DESCRIPTION}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_ROS2TOPIC="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_ROS2TOPIC}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_PORT_ALIAS="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_PORT_ALIAS}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_PORT_DEFAULT="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_PORT_DEFAULT}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_ALLOW_UNREACHABLE="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_ALLOW_UNREACHABLE}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_LOGGER_LEVEL="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_LOGGER_LEVEL}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_HIDDEN_PORTS="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_HIDDEN_PORTS}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_EXTRA="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_EXTRA}"
)
target_compile_definitions(${PROJECT_NAME} PUBLIC
  _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP="${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP="${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP="${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP="${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_NAMESPACE_DEFAULT_SEP="${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_NAMESPACE_DEFAULT_SEP}"
  _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR="${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR}"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${action_msgs_TARGETS}
  ${TinyXML2_TARGETS}
  Eigen3::Eigen
  behaviortree_cpp::behaviortree_cpp
  auto_apms_util::auto_apms_util
)

# Install the version header to the autogenerated headers dir also used by rosidl generators
ament_generate_version_header(${PROJECT_NAME} HEADER_PATH "${PROJECT_NAME}/version.hpp")

#
# ------------ Utility Executables --------------
#

# Merge multiple node manifest files to a single one and collect the shared library paths for the required nodes
add_executable(create_node_manifest "src/cli/create_node_manifest.cpp")
target_link_libraries(create_node_manifest ${PROJECT_NAME})

# Generate the node model according to a node manifest
add_executable(create_node_model "src/cli/create_node_model.cpp")
target_link_libraries(create_node_model ${PROJECT_NAME})

# Generate the node model according to a node manifest resource identity or native nodes
add_executable(create_node_model_for_manifest_resource "src/cli/create_node_model_for_manifest_resource.cpp")
target_link_libraries(create_node_model_for_manifest_resource ${PROJECT_NAME})

# Generate a C++ header containing classes for all registered nodes intended to be used with the TreeBuilder API
add_executable(create_node_model_header "src/cli/create_node_model_header.cpp")
target_link_libraries(create_node_model_header ${PROJECT_NAME})

# Generate the behavior tree node documentation for all nodes available in this workspace
add_executable(create_node_reference_markdown "src/cli/create_node_reference_markdown.cpp")
target_link_libraries(create_node_reference_markdown ${PROJECT_NAME})

### Quick test utilities to be used manually (not installed or exported) for debugging and development purposes

# Quick test utility to print node manifest identities
add_executable(test_print_node_manifest_identities "test/test_print_node_manifest_identities.cpp")
target_link_libraries(test_print_node_manifest_identities ${PROJECT_NAME})

# Quick test utility to print behavior resource identities
add_executable(test_print_behavior_identities "test/test_print_behavior_identities.cpp")
target_link_libraries(test_print_behavior_identities ${PROJECT_NAME})

#
# ------------ Testing --------------
#

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(example_interfaces REQUIRED)

  ament_copyright()

  ament_add_gtest(${PROJECT_NAME}_unit_tests
    "test/unit/node_manifest.cpp"
    "test/unit/ros_subscriber_node.cpp"
    "test/unit/tree_document_node_model.cpp"
    "test/unit/tree_document_recursive_include.cpp"
    "test/unit/tree_document_tree_generation.cpp"
  )
  target_link_libraries(${PROJECT_NAME}_unit_tests
    ${PROJECT_NAME}
    ${example_interfaces_TARGETS}
  )
endif()

#
# ------------ Packaging --------------
#

# Python tools
ament_python_install_package(${PROJECT_NAME})

# Export libraries
install(
  TARGETS
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
  action_msgs
  tinyxml2_vendor # Must be exported instead of TinyXML2 target
  eigen3_cmake_module
  Eigen3
  behaviortree_cpp
  auto_apms_util
)

install(
  TARGETS
  create_node_manifest
  create_node_model
  create_node_model_for_manifest_resource
  create_node_model_header
  create_node_reference_markdown
  DESTINATION "lib/${PROJECT_NAME}"
)

install(
  PROGRAMS
  "${PROJECT_NAME}/util/find_tree_names.py"
  DESTINATION "lib/${PROJECT_NAME}"
)

install(
  DIRECTORY "include/${PROJECT_NAME}"
  DESTINATION "include"
)

install(
  DIRECTORY "cmake"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package(CONFIG_EXTRAS "auto_apms_behavior_tree_core-extras.cmake.in")
