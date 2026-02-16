# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Add behavior tree node plugins to the package's resources.
#
# This macro must be called to make behavior tree node plugins available
# at runtime and configure their registration with the behavior tree
# factory. Optionally, a corresponding node model header is generated.
# This header facilitates integrating the specified nodes when building
# behavior trees using the TreeDocument API.
#
# :param target: Shared library target implementing the behavior tree
#   nodes registered under ARGN. If ARGN is empty, this macro only generates metadata for the
#   manifests given under NODE_MANIFEST without registering any plugins. In this case, the target
#   argument is interpreted as the node manifest alias for which the metadata is generated,
#   and the generated metadata is exported under that alias. The alias can also be set by specifying
#   the NODE_MANIFEST_ALIAS argument.
# :type target: string
# :param ARGN: The unique names of node classes being registered with this
#   macro call and exported by the shared library target.
# :type ARGN: list of strings
# :param NODE_MANIFEST: One or more relative paths or existing resource identities of node manifests.
#   Multiple file paths are concatenated to a single one.
# :type NODE_MANIFEST: list of strings
# :param NODE_MANIFEST_ALIAS: An optional alias for the node manifest resulting from NODE_MANIFEST.
#    By default, target will be used as alias, but this can be overridden by specifying this argument.
# :type NODE_MANIFEST_ALIAS: string
# :param NODE_MODEL_HEADER_TARGET: Name of a shared library target. If specified,
#   generate a C++ header that defines model classes for all behavior tree
#   nodes specified inside the node manifest files provided under NODE_MANIFEST and
#   add it to the includes of the given target.
# :type NODE_MODEL_HEADER_TARGET: string
#
# @public
#
macro(auto_apms_behavior_tree_register_nodes target)

  # Parse arguments
  set(options "")
  set(oneValueArgs NODE_MANIFEST_ALIAS NODE_MODEL_HEADER_TARGET)
  set(multiValueArgs NODE_MANIFEST)
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Calling this macro without any node classes is valid, if one only wants to register configurations
  # through the NODE_MANIFEST keyword. So we must check for that here.
  if(NOT "${ARGS_UNPARSED_ARGUMENTS}" STREQUAL "")
    # Register the specified node classes as plugins
    auto_apms_util_register_plugins(
      ${target}
      "auto_apms_behavior_tree::core::NodeRegistrationInterface"
      ${ARGS_UNPARSED_ARGUMENTS}
      FACTORY_TEMPLATE_CLASS "auto_apms_behavior_tree::core::NodeRegistrationTemplate"
    )

    # IMPORTANT: Disable LTO (Link-Time Optimization) for behavior tree node plugin libraries.
    #
    # Problem: When building on platforms that enable LTO by default (e.g., ROS 2 build farms
    # where dpkg-buildflags includes -flto=auto), the create_node_model CLI tool
    # crashes with a segfault during BT::writeTreeNodesModelXML(). This segfault is caught during
    # build time as we call the create_node_model executable as a custom command for generating
    # the node models for the standard behavior tree nodes that auto_apms_behavior_tree
    # comes with. It would also occur at runtime when loading any LTO-compiled plugin libraries
    # registered using this macro.
    #
    # Root cause: LTO optimization occurs within each shared library's compilation unit in isolation.
    # This plugin library instantiates BehaviorTree.CPP templates (e.g., registerNodeType<T>) that
    # manipulate internal data structures of the BT::BehaviorTreeFactory. When the LTO-compiled
    # plugin is loaded at runtime via dlopen() by a CLI tool linked against behaviortree_cpp
    # (which was compiled as a separate shared library), the optimized code makes assumptions
    # about memory layout and function signatures that may not hold across the shared library
    # boundary. Even if both libraries are built with LTO, the optimization cannot cross the
    # dlopen() boundary, leading to ABI incompatibilities and crashes.
    #
    # Solution: Disable LTO for these plugin libraries to ensure the template instantiations
    # produce standard ABI-compliant code that works correctly when loaded by any tool linked
    # against behaviortree_cpp, regardless of how that library was compiled.
    set_target_properties(${target} PROPERTIES
      INTERPROCEDURAL_OPTIMIZATION FALSE
      INTERPROCEDURAL_OPTIMIZATION_RELEASE FALSE
    )
    target_compile_options(${target} PRIVATE
      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fno-lto>
      $<$<COMPILE_LANG_AND_ID:C,GNU>:-fno-lto>
      $<$<COMPILE_LANG_AND_ID:CXX,Clang>:-fno-lto>
      $<$<COMPILE_LANG_AND_ID:C,Clang>:-fno-lto>
    )
    target_link_options(${target} PRIVATE
      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fno-lto>
      $<$<COMPILE_LANG_AND_ID:C,GNU>:-fno-lto>
      $<$<COMPILE_LANG_AND_ID:CXX,Clang>:-fno-lto>
      $<$<COMPILE_LANG_AND_ID:C,Clang>:-fno-lto>
    )

    # Append build information of the specified node plugins (<class_name>@<library_path>).
    # Make sure to do before calling generating the node metadata (Otherwise build info would be unavailable).
    foreach(_class_name ${ARGS_UNPARSED_ARGUMENTS})
      list(APPEND _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_BUILD_INFO "${_class_name}@$<TARGET_FILE:${target}>")
    endforeach()
  endif()

  # Automatically create node metadata if any manifest files are provided
  if("${ARGS_NODE_MANIFEST}" STREQUAL "")
    if(NOT "${ARGS_NODE_MANIFEST_ALIAS}" STREQUAL "")
        message(WARNING
            "auto_apms_behavior_tree_register_nodes(): Argument NODE_MANIFEST_ALIAS requires that you also specify NODE_MANIFEST. Unless you don't specify both arguments, NODE_MANIFEST_ALIAS will be ignored."
        )
    endif()
    if(NOT "${ARGS_NODE_MODEL_HEADER_TARGET}" STREQUAL "")
        message(WARNING
            "auto_apms_behavior_tree_register_nodes(): Argument NODE_MODEL_HEADER_TARGET requires that you also specify NODE_MANIFEST. Unless you don't specify both arguments, NODE_MODEL_HEADER_TARGET will be ignored."
        )
    endif()
  else()
    # Apply metadata ID aliasing if a manifest alias is provided, otherwise use the target name as default id.
    set(metadata_id "${target}")
    if(NOT "${ARGS_NODE_MANIFEST_ALIAS}" STREQUAL "")
      set(metadata_id "${ARGS_NODE_MANIFEST_ALIAS}")
    endif()

    # Generate node metadata for the specified manifests and optionally generate a node model header.
    auto_apms_behavior_tree_generate_node_metadata(
      "${metadata_id}"
      ${ARGS_NODE_MANIFEST}
      NODE_MODEL_HEADER_TARGET
      "${ARGS_NODE_MODEL_HEADER_TARGET}"
    )
  endif()

endmacro()
