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
# :param NODE_REGISTRATION_TYPE: Fully qualified class name for the node registration
#   mechanism. This is useful when the node registration requires a custom approach that
#   differs from the default NodeRegistrationTemplate factory template. The specified
#   class must implement the NodeRegistrationInterface.
#   Defaults to "auto_apms_behavior_tree::core::NodeRegistrationTemplate<>".
#   A trailing <> indicates a factory template class: the class names from ARGN are
#   substituted into the template (e.g. NodeRegistrationTemplate<MyClass>), and the
#   stripped prefix is passed as FACTORY_TEMPLATE_CLASS to auto_apms_util_register_plugins.
#   If the value does NOT end with <>, the ARGN class names are registered directly as
#   NodeRegistrationInterface subclasses without template wrapping.
# :type NODE_REGISTRATION_TYPE: string
#
# @public
#
macro(auto_apms_behavior_tree_register_nodes target)

  # Parse arguments
  set(options "")
  set(oneValueArgs NODE_MANIFEST_ALIAS NODE_MODEL_HEADER_TARGET NODE_REGISTRATION_TYPE)
  set(multiValueArgs NODE_MANIFEST)
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Default NODE_REGISTRATION_TYPE: auto_apms_behavior_tree::core::NodeRegistrationTemplate<>
  # The trailing <> indicates that this is a factory template class. The class names passed
  # as ARGN will be substituted into the template (e.g. NodeRegistrationTemplate<MyClass>).
  # If the value does NOT end with <>, it means the ARGN class names are registered directly
  # (i.e. the class itself is a NodeRegistrationInterface subclass, not template-wrapped).
  if("${ARGS_NODE_REGISTRATION_TYPE}" STREQUAL "")
    set(ARGS_NODE_REGISTRATION_TYPE "auto_apms_behavior_tree::core::NodeRegistrationTemplate<>")
  endif()

  # Calling this macro without any node classes is valid, if one only wants to register configurations
  # through the NODE_MANIFEST keyword. So we must check for that here.
  if(NOT "${ARGS_UNPARSED_ARGUMENTS}" STREQUAL "")
    # Determine if NODE_REGISTRATION_TYPE is a factory template (ends with <>)
    string(REGEX MATCH "<>$" _is_factory_template "${ARGS_NODE_REGISTRATION_TYPE}")

    if(_is_factory_template)
      # Template factory mode: strip trailing <> to get the template class prefix
      string(REGEX REPLACE "<>$" "" _factory_template_class "${ARGS_NODE_REGISTRATION_TYPE}")
      auto_apms_util_register_plugins(
        ${target}
        "auto_apms_behavior_tree::core::NodeRegistrationInterface"
        ${ARGS_UNPARSED_ARGUMENTS}
        FACTORY_TEMPLATE_CLASS "${_factory_template_class}"
      )
    else()
      # Direct registration mode: the class itself implements NodeRegistrationInterface
      auto_apms_util_register_plugins(
        ${target}
        "auto_apms_behavior_tree::core::NodeRegistrationInterface"
        ${ARGS_UNPARSED_ARGUMENTS}
      )
    endif()

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

    # Append build information of the specified node plugins (<class_name>@<library_path>@<registration_type>).
    # Make sure to do before calling generating the node metadata (Otherwise build info would be unavailable).
    foreach(_class_name ${ARGS_UNPARSED_ARGUMENTS})
      if(_is_factory_template)
        set(_registration_type "${_factory_template_class}<${_class_name}>")
      else()
        set(_registration_type "${_class_name}")
      endif()
      list(APPEND _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_BUILD_INFO "${_class_name}@$<TARGET_FILE:${target}>@${_registration_type}")
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
