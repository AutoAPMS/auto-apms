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
# Register behavior trees with the package's resources.
#
# This macro makes behavior trees available to the ROS 2 workspace by
# exposing the corresponding XML and node manifest files as ament_index
# resources.
#
# :param ARGN: Behavior tree XML files to be added to this package's resources.
# :type ARGN: list of files
# :param ALIAS_NAMESPACE: Option to customize the alias namespace used for all behavior trees to be registered.
#   The default is to use the XML file stem as namspace.
# :param NODE_MANIFEST: One or more relative paths or resource identities of existing node manifests.
#   If specified, behavior tree nodes associated with this manifest can be
#   loaded automatically and are available for every tree under ARGN.
# :type NODE_MANIFEST: list of strings
# :param MARK_AS_INTERNAL: If this option is set, the trees are assigned a special category which indicates that the
#    behavior is intended for internal use only.
# :type MARK_AS_INTERNAL: string
#
# @public
#
macro(auto_apms_behavior_tree_register_trees)

  # Parse arguments
  set(options MARK_AS_INTERNAL)
  set(oneValueArgs ALIAS_NAMESPACE)
  set(multiValueArgs NODE_MANIFEST)
  cmake_parse_arguments(_register_trees_ "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  foreach(_arg ${_register_trees__UNPARSED_ARGUMENTS})
    # Check if behavior tree file exists
    get_filename_component(_tree_abs_path__source "${_arg}" REALPATH)
    if(NOT EXISTS "${_tree_abs_path__source}")
      message(
        FATAL_ERROR
        "auto_apms_behavior_tree_register_trees(): Behavior tree file ${_arg} does not exist."
      )
    endif()

    if(NOT DEFINED _register_trees__ALIAS_NAMESPACE)
      # Verify that the file hasn't been registered.
      if("${_tree_abs_path__source}" IN_LIST _package_tree_file_abs_paths__source)
        message(
          FATAL_ERROR
          "auto_apms_behavior_tree_register_trees(): Behavior tree file ${_arg} has already been registered. This would create an ambigious resource identity. Use the ALIAS_NAMESPACE argument to assign a unique namespace."
        )
      endif()
      list(APPEND _package_tree_file_abs_paths__source "${_tree_abs_path__source}")
    endif()

    # Collect all available behavior tree IDs
    execute_process(
      COMMAND
      "${_AUTO_APMS_BEHAVIOR_TREE_CORE__FIND_TREE_NAMES_CMD}"
      "${_tree_abs_path__source}"
      OUTPUT_VARIABLE _tree_file_tree_names
      RESULT_VARIABLE _return_code
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _return_code EQUAL 0)
      message(
        FATAL_ERROR
        "auto_apms_behavior_tree_register_trees(): Failed to get a list of all behavior tree names contained in ${_arg}."
      )
    endif()

    # Check if there are any behavior trees
    if("${_tree_file_tree_names}" STREQUAL "")
      message(
        FATAL_ERROR
        "auto_apms_behavior_tree_register_trees(): Behavior tree file ${_arg} doesn't specify any valid behavior trees."
      )
    endif()

    # Verify no duplicate tree IDs inside file
    set(_seen_names "")
    foreach(_name ${_tree_file_tree_names})
      if("${_name}" IN_LIST _seen_names)
        message(
          FATAL_ERROR
          "auto_apms_behavior_tree_register_trees(): Behavior tree with name '${_name}' exists multiple times in file ${_arg}."
        )
      endif()
      list(APPEND _seen_names "${_name}")
    endforeach()

    # Assign tree resource namespace
    if(DEFINED _register_trees__ALIAS_NAMESPACE)
      set(_tree_resource_alias_namespace "${_register_trees__ALIAS_NAMESPACE}")
    else()
      get_filename_component(_tree_resource_alias_namespace "${_arg}" NAME_WE)
    endif()

    # Prevent ambiguous tree identities
    set(_index 0)
    foreach(_ns ${_package_tree_resource_alias_namespaces})
      # Check if a tree resource with the same alias namespace has been registered before.
      if("${_ns}" STREQUAL "${_tree_resource_alias_namespace}")
        # Check if this tree registers trees with the same name
        list(GET _package_tree_names "${_index}" _names_associated_with_ns)
        string(REPLACE "|" ";" _names_associated_with_ns "${_names_associated_with_ns}")
        foreach(_name ${_names_associated_with_ns})
          list(FIND _tree_file_tree_names "${_name}" _index2)
          if(_index2 GREATER -1)
            message(
              FATAL_ERROR
              "auto_apms_behavior_tree_register_trees(): Tree name '${_name}' found in behavior tree file ${_arg} has already been registered under ALIAS_NAMESPACE=${_ns}. This would create an ambigious resource identity (${PROJECT_NAME}::${_ns}::${_name})."
            )
          endif()
        endforeach()
      endif()
      math(EXPR _index "${_index} + 1")
    endforeach()

    # Store file stem and associated tree names
    set(_temp_names "${_tree_file_tree_names}")
    string(REPLACE ";" "|" _temp_names "${_temp_names}")
    list(APPEND _package_tree_names "${_temp_names}")
    list(APPEND _package_tree_resource_alias_namespaces "${_tree_resource_alias_namespace}")

    # Register all trees inside the file as individual behaviors
    foreach(_tree_name ${_tree_file_tree_names})
      set(_register_args
          "${_tree_abs_path__source}"
          BUILD_HANDLER "auto_apms_behavior_tree::TreeFromStringBuildHandler"
          CATEGORY "${_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE}"
          ALIAS "${_tree_resource_alias_namespace}::${_tree_name}"
          ENTRY_POINT "${_tree_name}"
          NODE_MANIFEST ${_register_trees__NODE_MANIFEST}
      )
      if(_register_trees__MARK_AS_INTERNAL)
        list(APPEND _register_args MARK_AS_INTERNAL)
      endif()
      auto_apms_behavior_tree_register_behavior(${_register_args})
    endforeach()
  endforeach()

endmacro()
