# Copyright 2025 Robin Müller
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
# Add AutoAPMS behaviors to the package's resources.
#
# This macro registers files, adds them to the package's
# resources and provides a consistent way of grouping and discovering behavior definitions
# (behavior build requests) at runtime. These definitions are a central concept in AutoAPMS. Essentially, they act as
# blueprints for building and initializing behaviors in ROS 2.
#
# ARGN can contain relative or absolute file paths. The file contents act as instructions for building a behavior given
# to the build handler specified under BUILD_HANDLER.
#
# ARGN can also contain simple strings. Such arguments are directly forwarded to the behavior build handler specified
# under BUILD_HANDLER. This only makes sense if the build handler is able to interpret the string correctly.
#
# :param build_request: Relative path to a file containing a behavior definition or a simple string. This argument
#    determines the build request given to the behavior build handler provided with BUILD_HANDLER. The user must
#    make sure that the given build handler is able to interpret the given request.
# :type build_request: string
# :param BUILD_HANDLER: Fully qualified class name of the behavior tree build handler that should be
#    used by default to interpret the given behaviors.
# :type BUILD_HANDLER: string
# :param CATEGORY: Optional category name to which the behaviors belong.
#    If omitted, the default category is used.
# :type CATEGORY: string
# :param ALIAS: Optional name for the behavior resource. If omitted, the file stem respectively the simple string is used as a behavior's alias.
# :type ALIAS: string
# :param ENTRY_POINT: Single point of entry for behavior execution. For behavior trees, this usually is the name of the root tree,
#    but for other types of behaviors, this may be populated differently.
# :type ENTRY_POINT: string
# :param NODE_MANIFEST: One or more relative paths or resource identities of existing node manifests.
#   If specified, behavior tree nodes associated with this manifest can be
#   loaded automatically and are available for every behavior registered with this macro call.
# :type NODE_MANIFEST: list of strings
# :param MARK_AS_INTERNAL: If this option is set, the behavior is assigned a special category which indicates it is intended for internal use only.
# :type MARK_AS_INTERNAL: string
#
# @public
#
macro(auto_apms_behavior_tree_register_behavior build_request)

  # Parse arguments
  set(options MARK_AS_INTERNAL)
  set(oneValueArgs BUILD_HANDLER CATEGORY ALIAS ENTRY_POINT)
  set(multiValueArgs NODE_MANIFEST)
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT DEFINED ARGS_BUILD_HANDLER)
    message(
      FATAL_ERROR
      "auto_apms_behavior_tree_register_behavior(): The BUILD_HANDLER keyword is required. You must specify the fully qualified class name of the default behavior tree build handler used to create the behavior from the given definitions"
    )
  endif()

  set(_category "${_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY}")
  if(DEFINED ARGS_CATEGORY)
    set(_category "${ARGS_CATEGORY}")
  endif()
  if(ARGS_MARK_AS_INTERNAL)
    # If the behavior is marked as internal, append the internal category suffix
    set(_category "${_category}${_AUTO_APMS_BEHAVIOR_TREE_CORE__INTERNAL_BEHAVIOR_CATEGORY_SUFFIX}")
  endif()

  set(_entry_point "")
  if(DEFINED ARGS_ENTRY_POINT)
    set(_entry_point "${ARGS_ENTRY_POINT}")
  endif()

  # Check if category is valid
  string(REGEX MATCH "[^A-Za-z0-9_-]" _has_illegal "${_category}")
  if(_has_illegal)
    message(
      FATAL_ERROR
      "auto_apms_behavior_tree_register_behavior(): Category '${_category}' contains illegal characters. Only alphanumeric, '_' and '-' are allowed."
    )
  endif()

  # Check if entry_point is valid
  string(REGEX MATCH "[^A-Za-z0-9_-]" _has_illegal "${_entry_point}")
  if(_has_illegal)
    message(
      FATAL_ERROR
      "auto_apms_behavior_tree_register_behavior(): ENTRY_POINT '${_entry_point}' contains illegal characters. Only alphanumeric, '_' and '-' are allowed."
    )
  endif()

  set(_behavior_file_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__BEHAVIOR}/${_category}")

  get_filename_component(_behavior_file_abs_path__source "${build_request}" REALPATH)
  get_filename_component(_behavior_alias "${build_request}" NAME_WE) # Simply returns the string itself if it is not a file path

  # If provided, overwrite the alias for the behavior
  if(DEFINED ARGS_ALIAS)
    set(_behavior_alias "${ARGS_ALIAS}")
  endif()

  # Check if alias is valid
  string(REGEX MATCH "[^A-Za-z0-9_:-]" _has_illegal "${_behavior_alias}")
  if(_has_illegal)
    message(
      FATAL_ERROR
      "auto_apms_behavior_tree_register_behavior(): Behavior alias ${_behavior_alias} for '${build_request}' contains illegal characters. Only alphanumeric, '_', '-', and ':' are allowed. Make sure to change the macro's arguments or specify a valid alias."
    )
  endif()

  # Verify no duplicate behaviors
  if("${_category}${_behavior_alias}" IN_LIST _all_behaviors)
    message(
      FATAL_ERROR
      "auto_apms_behavior_tree_register_behavior(): '${build_request}' is aliased with '${_behavior_alias}', but this alias was already used to register a behavior in category '${_category}'. An alias must be unique per category and package. Use the ALIAS keyword to specify a unique alias for behaviors."
    )
  endif()
  list(APPEND _all_behaviors "${_category}${_behavior_alias}")

  set(_metadata_id "${_category}__${_behavior_alias}")

  # Determine the build request
  if(EXISTS "${_behavior_file_abs_path__source}")
    get_filename_component(_file_name "${_behavior_file_abs_path__source}" NAME)

    # Track the behavior file so CMake knows it's an input dependency
    # Make sure to give the file a unique name
    set(_file_name__unique "${_metadata_id}__${_file_name}")
    configure_file(
      "${_behavior_file_abs_path__source}"
      "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${_file_name__unique}"
      COPYONLY
    )

    # Install the file (must be the source file for symlinks to work correctly)
    install(
      FILES "${_behavior_file_abs_path__source}"
      DESTINATION "${_behavior_file_rel_dir__install}"
      RENAME "${_file_name__unique}"
    )

    # Fill build request with install path
    set(_build_request_field "${_behavior_file_rel_dir__install}/${_file_name__unique}")
  else()
    # Fill build request with raw string
    set(_build_request_field "${build_request}")
  endif()

  auto_apms_behavior_tree_generate_node_metadata_hybrid_manifest_args("${_metadata_id}" ${ARGS_NODE_MANIFEST})

  # Populate resource file variable
  set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR}${_category}|${_behavior_alias}|${ARGS_BUILD_HANDLER}|${_build_request_field}|${_entry_point}|${_auto_apms_behavior_tree_generate_node_metadata_hybrid_manifest_args__node_manifest_rel_paths__install}\n")

endmacro()
