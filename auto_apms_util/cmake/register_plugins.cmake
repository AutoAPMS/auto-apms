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
# Register plugins for CMake according to the pluginlib style
# (See also http://wiki.ros.org/pluginlib).
#
# This macro populates a variable holding the content of the
# plugin XML manifest file which is added to the package's
# ament_index resources by register_plugins_hook.cmake
# invoked by each ament_package call, once auto_apms_util is discovered
# using find_package.
# See also PluginClassLoader for more information on how to load
# plugins registered using this macro.
#
# :param target: Shared library target implementing the plugins
#   specified under ARGN.
# :type target: string
# :param base_class: Fully qualified name of the plugin base class.
# :type base_class: string
# :param ARGN: The unique names of plugin classes being registered with this
#   macro call and exported by the shared library target.
# :type ARGN: list of strings
# :param FACTORY_TEMPLATE_CLASS: If specified, the plugin classes are configured
#  to be loadable using a factory template class. That is, the plugin's type is
#  set to FactoryTemplateClass<MyClass>, so there must be a class called
#  FactoryTemplateClass that takes the plugin class type as a template argument.
# :type FACTORY_TEMPLATE_CLASS: string
#
# @public
#
macro(auto_apms_util_register_plugins target base_class)

  if(NOT TARGET ${target})
    message(
    FATAL_ERROR
    "auto_apms_util_register_plugins(): '${target}' is not a target.")
  endif()

  # Check target type
  get_target_property(_target_type ${target} TYPE)
  if(NOT _target_type STREQUAL "SHARED_LIBRARY")
    message(
    FATAL_ERROR
    "auto_apms_util_register_plugins(): '${target}' is not a shared library target.")
  endif()

  # Parse arguments
  set(options "")
  set(oneValueArgs FACTORY_TEMPLATE_CLASS)
  set(multiValueArgs "")
  cmake_parse_arguments(_register_plugins_ "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  foreach(_class_name ${_register_plugins__UNPARSED_ARGUMENTS})
    if("${_class_name}" IN_LIST _AUTO_APMS_UTIL__PLUGIN_CLASS_NAMES)
      message(
      FATAL_ERROR
      "auto_apms_util_register_plugins(): Class name '${_class_name}' has already been registered before.")
    endif()

    # Append all class names to a list to keep track of all registered classes
    list(APPEND _AUTO_APMS_UTIL__PLUGIN_CLASS_NAMES "${_class_name}")

    # Append to the variable that holds the content of the pluginlib plugins.xml file
    if("${_register_plugins__FACTORY_TEMPLATE_CLASS}" STREQUAL "")
      set(_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT "${_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT}<library path=\"${target}\"><class name=\"${_class_name}\" type=\"${_class_name}\" base_class_type=\"${base_class}\" /></library>\n")
    else()
      set(_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT "${_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT}<library path=\"${target}\"><class name=\"${_class_name}\" type=\"${_register_plugins__FACTORY_TEMPLATE_CLASS}<${_class_name}>\" base_class_type=\"${base_class}\" /></library>\n")
    endif()
  endforeach()

endmacro()