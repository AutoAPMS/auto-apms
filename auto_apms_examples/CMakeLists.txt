cmake_minimum_required(VERSION 3.9)
project(auto_apms_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_mission REQUIRED)

#
# ------------ Example Libraries --------------
#

add_library(${PROJECT_NAME}_example_nodes SHARED
  "src/simple_skill_node.cpp"
)
target_link_libraries(${PROJECT_NAME}_example_nodes PUBLIC
  rclcpp_components::component
  ${auto_apms_interfaces_TARGETS}
  auto_apms_util::auto_apms_util
  auto_apms_behavior_tree::auto_apms_behavior_tree
)

add_library(${PROJECT_NAME}_example_build_handlers SHARED
  "src/simple_skill_build_handler.cpp"
)
target_link_libraries(${PROJECT_NAME}_example_build_handlers auto_apms_behavior_tree::auto_apms_behavior_tree)

auto_apms_behavior_tree_register_nodes(${PROJECT_NAME}_example_nodes
  "auto_apms_examples::SimpleSkillClient"
  NODE_MANIFEST
  "behavior/simple_skill_node_manifest.yaml"
  NODE_MANIFEST_ALIAS
  "example_nodes"
  NODE_MODEL_HEADER_TARGET
  ${PROJECT_NAME}_example_build_handlers
)

auto_apms_behavior_tree_register_build_handlers(${PROJECT_NAME}_example_build_handlers
  "auto_apms_examples::SimpleSkillBuildHandler"
)

#
# ------------ Example Executables --------------
#

rclcpp_components_register_node(${PROJECT_NAME}_example_nodes
  PLUGIN "auto_apms_examples::SimpleSkillServer"
  EXECUTABLE "simple_skill_server"
)

#
# ------------ Example Resources --------------
#

auto_apms_behavior_tree_register_trees(
  "behavior/simple_skill_tree.xml"
  NODE_MANIFEST
  "${PROJECT_NAME}::example_nodes"
)

auto_apms_behavior_tree_register_trees(
  "behavior/demo.xml"
  NODE_MANIFEST
  "auto_apms_behavior_tree::behavior_tree_nodes"
)

#
# ------------ Testing --------------
#

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()
endif()

#
# ------------ Packaging --------------
#

install(
  DIRECTORY "launch"
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  TARGETS
  ${PROJECT_NAME}_example_nodes
  ${PROJECT_NAME}_example_build_handlers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
