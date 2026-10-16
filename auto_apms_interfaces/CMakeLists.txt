cmake_minimum_required(VERSION 3.8)
project(auto_apms_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(TEST_SOURCES
  "action/TestActionWrapper.action"
)

set(EXAMPLE_SOURCES
  "action/ExampleSimpleSkill.action"
)

set(MESSAGE_SOURCES
)
set(SERVICE_SOURCES
  "srv/SetBehavior.srv"
)
set(ACTION_SOURCES
  "action/StartTreeExecutor.action"
  "action/CommandTreeExecutor.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${TEST_SOURCES} ${EXAMPLE_SOURCES} ${ACTION_SOURCES} ${SERVICE_SOURCES} ${MESSAGE_SOURCES}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()
endif()

ament_package()
