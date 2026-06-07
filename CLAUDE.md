# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

AutoAPMS (Automated Action Planning and Management System) is a ROS 2 framework for building
behavior-based robotics applications on top of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).
The repo is a multi-package colcon workspace (mostly C++, with a supplementary Python API). It targets ROS 2
Jazzy, Kilted, and Rolling.

## Build, Test, Lint

This repo is a set of ROS 2 packages — it is meant to be built from a colcon workspace, not standalone.
Commands assume you are at the workspace root (one level above `src/`) with ROS 2 sourced.

```bash
# Build everything needed for the examples (default for development)
colcon build --packages-up-to auto_apms_examples --symlink-install

# Build a single package and its dependencies
colcon build --packages-up-to auto_apms_behavior_tree --symlink-install

# Rebuild only one package (after deps are built)
colcon build --packages-select auto_apms_behavior_tree_core --symlink-install

# Always re-source after building so ament_index resources are visible
source install/setup.bash
```

Testing (gtest for C++, pytest for Python, via `ament`):

```bash
colcon test --packages-select auto_apms_behavior_tree_core
colcon test-result --verbose          # show detailed failures

# Run a single gtest binary directly after building (faster iteration)
./build/auto_apms_behavior_tree_core/<test_target_name> --gtest_filter='SuiteName.CaseName'
```

Linting is enforced via pre-commit (see `.pre-commit-config.yaml`), not invoked by the build:

```bash
pre-commit run -a            # run all hooks on all files
pre-commit install           # auto-run on every git commit
```

Key style rules: clang-format (`.clang-format`) for C++/C, black with **line-length 120** for Python,
cmakelint (line-length and wonkycase filters disabled, see `.cmakelintrc`).

Generate API docs (requires Doxygen >= 1.10): `doxygen doc/Doxyfile` from the repo root.

## Package Layout & Dependency Order

Packages build bottom-up in this dependency order:

1. **`auto_apms_interfaces`** (rosidl) — ROS 2 action/message definitions used across the framework.
2. **`auto_apms_util`** (ament_cmake + Python) — foundational utilities with no BT knowledge:
   `PluginClassLoader` (pluginlib wrapper with ambiguity checks), ament_index resource collection,
   action client/server wrappers, YAML conversion, string/filesystem/logging helpers.
3. **`auto_apms_behavior_tree_core`** (ament_cmake + Python) — the core BT API and the CMake tooling that
   everything else relies on. Contains the `TreeDocument` builder API, the node registration/manifest system,
   ROS node base classes (`RosActionNode`, `RosServiceNode`, `RosPublisherNode`, `RosSubscriberNode`), and the
   `ros2 run`-style CLI tools that generate node metadata/models.
4. **`auto_apms_behavior_tree`** (ament_cmake + Python) — standard library: built-in BT nodes
   (`src/node/*.cpp`), the behavior **executor node**, and the **build handler** plugin system.
5. **`auto_apms_ros2behavior`** (ament_python) — the `ros2 behavior` CLI extension (pure Python).
6. **`auto_apms_mission`** (ament_cmake) — mission management layer: mission build handlers and executor
   components built on the behavior tree layer.
7. **`auto_apms_examples`** — runnable examples and launch files.

## Core Architecture

The whole framework is built around a **plugin + ament_index resource** pattern. C++ classes (BT nodes, build
handlers) are registered as pluginlib plugins; CMake macros additionally write framework-specific resource
metadata into the ament index at build time; runtime loaders discover and instantiate them by **fully qualified
class name** (`namespace::ClassName`). When adding a new plugin type, you generally touch three places: the C++
registration macro call, a CMake `..._register_...` macro in `CMakeLists.txt`, and the runtime loader.

Four concepts are central and interconnected:

- **Behavior tree nodes** — Custom nodes subclass the ROS node bases in `auto_apms_behavior_tree_core/node/`.
  They are registered via the CMake macro `auto_apms_behavior_tree_register_nodes(<library_target> <Class>...)`
  (see `auto_apms_behavior_tree_core/cmake/register_nodes.cmake`). Registration normally wraps each class in
  `NodeRegistrationTemplate<>`, but `NODE_REGISTRATION_TYPE` allows a custom `NodeRegistrationInterface`.

- **Node manifests** — YAML files describing *which* nodes a tree needs and *how* to load each
  (`NodeRegistrationOptions`). Manifests support a `parent` field so entries can extend a parent entry. They are
  the bridge that lets you assemble trees from nodes across packages without writing registration code. Manifest
  entries can be installed as resources and referenced later by identity.

- **Node model headers** — Code-generated C++ headers (driven by the `cli/create_node_model*.cpp` tools and the
  `generate_node_metadata*.cmake` macros) that produce a typed model class per node listed in a manifest. These
  give the `TreeDocument` builder API compile-time-checked node insertion. Generate them via the
  `NODE_MODEL_HEADER_TARGET` argument of `auto_apms_behavior_tree_register_nodes`.

- **Behaviors & build handlers** — A *behavior definition* (a.k.a. build request) is a blueprint registered as a
  resource via `auto_apms_behavior_tree_register_behavior(...)` (`cmake/register_behavior.cmake`). At runtime a
  **`TreeBuildHandler`** plugin (`auto_apms_behavior_tree/build_handler/`) turns a build-request string + entry
  point + node manifest into an actual tree. The executor node selects the handler by class name (its
  `build_handler` ROS parameter), so the *same* executor can build trees from XML resources, raw strings, or
  custom algorithms just by swapping the handler. Register handlers with
  `auto_apms_behavior_tree_register_build_handlers(...)` (`cmake/register_build_handlers.cmake`). The mission
  layer adds its own build handlers in `auto_apms_mission/src/build_handler/`.

The **executor** (`auto_apms_behavior_tree/executor/`) wraps tree execution behind a ROS 2 action interface
(`StartTreeExecutor`). `TreeExecutorNode` / `GenericExecutorNode` are also exposed as `rclcpp_components` so they
can be composed into a container.

The CLI flow: `ros2 behavior <verb>` (Python, `auto_apms_ros2behavior/verb/`) dispatches to verbs
(`run`, `show`, `send`, `list`, and the `node` subgroup `plugins`/`manifest`/`model`/`call`). These are thin
Python wrappers that discover registered behaviors/nodes via the resource index and drive the C++ executor/tools.

## Conventions

- Plugin classes are always referenced by fully qualified name (`namespace::ClassName`); template plugins use
  `MyClass<T>` syntax in registration strings.
- All packages share one synchronized version (`package.xml`). Apache-2.0 licensed; source files carry the
  standard Apache header (copyright "Robin Müller").
- When you add or change a registered node/handler/behavior, the corresponding resource metadata is produced at
  **build time** — you must rebuild and re-source before the runtime loaders and CLI can see it.
