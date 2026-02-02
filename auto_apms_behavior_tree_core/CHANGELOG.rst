^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package auto_apms_behavior_tree_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix issue that leads to segfaults when loading behavior tree node plugins from libraries that have been built using link-time optimization (LTO) as in the ROS 2 build farm
* Contributors: Robin Müller

1.4.0 (2026-01-30)
------------------
* Feat: Enable to use the CLI to define behaviors dynamically (`#14 <https://github.com/AutoAPMS/auto-apms/issues/14>`_)
  * Add build_request, entrypoint and node_manifest keyword args to run and send cli
  * Rename entrypoint to entry_point
  * Use hyphen instead of underscore for entry point in argparse arguments
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Rename forgotten entry point occurences
  * Update auto_apms_ros2behavior/auto_apms_ros2behavior/verb/send.py
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Update auto_apms_behavior_tree/auto_apms_behavior_tree/scripting.py
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* Fix version number to match last release
* Fix: Avoid silently skipping registering nodes when another one with the same name was previously registered (`#12 <https://github.com/AutoAPMS/auto-apms/issues/12>`_)
  * Remove insufficient check for node equality
  * Update documentation for clarifying the decision of being more strict with duplicate node names
  * Add applyNodeNamespace methods and extend tests for the tree builder API
  * Fix typo
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Add tests for registering nodes using existing plugins
  * fix docs for newTreeFromResource
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Also apply namespace to bt factory registrations
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* Add ament_cmake_copyright to all packages
* Add ALIAS_NAMESPACE cmake macro argument to auto_apms_behavior_tree_register_trees to allow customizing the tree namespace and reusing the same xml file under different names and possibly different node manifests
* Change default integer type from int64_t to standard int for better compatibility
* Move auto_apms_simulation to seperate repo and update package manifests for ros index
* feat: support include tag with tree documents (`#11 <https://github.com/AutoAPMS/auto-apms/issues/11>`_)
  * feat: support include tag with tree documents
  * Use filesystem path to make ros_pkg attribute more robust
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Fix typo
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Fix linting
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* Fix linting
* Fix typo
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* Use filesystem path to make ros_pkg attribute more robust
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* feat: support include tag with tree documents
* Add markdown cli lint exception MD060 for auto generated reference files
* Change URLs after move to organization
* Support calling cmake macro auto_apms_behavior_tree_register_nodes without any node class names (keyword only)
* fix: Make RosPublisherNode waiting for at least one subscriber on init
* Add an error when port aliasing is used when the nodes constructor does not support it
* [NodeManifest] Add port aliasing feature (`#10 <https://github.com/AutoAPMS/auto-apms/issues/10>`_)
  * Add port aliasing feature
  * Update auto_apms_behavior_tree_core/src/node/ros_node_context.cpp
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Fix build error
  * Apply suggestion from @Copilot
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* [NodeManifest] Allow hiding node ports (`#9 <https://github.com/AutoAPMS/auto-apms/issues/9>`_)
  * Add method to write node model from NodeModelMap
  * Add registration option hidden_ports and TreeDocument implementation to hide port
  * Introduce individual job names in CI for each ROS 2 distro
  NOTE: Rolling build is currently broken since there is an issue with BT.CPP 4.8 and the tinyxml2 version on Ubuntu 24 systems
* Contributors: Robin Müller

1.3.0 (2025-09-21)
------------------
* Add a method for accessing the extra registration options
* Include description in node reference for website
* Rename connection to topic since it's the ros nomenclature
* Add support for port_defaults and description fields in node manifest
* Allow empty behavior identities with ros2 behavior run
* Add native node model xml file
* Contributors: Robin Müller

1.2.0 (2025-08-17)
------------------
* Remove overview heading from generated bt node reference
* Rename declare\_* to register\_* for clarity and consistency
* Update links to docs
* Refactor for multi distro build
* Add test for RosSubscriberNode
* Remove boost dependency
* Add RunBehavior launch action
* Fix node reference generation
* Fix node manifest resource
* Refactor
* Restructure examples
* Add node model parsing
* Add node sub verb
* Apply changes to auto_apms_mission
* Update ros2cli verbs
* Add entrypoint
* A lot of refactorings
* Add python translation
* Add BehaviorResourceTemplate
* Start working on BehaviorResource
* Add behavior resource concept
* Add python api for more resources
* Add auto_apms_ros2cli package
* Add std::vector<float> conversion from string for behavior trees
* Add extra field to node registration options for customizing node implementations using additional parameters
* Explicitly set the build handler in run_tree executable
* Properly support behavior trees defined in files with the same name
* Make sure declare_trees.cmake is rerun when xml files are changed
* Support inheriting logging severity from parent logger in individual behavior tree nodes
* Add python helpers for tree resource identification
* Contributors: Robin Müller

1.1.0 (2025-05-07)
------------------
* Add auto update behavior tree node reference docs workflow
* Make GoTo node more general
* Update pre commits
* Don't update ros params after tick if new_parameters empty
* Fix link to standard nodes
* Contributors: Robin Müller

1.0.0 (2025-02-02)
------------------
* Update docs
* Update docs
* Add simple skill example
* Improve cmake configuration when building examples
* Extend API docs
* Add docs for tree document API
* Add more API docs
* Introduce individual blackboards to event monitor and handler subtrees
* Improvements for enabling hogwarts demo
* Update readme
* Add pyrobosim hogwarts mission
* Add hogwarts simulation
* Add lightweight package for simulation using pyrobosim
* Implement mission launch using multiple nodes
* Implement mission builder and verify functionality
* Fix mission builder. Currently without events
* Commit before TreeDocument and TreeBuilder refactor
* Add dynamic type conversion for node models
* Add code generation for declared nodes using CMake
* Test set and get parameter nodes
* Improve TreeBuilder and refactor ROS behavior tree nodes
* Add more insertTree overloads
* Seperate TreeBuilder and TreeDocument API
* Improve mission builder
* Add mission framework
* Add mission orchestrator
* Improve cmake macros for metadata generation
* Fix bug when creating node models (Ambiguity check fails)
* Fix tinyxml2
* Fix build by using behaviortree_cpp package from ros index
* Add auto_apms_behavior_tree_core
* Contributors: Robin Müller
