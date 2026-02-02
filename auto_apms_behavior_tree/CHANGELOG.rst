^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package auto_apms_behavior_tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Change default integer type from int64_t to standard int for better compatibility
* Move auto_apms_simulation to seperate repo and update package manifests for ros index
* Change URLs after move to organization
* Move python scripting tools to auto_apms_behavior_tree package
* fix: Make RosPublisherNode waiting for at least one subscriber on init
* node: add PublishPose node
* [NodeManifest] Allow hiding node ports (`#9 <https://github.com/AutoAPMS/auto-apms/issues/9>`_)
  * Add method to write node model from NodeModelMap
  * Add registration option hidden_ports and TreeDocument implementation to hide port
  * Introduce individual job names in CI for each ROS 2 distro
  NOTE: Rolling build is currently broken since there is an issue with BT.CPP 4.8 and the tinyxml2 version on Ubuntu 24 systems
* Fix linting
* Add std_srvs nodes
* Contributors: Robin Müller

1.3.0 (2025-09-21)
------------------
* Rename connection to topic since it's the ros nomenclature
* Add support for port_defaults and description fields in node manifest
* Add native node model xml file
* Contributors: Robin Müller

1.2.0 (2025-08-17)
------------------
* Rename declare\_* to register\_* for clarity and consistency
* Update links to docs
* Refactor for multi distro build
* Add test for RosSubscriberNode
* Remove boost dependency
* Add RunBehavior launch action
* Fix linting
* Fix linting
* Refactor
* Add node sub verb
* Apply changes to auto_apms_mission
* Update ros2cli verbs
* Add entrypoint
* A lot of refactorings
* Add python api for more resources
* Add extra field to node registration options for customizing node implementations using additional parameters
* Explicitly set the build handler in run_tree executable
* Add std_srvs to export deps of auto_apms_behavior_tree
* Add clear_blackboard service for tree executor
* Support inheriting logging severity from parent logger in individual behavior tree nodes
* Contributors: Robin Müller

1.1.0 (2025-05-07)
------------------
* Update pre commits
* Don't update ros params after tick if new_parameters empty
* Add python dependencies to package.xml of auto_apms_simulation
* Contributors: Robin Müller

1.0.0 (2025-02-02)
------------------
* Update docs
* Add simple skill example
* Improve cmake configuration when building examples
* Extend API docs
* Add docs for tree document API
* Improvements for enabling hogwarts demo
* Update readme
* Optimize run_tree for pyrobosim
* Add lightweight package for simulation using pyrobosim
* Implement mission launch using multiple nodes
* Implement mission builder and verify functionality
* Fix mission builder. Currently without events
* Commit before TreeDocument and TreeBuilder refactor
* Add code generation for declared nodes using CMake
* Test set and get parameter nodes
* Improve TreeBuilder and refactor ROS behavior tree nodes
* Seperate TreeBuilder and TreeDocument API
* Improve mission builder
* Add mission framework
* Add mission orchestrator
* Improve cmake macros for metadata generation
* Fix bug when creating node models (Ambiguity check fails)
* Fix tinyxml2
* Fix build by using behaviortree_cpp package from ros index
* Add auto_apms_behavior_tree_core
* Renaming and bug fix
* Support type conversion for blackboard and enum parameters
* Add dynamic blackboard parameter support to tree executor server
* Add possibility to specify port values of nodes using TreeBuilder
* New clang format style
* Add run_tree_node executable
* Remove unnecessary parameters from node manifest and add ambuiguity check for node registration loader
* Rename auto_apms_core to auto_apms_util
* Polish tree executor
* Add node overrides
* Refactor node base classes
* Test tree executor
* Mostly renaming
* Add build directors
* Implement executor in run_behavior_tree
* Extend BTCreator and BTExecutorBase
* Executor base
* Introduce generic executable for running behavior trees
* Use own node base implementation
* Fix docs and ci
* Improve api docs
* Seperate to individual packages
* Contributors: Robin Müller
