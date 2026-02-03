^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package auto_apms_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.2 (2026-02-03)
------------------

1.4.1 (2026-02-03)
------------------

1.4.0 (2026-01-30)
------------------
* Fix version number to match last release
* Add ament_cmake_copyright to all packages
* Move auto_apms_simulation to seperate repo and update package manifests for ros index
* Change URLs after move to organization
* Contributors: Robin Müller

1.3.0 (2025-09-21)
------------------
* Rename connection to topic since it's the ros nomenclature
* Contributors: Robin Müller

1.2.0 (2025-08-17)
------------------
* Rename declare\_* to register\_* for clarity and consistency
* Update links to docs
* Add RunBehavior launch action
* Remove simulation launch file from examples because it was moved
* Refactor
* Remove simulation launch file from examples because it was moved
* Restructure examples
* Add node sub verb
* Small changes
* Apply changes to auto_apms_mission
* A lot of refactorings
* Contributors: Robin Müller

1.1.0 (2025-05-07)
------------------
* Add warning if auto_apms_px4 is not found when examples are built
* Replace deprecated launch substitutions
* Remove smiley from GoTo docs
* Make GoTo node more general
* Contributors: Robin Müller

1.0.0 (2025-02-02)
------------------
* Update docs
* Update docs
* Add simple skill example
* Improve cmake configuration when building examples
* Improvements for enabling hogwarts demo
* Update readme
* Add pyrobosim hogwarts mission
* Optimize run_tree for pyrobosim
* Add hogwarts simulation
* Add lightweight package for simulation using pyrobosim
* Implement mission launch using multiple nodes
* Implement mission builder and verify functionality
* Fix mission builder. Currently without events
* Commit before TreeDocument and TreeBuilder refactor
* Add code generation for declared nodes using CMake
* Improve TreeBuilder and refactor ROS behavior tree nodes
* Seperate TreeBuilder and TreeDocument API
* Improve mission builder
* Add mission framework
* Improve cmake macros for metadata generation
* Fix bug when creating node models (Ambiguity check fails)
* Fix tinyxml2
* Fix build by using behaviortree_cpp package from ros index
* Add auto_apms_behavior_tree_core
* Renaming and bug fix
* Add dynamic blackboard parameter support to tree executor server
* Add possibility to specify port values of nodes using TreeBuilder
* New clang format style
* Add run_tree_node executable
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
* Improve api docs
* Seperate to individual packages
* Clean up cmake macros
* Clean up behavior tree api
* Integrate ros params interface to node plugin loader
* Change package.xml details
* Redesign resource system
* Contributors: Robin Müller
