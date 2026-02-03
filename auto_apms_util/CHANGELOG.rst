^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package auto_apms_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.4.1 (2026-02-03)
------------------

1.4.0 (2026-01-30)
------------------
* Fix version number to match last release
* Add ament_cmake_copyright to all packages
* Stick to SPDX identifier for license tag
* Change URLs after move to organization
* Contributors: Robin Müller

1.3.0 (2025-09-21)
------------------
* Add native node model xml file
* Contributors: Robin Müller

1.2.0 (2025-08-17)
------------------
* Rename declare\_* to register\_* for clarity and consistency
* Remove boost dependency
* Refactor
* Add node sub verb
* Apply changes to auto_apms_mission
* A lot of refactorings
* Add behavior resource concept
* Add python api for more resources
* Contributors: Robin Müller

1.1.0 (2025-05-07)
------------------
* Change API docs main page
* Don't update ros params after tick if new_parameters empty
* Add boost dep to auto_apms_util
* Contributors: Robin Müller

1.0.0 (2025-02-02)
------------------
* Update docs
* Update docs
* Add simple skill example
* Update readme
* Improve cmake configuration when building examples
* Extend API docs
* Add docs for tree document API
* Improvements for enabling hogwarts demo
* Add more API docs
* Add lightweight package for simulation using pyrobosim
* Implement mission launch using multiple nodes
* Implement mission builder and verify functionality
* Fix mission builder. Currently without events
* Test set and get parameter nodes
* Improve TreeBuilder and refactor ROS behavior tree nodes
* Seperate TreeBuilder and TreeDocument API
* Improve mission builder
* Add mission framework
* Add mission orchestrator
* Improve cmake macros for metadata generation
* Fix bug when creating node models (Ambiguity check fails)
* Add auto_apms_behavior_tree_core
* Renaming and bug fix
* Add dynamic blackboard parameter support to tree executor server
* Add possibility to specify port values of nodes using TreeBuilder
* New clang format style
* Add run_tree_node executable
* Remove unnecessary parameters from node manifest and add ambuiguity check for node registration loader
* Rename auto_apms_core to auto_apms_util
* Contributors: Robin Müller
