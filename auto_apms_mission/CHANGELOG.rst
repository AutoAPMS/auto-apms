^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package auto_apms_mission
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.1 (2026-02-03)
------------------

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
* Add ament_cmake_copyright to all packages
* Move auto_apms_simulation to seperate repo and update package manifests for ros index
* Change URLs after move to organization
* Contributors: Robin Müller

1.3.0 (2025-09-21)
------------------

1.2.0 (2025-08-17)
------------------
* Rename declare\_* to register\_* for clarity and consistency
* Refactor
* Apply changes to auto_apms_mission
* Add entrypoint
* A lot of refactorings
* Contributors: Robin Müller

1.1.0 (2025-05-07)
------------------

1.0.0 (2025-02-02)
------------------
* Update docs
* Add simple skill example
* Update readme
* Improve cmake configuration when building examples
* Extend API docs
* Add docs for tree document API
* Fix typo
* Introduce individual blackboards to event monitor and handler subtrees
* Improvements for enabling hogwarts demo
* Fix pre commit errors
* Add pyrobosim hogwarts mission
* Optimize run_tree for pyrobosim
* Add lightweight package for simulation using pyrobosim
* Implement mission launch using multiple nodes
* Implement mission builder and verify functionality
* Fix mission builder. Currently without events
* Commit before TreeDocument and TreeBuilder refactor
* Add dynamic type conversion for node models
* Add code generation for declared nodes using CMake
* Add more insertTree overloads
* Seperate TreeBuilder and TreeDocument API
* Improve mission builder
* Add mission framework
* Add mission orchestrator
* Fix build by using behaviortree_cpp package from ros index
* Improve api docs
* Contributors: Robin Müller
