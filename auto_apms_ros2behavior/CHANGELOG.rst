^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package auto_apms_ros2behavior
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Improve ros2behavior cli by adding dynamic help messages
* Change URLs after move to organization
* Move python scripting tools to auto_apms_behavior_tree package
* Contributors: Robin Müller

1.3.0 (2025-09-21)
------------------
* Add support for port_defaults and description fields in node manifest
* Allow empty behavior identities with ros2 behavior run
* Contributors: Robin Müller

1.2.0 (2025-08-17)
------------------
* Refactor for multi distro build
* Fix linting
* Refactor
* Restructure examples
* Add node model parsing
* Add node sub verb
* Contributors: Robin Müller

1.1.0 (2025-05-07)
------------------

1.0.0 (2025-02-02)
------------------
