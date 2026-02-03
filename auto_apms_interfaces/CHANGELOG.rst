^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package auto_apms_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.4.1 (2026-02-03)
------------------

1.4.0 (2026-01-30)
------------------
* Change entrypoint field of StartTreeExecutor to entry_point (could be a breaking change)

1.3.0 (2025-09-21)
------------------

1.2.0 (2025-08-17)
------------------
* Add test for RosSubscriberNode
* Remove boost dependency
* Refactor
* Update ros2cli verbs
* Add entrypoint
* Add std::vector<float> conversion from string for behavior trees
* Contributors: Robin Müller

1.1.0 (2025-05-07)
------------------
* Make GoTo node more general
* find_package(rosidl_default_generators) before geometry_msgs
* Contributors: Robin Müller

1.0.0 (2025-02-02)
------------------
* Add simple skill example
* Improve cmake configuration when building examples
* Commit before TreeDocument and TreeBuilder refactor
* Add mission framework
* Add mission orchestrator
* Renaming and bug fix
* Support type conversion for blackboard and enum parameters
* Add possibility to specify port values of nodes using TreeBuilder
* Rename auto_apms_core to auto_apms_util
* Add node overrides
* Test tree executor
* Add build directors
* Seperate to individual packages
* Change package.xml details
* Redesign resource system
* Contributors: Robin Müller
