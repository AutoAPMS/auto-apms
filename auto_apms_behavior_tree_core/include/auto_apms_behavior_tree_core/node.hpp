// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "auto_apms_behavior_tree_core/node/node_registration_template.hpp"
#include "auto_apms_behavior_tree_core/node/ros_action_node.hpp"
#include "auto_apms_behavior_tree_core/node/ros_publisher_node.hpp"
#include "auto_apms_behavior_tree_core/node/ros_service_node.hpp"
#include "auto_apms_behavior_tree_core/node/ros_subscriber_node.hpp"
#include "pluginlib/class_list_macros.hpp"

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Macro for registering a behavior tree node plugin.
 *
 * Behavior tree nodes should be created by inheriting from one of
 *
 * - auto_apms_behavior_tree::core::RosActionNode
 *
 * - auto_apms_behavior_tree::core::RosServiceNode
 *
 * - auto_apms_behavior_tree::core::RosPublisherNode
 *
 * - auto_apms_behavior_tree::core::RosSubscriberNode
 *
 * One may also inherit from one of the other base classes provided by BehaviorTree.CPP if the functionality to be
 * implemented doesn't require ROS 2 interfaces.
 *
 * @param type Fully qualified name of the class.
 */
#define AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(type)                \
  PLUGINLIB_EXPORT_CLASS(                                          \
    auto_apms_behavior_tree::core::NodeRegistrationTemplate<type>, \
    auto_apms_behavior_tree::core::NodeRegistrationInterface)
