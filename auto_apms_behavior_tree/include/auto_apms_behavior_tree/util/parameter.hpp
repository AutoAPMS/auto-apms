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

#include "auto_apms_behavior_tree_core/convert.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Convert a ROS 2 parameter value to a `BT::Any` object.
 *
 * Since the type information of a parameter value is stored internally, it's possible to create the corresponding
 * `BT::Any` object dynamically.
 * @param val ROS 2 parameter value.
 * @return Expected object for `BT::Any`. Holds an error message if the type of @p val couldn't be
 * converted to a C++ type.
 */
BT::Expected<BT::Any> createAnyFromParameterValue(const rclcpp::ParameterValue & val);

/**
 * @brief Convert a `BT::Any` object to a ROS 2 parameter value.
 *
 * You may optionally specify the type of the parameter value to convert to using @p type. If you want to deduce the
 * type from @p any, set this to `rclcpp::PARAMETER_NOT_SET`.
 * @param any `BT::Any` object.
 * @param type Desired type of the parameter value. Set to `rclcpp::PARAMETER_NOT_SET` if it is unknown (e.g. if the
 * parameter hasn't been set before).
 * @return Expected object for `rclcpp::ParameterValue`. Holds an error message if @p any couldn't be converted to @p
 * type.
 */
BT::Expected<rclcpp::ParameterValue> createParameterValueFromAny(const BT::Any & any, rclcpp::ParameterType type);

}  // namespace auto_apms_behavior_tree
