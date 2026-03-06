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

#include <utility>
#include <vector>

#include "auto_apms_behavior_tree_core/behavior.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_mission
{

struct MissionConfigResourceIdentity : public auto_apms_behavior_tree::core::BehaviorResourceIdentity
{
  /**
   * @brief Constructor of a mission config resource identity object.
   *
   * @p identity must be formatted like `<package_name>::<config_file_stem>`.
   * @param identity Identity string for a specific mission config resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  MissionConfigResourceIdentity(const std::string & identity);

  /**
   * @brief Constructor of a mission config resource identity object.
   *
   * @p identity must be formatted like `<package_name>::<config_file_stem>`.
   * @param identity C-style identity string for a specific mission config resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  MissionConfigResourceIdentity(const char * identity);

  /**
   * @brief Constructor of an empty mission config resource identity object.
   *
   * The user must manually populate the member fields.
   */
  MissionConfigResourceIdentity() = default;
};

/**
 * @ingroup auto_apms_mission
 * @brief Configuration parameters for generic missions supported by AutoAPMS.
 */
struct MissionConfig
{
  using TreeResourceIdentity = auto_apms_behavior_tree::core::TreeResourceIdentity;

  static const std::string YAML_KEY_BRINGUP;
  static const std::string YAML_KEY_MISSION;
  static const std::string YAML_KEY_CONTINGENCY;
  static const std::string YAML_KEY_EMERGENCY;
  static const std::string YAML_KEY_SHUTDOWN;

  MissionConfig() = default;

  AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS(MissionConfig)

  /**
   * @brief Create a mission configuration from an installed resource.
   *
   * The resource identity must be specified in the format `<package_name>::<config_file_stem>` or simply
   * `<config_file_stem>`.
   * @param identity Identity of the  mission configuration resource.
   * @return Object created from the corresponding resource.
   * @throw auto_apms_util::exceptions::ResourceIdentityFormatError if @p identity has wrong format.
   * @throw auto_apms_util::exceptions::ResourceError if resource cannot be determined using @p identity.
   */
  static MissionConfig fromResource(const MissionConfigResourceIdentity & identity);

  std::vector<TreeResourceIdentity> bringup;
  std::vector<TreeResourceIdentity> mission;
  std::vector<std::pair<TreeResourceIdentity, TreeResourceIdentity>> contingency;
  std::vector<std::pair<TreeResourceIdentity, TreeResourceIdentity>> emergency;
  std::vector<TreeResourceIdentity> shutdown;
};

class MissionConfigResource
: public auto_apms_behavior_tree::core::BehaviorResourceTemplate<MissionConfigResourceIdentity>
{
  friend class MissionConfig;

public:
  using BehaviorResourceTemplate::BehaviorResourceTemplate;
};

}  // namespace auto_apms_mission

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond INTERNAL
namespace YAML
{
template <>
struct convert<auto_apms_mission::MissionConfig>
{
  using Config = auto_apms_mission::MissionConfig;
  inline static Node encode(const Config & rhs)
  {
    Node node(NodeType::Map);
    node[Config::YAML_KEY_BRINGUP] = rhs.bringup;
    node[Config::YAML_KEY_MISSION] = rhs.mission;
    Node contingency_node = node[Config::YAML_KEY_CONTINGENCY];
    for (const auto & [key, val] : rhs.contingency) {
      contingency_node[key.str()] = val;
    }
    Node emergency_node = node[Config::YAML_KEY_EMERGENCY];
    for (const auto & [key, val] : rhs.emergency) {
      emergency_node[key.str()] = val;
    }
    node[Config::YAML_KEY_SHUTDOWN] = rhs.shutdown;
    return node;
  }
  inline static bool decode(const Node & node, Config & rhs)
  {
    if (!node.IsMap())
      throw auto_apms_util::exceptions::YAMLFormatError(
        "YAML::Node for auto_apms_mission::MissionConfig must be map but is type " + std::to_string(node.Type()) +
        " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

    for (auto it = node.begin(); it != node.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      const Node val = it->second;

      if (key == Config::YAML_KEY_BRINGUP) {
        if (val.IsNull()) continue;
        if (!val.IsSequence()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
            " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        rhs.bringup = val.as<std::vector<Config::TreeResourceIdentity>>();
        continue;
      }

      if (key == Config::YAML_KEY_MISSION) {
        if (!val.IsSequence()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
            " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        rhs.mission = val.as<std::vector<Config::TreeResourceIdentity>>();
        continue;
      }

      if (key == Config::YAML_KEY_CONTINGENCY) {
        if (val.IsNull()) continue;
        if (!val.IsMap()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
            " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        for (YAML::const_iterator it2 = val.begin(); it2 != val.end(); ++it2) {
          const std::string monitor_id = it2->first.as<std::string>();
          if (!it2->second.IsScalar()) {
            throw auto_apms_util::exceptions::YAMLFormatError(
              "Value for key '" + monitor_id + "' in the CONTINGENCY group must be scalar but is type " +
              std::to_string(val.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
          }
          rhs.contingency.push_back(
            {Config::TreeResourceIdentity(monitor_id), it2->second.as<Config::TreeResourceIdentity>()});
        }
        continue;
      }

      if (key == Config::YAML_KEY_EMERGENCY) {
        if (val.IsNull()) continue;
        if (!val.IsMap()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
            " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        for (YAML::const_iterator it2 = val.begin(); it2 != val.end(); ++it2) {
          const std::string monitor_id = it2->first.as<std::string>();
          if (!it2->second.IsScalar()) {
            throw auto_apms_util::exceptions::YAMLFormatError(
              "Value for key '" + monitor_id + "' in the EMERGENCY group must be scalar but is type " +
              std::to_string(val.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
          }
          rhs.emergency.push_back(
            {Config::TreeResourceIdentity(monitor_id), it2->second.as<Config::TreeResourceIdentity>()});
        }
        continue;
      }

      if (key == Config::YAML_KEY_SHUTDOWN) {
        if (val.IsNull()) continue;
        if (!val.IsSequence()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
            " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        rhs.shutdown = val.as<std::vector<Config::TreeResourceIdentity>>();
        continue;
      }

      // Unknown parameter
      throw auto_apms_util::exceptions::YAMLFormatError("Unknown parameter name '" + key + "'.");
    }
    return true;
  }
};
}  // namespace YAML
/// @endcond
