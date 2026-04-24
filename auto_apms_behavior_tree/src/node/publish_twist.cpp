// Copyright 2026 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_behavior_tree_core/node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#define INPUT_KEY_LINEAR_X "linear_x"
#define INPUT_KEY_LINEAR_Y "linear_y"
#define INPUT_KEY_LINEAR_Z "linear_z"
#define INPUT_KEY_ANGULAR_X "angular_x"
#define INPUT_KEY_ANGULAR_Y "angular_y"
#define INPUT_KEY_ANGULAR_Z "angular_z"
#define INPUT_KEY_FRAME_ID "frame_id"

namespace auto_apms_behavior_tree
{

class PublishTwist : public core::RosPublisherNode<geometry_msgs::msg::TwistStamped>
{
public:
  PublishTwist(const std::string & instance_name, const Config & config, const Context & context)
  : RosPublisherNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>(INPUT_KEY_LINEAR_X, 0.0, "Linear velocity along x-axis in m/s."),
      BT::InputPort<double>(INPUT_KEY_LINEAR_Y, 0.0, "Linear velocity along y-axis in m/s."),
      BT::InputPort<double>(INPUT_KEY_LINEAR_Z, 0.0, "Linear velocity along z-axis in m/s."),
      BT::InputPort<double>(INPUT_KEY_ANGULAR_X, 0.0, "Angular velocity around x-axis in rad/s."),
      BT::InputPort<double>(INPUT_KEY_ANGULAR_Y, 0.0, "Angular velocity around y-axis in rad/s."),
      BT::InputPort<double>(INPUT_KEY_ANGULAR_Z, 0.0, "Angular velocity around z-axis in rad/s."),
      BT::InputPort<std::string>(INPUT_KEY_FRAME_ID, "map", "Frame ID for the twist."),
    });
  }

  bool setMessage(geometry_msgs::msg::TwistStamped & msg) override final
  {
    const BT::Expected<double> expected_linear_x = getInput<double>(INPUT_KEY_LINEAR_X);
    const BT::Expected<double> expected_linear_y = getInput<double>(INPUT_KEY_LINEAR_Y);
    const BT::Expected<double> expected_linear_z = getInput<double>(INPUT_KEY_LINEAR_Z);
    const BT::Expected<double> expected_angular_x = getInput<double>(INPUT_KEY_ANGULAR_X);
    const BT::Expected<double> expected_angular_y = getInput<double>(INPUT_KEY_ANGULAR_Y);
    const BT::Expected<double> expected_angular_z = getInput<double>(INPUT_KEY_ANGULAR_Z);
    const BT::Expected<std::string> expected_frame_id = getInput<std::string>(INPUT_KEY_FRAME_ID);

    if (
      !expected_linear_x || !expected_linear_y || !expected_linear_z || !expected_angular_x || !expected_angular_y ||
      !expected_angular_z || !expected_frame_id) {
      RCLCPP_ERROR(logger_, "%s - Failed to get input values", context_.getFullyQualifiedTreeNodeName(this).c_str());
      return false;
    }

    msg.twist.linear.x = expected_linear_x.value();
    msg.twist.linear.y = expected_linear_y.value();
    msg.twist.linear.z = expected_linear_z.value();
    msg.twist.angular.x = expected_angular_x.value();
    msg.twist.angular.y = expected_angular_y.value();
    msg.twist.angular.z = expected_angular_z.value();

    msg.header.frame_id = expected_frame_id.value();
    msg.header.stamp = context_.getCurrentTime();

    RCLCPP_DEBUG(
      logger_, "%s - Publishing twist: linear=[%.2f, %.2f, %.2f m/s] angular=[%.2f, %.2f, %.2f rad/s]",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
      msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);

    return true;
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::PublishTwist)
