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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "auto_apms_behavior_tree/executor/generic_executor_node.hpp"
#include "auto_apms_behavior_tree/executor/options.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_tree;

namespace
{

// Name of the additional parameter a subclass declares on top of the executor's own parameter set.
constexpr const char * kExtraParamName = "my_extra_param";

// Minimal subclass mirroring how BehaviorModeExecutorNode extends the executor: it declares a node-specific parameter
// in its constructor, i.e. after the base GenericTreeExecutorNode has installed its on-set-parameters callback.
class ExtraParamExecutorNode : public GenericTreeExecutorNode
{
public:
  explicit ExtraParamExecutorNode(const Options & options)
  : GenericTreeExecutorNode("test_extra_param_executor", options)
  {
    getNodePtr()->declare_parameter(kExtraParamName, false);
  }
};

// Build executor options that don't require any build handler plugin to be discoverable at runtime.
GenericTreeExecutorNode::Options makeOptions(bool strict_unknown_parameter_removal)
{
  rclcpp::NodeOptions ros_options;
  // Selecting the "no build handler" sentinel keeps construction free of plugin discovery.
  ros_options.append_parameter_override("build_handler", GenericTreeExecutorNode::PARAM_VALUE_NO_BUILD_HANDLER);
  return GenericTreeExecutorNode::Options(ros_options)
    .enableStrictUnkownParameterRemoval(strict_unknown_parameter_removal);
}

class ExecutorNodeParametersTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
};

}  // namespace

TEST_F(ExecutorNodeParametersTest, SubclassMayDeclareExtraParameterWhenStrictRemovalDisabled)
{
  // Constructing the subclass must not throw: declaring the extra parameter must pass the executor's on-set callback.
  std::shared_ptr<ExtraParamExecutorNode> node;
  ASSERT_NO_THROW(node = std::make_shared<ExtraParamExecutorNode>(makeOptions(false)));

  ASSERT_TRUE(node->getNodePtr()->has_parameter(kExtraParamName));
  EXPECT_FALSE(node->getNodePtr()->get_parameter(kExtraParamName).as_bool());
}

TEST_F(ExecutorNodeParametersTest, ExtraParameterCanBeSetAtRuntimeWhenStrictRemovalDisabled)
{
  // A foreign parameter is left untouched by the callback, so it can also be updated at runtime like any other.
  ExtraParamExecutorNode node(makeOptions(false));

  const auto results = node.getNodePtr()->set_parameters({rclcpp::Parameter(kExtraParamName, true)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results.front().successful) << results.front().reason;
  EXPECT_TRUE(node.getNodePtr()->get_parameter(kExtraParamName).as_bool());
}

TEST_F(ExecutorNodeParametersTest, UnknownParameterRejectedWhenStrictRemovalEnabled)
{
  // Default standalone executor: it owns the full parameter set, so an unknown parameter is still rejected.
  GenericTreeExecutorNode node("test_strict_executor", makeOptions(true));

  const auto results = node.getNodePtr()->set_parameters({rclcpp::Parameter("some_unknown_param", true)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results.front().successful);
  EXPECT_NE(results.front().reason.find("unknown"), std::string::npos) << results.front().reason;
}
