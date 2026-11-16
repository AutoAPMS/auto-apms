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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// The parameter nodes (SetParameter*, GetParameter*, HasParameter) create their service client when the tree is
// created. Their 'node' input port may refer to a blackboard entry that is set before that (e.g. by the entry point of
// a behavior), so one tree can target the nodes of several robots whose names are only known at runtime.

static const std::string TREE_TEMPLATE = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <%s node="%s" parameter="p"%s/>
  </BehaviorTree>
</root>
)";

static std::string makeTree(const std::string & node_type, const std::string & node_port, const std::string & extra)
{
  char buffer[512];
  std::snprintf(buffer, sizeof(buffer), TREE_TEMPLATE.c_str(), node_type.c_str(), node_port.c_str(), extra.c_str());
  return buffer;
}

class ParameterNodeTargetTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    ros_node_ = std::make_shared<rclcpp::Node>("parameter_node_target_test");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    callback_group_ = ros_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    builder_ = std::make_unique<TreeBuilder>(ros_node_, callback_group_, executor_);
    NodeManifest manifest;
    for (const std::string name : {"SetParameterString", "GetParameterString", "HasParameter"}) {
      NodeManifest::RegistrationOptions options;
      options.class_name = "auto_apms_behavior_tree::" + name;
      options.allow_unreachable = true;  // No target node runs: only the name of the created client is checked
      manifest.add(name, options);
    }
    builder_->registerNodes(manifest);
  }

  // Whether the ROS 2 node has a client for the given service (the graph may take a moment to show it)
  bool hasClient(const std::string & service, int attempts = 40)
  {
    for (int attempt = 0; attempt < attempts; ++attempt) {
      for (const auto & [name, types] : ros_node_->get_node_graph_interface()->get_client_names_and_types_by_node(
             ros_node_->get_name(), ros_node_->get_namespace())) {
        if (name == service) return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<TreeBuilder> builder_;
};

TEST_F(ParameterNodeTargetTest, StaticNodeName)
{
  builder_->mergeString(makeTree("SetParameterString", "/static_target", " value=\"v\""), true);
  auto tree = builder_->instantiate("TestTree");
  EXPECT_TRUE(hasClient("/static_target/set_parameters"));
}

TEST_F(ParameterNodeTargetTest, NodeNameFromBlackboardEntrySetBeforeCreation)
{
  builder_->mergeString(makeTree("SetParameterString", "{target}", " value=\"v\""), true);
  auto bb = TreeBlackboard::create();
  bb->set("target", std::string("/robot_a/target"));
  auto tree = builder_->instantiate("TestTree", bb);
  EXPECT_TRUE(hasClient("/robot_a/target/set_parameters"));
}

TEST_F(ParameterNodeTargetTest, GetAndHasParameterResolveBlackboardEntries)
{
  builder_->mergeString(
    std::string(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <Sequence>
      <GetParameterString node="{target}" parameter="p" value="{out}"/>
      <HasParameter node="{target}" parameter="p"/>
    </Sequence>
  </BehaviorTree>
</root>
)"),
    true);
  auto bb = TreeBlackboard::create();
  bb->set("target", std::string("/robot_b/target"));
  auto tree = builder_->instantiate("TestTree", bb);
  EXPECT_TRUE(hasClient("/robot_b/target/get_parameters"));
  EXPECT_TRUE(hasClient("/robot_b/target/list_parameters"));
}

TEST_F(ParameterNodeTargetTest, UnsetBlackboardEntryThrows)
{
  builder_->mergeString(makeTree("SetParameterString", "{unset}", " value=\"v\""), true);
  EXPECT_THROW(builder_->instantiate("TestTree"), exceptions::TreeBuildError);
}

// A node manifest may name the parameter service through its 'topic' option instead, e.g. to address a node of the
// robot named by an input port: here the 'node' port is renamed 'robot' and the topic appends the node's name.
class ParameterNodeManifestTopicTest : public ParameterNodeTargetTest
{
protected:
  void SetUp() override
  {
    ParameterNodeTargetTest::SetUp();
    NodeManifest::RegistrationOptions options;
    options.class_name = "auto_apms_behavior_tree::SetParameterString";
    options.topic = "(input:robot)/modes/set_parameters";
    options.port_alias = {{"node", "robot"}};
    options.allow_unreachable = true;
    NodeManifest manifest;
    manifest.add("SetRobotMode", options);
    builder_->registerNodes(manifest);
  }
};

TEST_F(ParameterNodeManifestTopicTest, ServiceNameFromTopicWithBlackboardEntry)
{
  builder_->mergeString(
    R"(<root BTCPP_format="4"><BehaviorTree ID="TestTree">
         <SetRobotMode robot="{robot}" parameter="mode" value="hover"/>
       </BehaviorTree></root>)",
    true);
  auto bb = TreeBlackboard::create();
  bb->set("robot", std::string("/robot_c"));
  auto tree = builder_->instantiate("TestTree", bb);
  EXPECT_TRUE(hasClient("/robot_c/modes/set_parameters"));
}

TEST_F(ParameterNodeManifestTopicTest, ServiceNameFromEntrySetAtRuntimeIsResolvedOnFirstTick)
{
  builder_->mergeString(
    R"(<root BTCPP_format="4"><BehaviorTree ID="TestTree"><Sequence>
         <Script code="robot := '/robot_d'"/>
         <SetRobotMode robot="{robot}" parameter="mode" value="hover"/>
       </Sequence></BehaviorTree></root>)",
    true);
  auto tree = builder_->instantiate("TestTree");
  EXPECT_FALSE(hasClient("/robot_d/modes/set_parameters", 2));
  // The script sets the entry and the first tick of SetRobotMode creates the client. No node offers the service, so
  // the tick fails (allow_unreachable only allows the tree to be created): only the client's name is checked.
  try {
    tree.tickOnce();
    tree.tickOnce();
  } catch (const std::exception &) {
  }
  EXPECT_TRUE(hasClient("/robot_d/modes/set_parameters"));
}
