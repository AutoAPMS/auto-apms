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

#include <string>
#include <vector>

#include "auto_apms_behavior_tree_core/node/base/ros_condition_node.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_template.hpp"
#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_tree::core;

// A minimal ROS-aware condition node with a single std::vector<std::string> input port. On tick it reads the port and
// stores the received value so the test can assert what the node actually observed after port aliasing was applied.
class VectorPortConditionNode : public RosConditionNode
{
public:
  VectorPortConditionNode(const std::string & instance_name, const Config & config, Context context)
  : RosConditionNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::vector<std::string>>("vector_port")}; }

  BT::NodeStatus tick() override
  {
    const BT::Expected<std::vector<std::string>> expected = getInput<std::vector<std::string>>("vector_port");
    if (!expected) {
      last_error_ = expected.error();
      return BT::NodeStatus::FAILURE;
    }
    received_ = expected.value();
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<std::string> received_;
  std::string last_error_;
};

// A condition node with several input ports of different (non-string-vector) types, used to check that aliasing works
// regardless of the port's value type.
class MixedTypesConditionNode : public RosConditionNode
{
public:
  MixedTypesConditionNode(const std::string & instance_name, const Config & config, Context context)
  : RosConditionNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<int>>("int_vector_port"),
      BT::InputPort<int>("int_port"),
      BT::InputPort<std::string>("string_port"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (
      !getInput("int_vector_port", int_vector_) || !getInput("int_port", integer_) || !getInput("string_port", text_)) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<int> int_vector_;
  int integer_ = 0;
  std::string text_;
};

// A condition node with a single input port that declares a default value, used to check the interaction between
// aliasing and port defaults.
class DefaultPortConditionNode : public RosConditionNode
{
public:
  static constexpr int DEFAULT_COUNT = 42;

  DefaultPortConditionNode(const std::string & instance_name, const Config & config, Context context)
  : RosConditionNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("count", DEFAULT_COUNT, "A count with a default value")};
  }

  BT::NodeStatus tick() override
  {
    if (!getInput("count", count_)) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
  }

  int count_ = 0;
};

class RosNodePortAliasTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_port_alias_node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Register node type NodeT with the given registration options (exercising the exact production path:
  // NodeRegistrationTemplate adds the aliased port to the ports list and passes the context on to the factory).
  template <typename NodeT>
  void registerNode(const std::string & registration_name, const NodeRegistrationOptions & options)
  {
    context_ptr_ = std::make_unique<RosNodeContext>(node_, callback_group_, executor_, options);
    const NodeRegistrationTemplate<NodeT> registration;
    registration.registerWithBehaviorTreeFactory(factory_, registration_name, context_ptr_.get());
  }

  // Find the (single) instantiated node of type NodeT in the tree.
  template <typename NodeT>
  static NodeT * findNode(BT::Tree & tree)
  {
    NodeT * found = nullptr;
    for (const auto & subtree : tree.subtrees) {
      for (const auto & node : subtree->nodes) {
        if (auto * const casted = dynamic_cast<NodeT *>(node.get())) found = casted;
      }
    }
    return found;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<RosNodeContext> context_ptr_;
};

// A string-vector value written as a literal on the aliased port must reach the original port the node reads.
TEST_F(RosNodePortAliasTest, AliasedStringVectorLiteral)
{
  NodeRegistrationOptions options;
  options.class_name = "test::VectorPortConditionNode";
  options.port_alias["vector_port"] = "aliased_vector_port";
  registerNode<VectorPortConditionNode>("VectorPortConditionNode", options);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <VectorPortConditionNode aliased_vector_port="alpha;beta;gamma"/>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  VectorPortConditionNode * const node = findNode<VectorPortConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->received_, (std::vector<std::string>{"alpha", "beta", "gamma"}))
    << "Aliased string-vector literal was not forwarded to the original port (got a list of size "
    << node->received_.size() << ").";
}

// The same must hold when the aliased port points at a blackboard entry that holds a std::vector<std::string>.
TEST_F(RosNodePortAliasTest, AliasedStringVectorFromBlackboard)
{
  NodeRegistrationOptions options;
  options.class_name = "test::VectorPortConditionNode";
  options.port_alias["vector_port"] = "aliased_vector_port";
  registerNode<VectorPortConditionNode>("VectorPortConditionNode", options);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <VectorPortConditionNode aliased_vector_port="{vec}"/>
      </BehaviorTree>
    </root>)";

  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  blackboard->set("vec", std::vector<std::string>{"one", "two", "three"});

  BT::Tree tree = factory_.createTreeFromText(xml, blackboard);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  VectorPortConditionNode * const node = findNode<VectorPortConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->received_, (std::vector<std::string>{"one", "two", "three"}))
    << "Aliased string-vector from the blackboard was not forwarded to the original port (got a list of size "
    << node->received_.size() << ").";
}

// Sanity check: without any alias, providing the value directly on the original port works. This isolates whether a
// failure above is specific to aliasing rather than to string-vector port handling in general.
TEST_F(RosNodePortAliasTest, NonAliasedStringVectorBaseline)
{
  NodeRegistrationOptions options;
  options.class_name = "test::VectorPortConditionNode";
  registerNode<VectorPortConditionNode>("VectorPortConditionNode", options);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <VectorPortConditionNode vector_port="alpha;beta;gamma"/>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  VectorPortConditionNode * const node = findNode<VectorPortConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->received_, (std::vector<std::string>{"alpha", "beta", "gamma"}));
}

// Aliasing must work for arbitrary port value types, not just string vectors: an int vector, an int and a string are
// each provided on their aliased ports and must reach the original ports.
TEST_F(RosNodePortAliasTest, AliasedMixedTypes)
{
  NodeRegistrationOptions options;
  options.class_name = "test::MixedTypesConditionNode";
  options.port_alias["int_vector_port"] = "aliased_int_vector";
  options.port_alias["int_port"] = "aliased_int";
  options.port_alias["string_port"] = "aliased_string";
  registerNode<MixedTypesConditionNode>("MixedTypesConditionNode", options);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <MixedTypesConditionNode aliased_int_vector="1;2;3" aliased_int="7" aliased_string="hello"/>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  MixedTypesConditionNode * const node = findNode<MixedTypesConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->int_vector_, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(node->integer_, 7);
  EXPECT_EQ(node->text_, "hello");
}

// A value set on the aliased port must override the original port's declared default value.
TEST_F(RosNodePortAliasTest, AliasedPortDefaultOverridden)
{
  NodeRegistrationOptions options;
  options.class_name = "test::DefaultPortConditionNode";
  options.port_alias["count"] = "aliased_count";
  registerNode<DefaultPortConditionNode>("DefaultPortConditionNode", options);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <DefaultPortConditionNode aliased_count="99"/>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  DefaultPortConditionNode * const node = findNode<DefaultPortConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->count_, 99);
}

// When the aliased port is left unset, the original port's declared default value must still apply (aliasing must not
// clobber the default with an empty value).
TEST_F(RosNodePortAliasTest, AliasedPortDefaultUsedWhenUnset)
{
  NodeRegistrationOptions options;
  options.class_name = "test::DefaultPortConditionNode";
  options.port_alias["count"] = "aliased_count";
  registerNode<DefaultPortConditionNode>("DefaultPortConditionNode", options);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <DefaultPortConditionNode/>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  DefaultPortConditionNode * const node = findNode<DefaultPortConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->count_, DefaultPortConditionNode::DEFAULT_COUNT);
}

// The optional alias description (in round brackets) is applied to the aliased port and the alias name is parsed
// without the description suffix.
TEST_F(RosNodePortAliasTest, AliasDescriptionSimple)
{
  NodeRegistrationOptions options;
  options.class_name = "test::VectorPortConditionNode";
  options.port_alias["vector_port"] = "aliased_vector_port (A list of names)";
  ASSERT_NO_THROW(registerNode<VectorPortConditionNode>("VectorPortConditionNode", options));

  const auto & manifests = factory_.manifests();
  ASSERT_TRUE(manifests.find("VectorPortConditionNode") != manifests.end());
  const BT::PortsList & ports = manifests.at("VectorPortConditionNode").ports;
  ASSERT_TRUE(ports.find("aliased_vector_port") != ports.end());
  EXPECT_EQ(ports.at("aliased_vector_port").description(), "A list of names");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <VectorPortConditionNode aliased_vector_port="alpha;beta"/>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  VectorPortConditionNode * const node = findNode<VectorPortConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->received_, (std::vector<std::string>{"alpha", "beta"}));
}

// The optional alias description (in round brackets) may itself contain round brackets: the outermost pair delimits
// the description while inner brackets are preserved. Registration must succeed, the parsed description must be applied
// to the aliased port, and the port value must still flow through.
TEST_F(RosNodePortAliasTest, AliasDescriptionWithNestedBrackets)
{
  NodeRegistrationOptions options;
  options.class_name = "test::VectorPortConditionNode";
  // Description contains both a nested group and a second, separate group; both must be preserved.
  options.port_alias["vector_port"] = "aliased_vector_port (Coordinates (x, y) and bounds (min, max))";
  ASSERT_NO_THROW(registerNode<VectorPortConditionNode>("VectorPortConditionNode", options));

  // The parsed description is everything between the outermost brackets, with the inner brackets kept intact.
  const auto & manifests = factory_.manifests();
  ASSERT_TRUE(manifests.find("VectorPortConditionNode") != manifests.end());
  const BT::PortsList & ports = manifests.at("VectorPortConditionNode").ports;
  ASSERT_TRUE(ports.find("aliased_vector_port") != ports.end());
  EXPECT_EQ(ports.at("aliased_vector_port").description(), "Coordinates (x, y) and bounds (min, max)");

  // The alias itself (the name before the brackets) must be parsed cleanly and still forward the value.
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <VectorPortConditionNode aliased_vector_port="alpha;beta"/>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  VectorPortConditionNode * const node = findNode<VectorPortConditionNode>(tree);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->received_, (std::vector<std::string>{"alpha", "beta"}));
}
