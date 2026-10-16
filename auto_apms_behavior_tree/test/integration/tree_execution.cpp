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

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// The ThrowException node is a pure BT node (no ROS context required) that throws
// exceptions::RosNodeError when ticked, making it ideal for verifying actual execution.
// It also exposes a single "message" input port that is echoed into the exception text,
// which lets us observe how a registered node's ports are resolved at runtime.
static constexpr const char * ERROR_NODE_NAME = "Error";
static constexpr const char * ERROR_CLASS_NAME = "auto_apms_behavior_tree::ThrowException";

// Tree XML template using a single conventionally-registered node.
static const std::string TREE_XML_CONVENTIONAL = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <Error message="test"/>
  </BehaviorTree>
</root>
)";

// =============================================================================
// Helpers
// =============================================================================

/// Builds the attribute name that flags an inline registration option, e.g.
/// `_autoapms_reg_opt__class_name` for the "class_name" option.
static std::string regOptAttr(const std::string & option)
{
  return std::string(TreeDocument::INLINE_NODE_REGISTRATION_OPTIONS_ATTRIBUTE_PREFIX) + option;
}

/// Returns a NodeManifest registering a single ThrowException node under the given name.
static NodeManifest makeErrorManifest(const std::string & node_name)
{
  NodeManifest::RegistrationOptions opts;
  opts.class_name = ERROR_CLASS_NAME;
  NodeManifest manifest;
  manifest.add(node_name, opts);
  return manifest;
}

/// Builds the inline-registration XML for a single ThrowException node carrying only the
/// mandatory `class_name` option (the minimal inline registration).
static std::string makeInlineXml(const std::string & node_name, const std::string & class_name)
{
  return std::string("<root BTCPP_format=\"4\">") + "<BehaviorTree ID=\"TestTree\">" + "<" + node_name + " " +
         regOptAttr(NodeRegistrationOptions::PARAM_NAME_CLASS) + "=\"" + class_name + "\" message=\"test\"/>" +
         "</BehaviorTree>" + "</root>";
}

class TreeExecutionTest : public ::testing::Test
{
protected:
  // Use the non-ROS TreeBuilder constructor — sufficient for nodes that don't
  // require a RosNodeContext (ThrowException falls into this category).
  void SetUp() override { builder_ = std::make_unique<TreeBuilder>(); }

  std::unique_ptr<TreeBuilder> builder_;
};

// =============================================================================
// Conventional Registration Execution Tests
//
// These tests verify that nodes registered via NodeManifest (the conventional
// approach) are loaded by the plugin loader, assembled into a runnable tree by
// instantiate(), and actually execute when the tree is ticked. Plain registration
// (getRegisteredNodeNames) is already covered by the unit suite — here we go all
// the way to instantiation and ticking, which the unit tests do not.
// =============================================================================

TEST_F(TreeExecutionTest, ConventionalRegistrationInstantiatesTree)
{
  builder_->registerNodes(makeErrorManifest(ERROR_NODE_NAME));
  builder_->mergeString(TREE_XML_CONVENTIONAL);

  // instantiate() must succeed — proving the registered plugin is usable
  EXPECT_NO_THROW(builder_->instantiate("TestTree"));
}

TEST_F(TreeExecutionTest, ConventionalRegistrationExecutesNode)
{
  builder_->registerNodes(makeErrorManifest(ERROR_NODE_NAME));
  builder_->mergeString(TREE_XML_CONVENTIONAL);

  auto tree = builder_->instantiate("TestTree");

  // ThrowException throws exceptions::RosNodeError on tick, which BT.CPP
  // wraps in BT::NodeExecutionError — confirming the plugin was correctly
  // loaded and its tick() implementation ran.
  EXPECT_THROW(tree.tickWhileRunning(), std::exception);
}

// =============================================================================
// Inline Registration Execution Tests
//
// These tests verify the inline registration path: when a tree node element
// carries a "registration options" attribute, mergeString() triggers
// plugin loading automatically without a prior registerNodes() call.
// =============================================================================

TEST_F(TreeExecutionTest, InlineRegistrationInstantiatesTree)
{
  builder_->mergeString(makeInlineXml(ERROR_NODE_NAME, ERROR_CLASS_NAME));

  // instantiate() must succeed — the plugin was loaded by mergeString
  EXPECT_NO_THROW(builder_->instantiate("TestTree"));
}

TEST_F(TreeExecutionTest, InlineRegistrationExecutesNode)
{
  builder_->mergeString(makeInlineXml(ERROR_NODE_NAME, ERROR_CLASS_NAME));

  auto tree = builder_->instantiate("TestTree");

  // Same execution result as conventional registration — the plugin ran
  EXPECT_THROW(tree.tickWhileRunning(), std::exception);
}

// =============================================================================
// Equivalence Test
//
// Verifies that both registration paths produce identical runtime behaviour.
// =============================================================================

TEST(TreeExecutionEquivalenceTest, ConventionalAndInlineProduceSameResult)
{
  // --- Conventional ---
  TreeBuilder builder_conv;
  builder_conv.registerNodes(makeErrorManifest(ERROR_NODE_NAME));
  builder_conv.mergeString(TREE_XML_CONVENTIONAL);
  auto tree_conv = builder_conv.instantiate("TestTree");

  // --- Inline ---
  TreeBuilder builder_inline;
  builder_inline.mergeString(makeInlineXml(ERROR_NODE_NAME, ERROR_CLASS_NAME));
  auto tree_inline = builder_inline.instantiate("TestTree");

  // Both trees must throw the same exception type when ticked
  EXPECT_THROW(tree_conv.tickWhileRunning(), std::exception);
  EXPECT_THROW(tree_inline.tickWhileRunning(), std::exception);
}

// =============================================================================
// Inline Registration Options Beyond class_name
//
// Inline registration parses arbitrary NodeRegistrationOptions fields, not just
// class_name. This verifies that a non-class option (a port_default) is parsed
// from the XML attribute and propagates all the way into the running node — a
// path that verify()-level unit tests cannot exercise.
// =============================================================================

TEST_F(TreeExecutionTest, InlineRegistrationAppliesPortDefault)
{
  // Register the Error node inline with a port_default for its "message" port. The
  // element deliberately does NOT set "message" itself, so the default must supply it.
  const std::string xml = std::string("<root BTCPP_format=\"4\">") + "<BehaviorTree ID=\"TestTree\">" + "<" +
                          ERROR_NODE_NAME + " " + regOptAttr(NodeRegistrationOptions::PARAM_NAME_CLASS) + "=\"" +
                          ERROR_CLASS_NAME + "\" " + regOptAttr(NodeRegistrationOptions::PARAM_NAME_PORT_DEFAULT) +
                          "=\"{message: from_default}\"/>" + "</BehaviorTree>" + "</root>";

  builder_->mergeString(xml);
  auto tree = builder_->instantiate("TestTree");

  // ThrowException embeds the resolved "message" input in its exception text. Seeing
  // "from_default" there proves the inline port_default was parsed and applied.
  try {
    tree.tickWhileRunning();
    FAIL() << "Expected tickWhileRunning() to throw";
  } catch (const std::exception & e) {
    EXPECT_NE(std::string(e.what()).find("from_default"), std::string::npos)
      << "Exception text did not contain the inline port_default value: " << e.what();
  }
}

// =============================================================================
// Mixed Registration Paths in a Single Tree
//
// The equivalence test runs the two paths in separate builders. This verifies they
// coexist within one document and one instantiated tree: instantiate() resolves the
// whole tree, so it only succeeds if both the conventionally- and inline-registered
// factories are present.
// =============================================================================

TEST_F(TreeExecutionTest, ConventionalAndInlineNodesCoexistInOneTree)
{
  // "ConvError" is registered conventionally via a manifest...
  builder_->registerNodes(makeErrorManifest("ConvError"));

  // ...while "InlineError" is registered inline during mergeString. Both share one tree.
  const std::string xml = std::string("<root BTCPP_format=\"4\">") + "<BehaviorTree ID=\"TestTree\">" + "<Sequence>" +
                          "<ConvError message=\"conv\"/>" + "<InlineError " +
                          regOptAttr(NodeRegistrationOptions::PARAM_NAME_CLASS) + "=\"" + ERROR_CLASS_NAME +
                          "\" message=\"inline\"/>" + "</Sequence>" + "</BehaviorTree>" + "</root>";
  builder_->mergeString(xml);

  const auto registered = builder_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count("ConvError"), 1u);
  EXPECT_EQ(registered.count("InlineError"), 1u);

  // instantiate() builds the whole Sequence, which requires both factories to resolve.
  auto tree = builder_->instantiate("TestTree");

  // The first child (ConvError) throws on tick, confirming execution proceeds into the
  // mixed tree. (The Sequence short-circuits, so InlineError's tick is not reached here.)
  EXPECT_THROW(tree.tickWhileRunning(), std::exception);
}

// =============================================================================
// SubTree Elements Are Not Inline-Registered
//
// Inline registration options are collected recursively across every tree in the
// document. A <SubTree> reference element must never be mistaken for an inline node
// registration, even if it happens to carry a registration-option attribute, while
// genuine inline nodes nested inside the referenced subtree must still be registered.
// =============================================================================

TEST_F(TreeExecutionTest, SubTreeReferenceIsNotTreatedAsInlineRegistration)
{
  // "Main" references subtree "Sub" and — to exercise the guard directly — puts a stray
  // registration-option attribute on the <SubTree> element. "Sub" contains a genuine
  // inline Error node.
  const std::string xml = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">") +
                          "<BehaviorTree ID=\"Main\">" + "<SubTree ID=\"Sub\" " +
                          regOptAttr(NodeRegistrationOptions::PARAM_NAME_CLASS) + "=\"" + ERROR_CLASS_NAME + "\"/>" +
                          "</BehaviorTree>" + "<BehaviorTree ID=\"Sub\">" + "<" + ERROR_NODE_NAME + " " +
                          regOptAttr(NodeRegistrationOptions::PARAM_NAME_CLASS) + "=\"" + ERROR_CLASS_NAME +
                          "\" message=\"from_subtree\"/>" + "</BehaviorTree>" + "</root>";

  ASSERT_NO_THROW(builder_->mergeString(xml));

  const auto registered = builder_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u) << "Genuine inline node inside the subtree should be registered";
  EXPECT_EQ(registered.count(TreeDocument::SUBTREE_ELEMENT_NAME), 0u)
    << "The <SubTree> reference must not be registered as a node";

  // The inline node nested in the subtree still executes through the parent tree.
  auto tree = builder_->instantiate("Main");
  EXPECT_THROW(tree.tickWhileRunning(), std::exception);
}
