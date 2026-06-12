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

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// =============================================================================
// Test Fixture
// =============================================================================

class NodeRegistrationTest : public ::testing::Test
{
protected:
  void SetUp() override { doc_ = std::make_unique<TreeDocument>(); }

  std::unique_ptr<TreeDocument> doc_;

  // The builtin Logger node from auto_apms_behavior_tree package
  static constexpr const char * LOGGER_NODE_NAME = "Logger";
  static constexpr const char * LOGGER_CLASS_NAME = "auto_apms_behavior_tree::Logger";

  // Another builtin node for testing different registrations
  static constexpr const char * ERROR_NODE_NAME = "Error";
  static constexpr const char * ERROR_CLASS_NAME = "auto_apms_behavior_tree::ThrowException";

  // Helper to create a node manifest with a single node
  static NodeManifest makeSingleNodeManifest(const std::string & node_name, const std::string & class_name)
  {
    NodeManifest::RegistrationOptions opts;
    opts.class_name = class_name;
    NodeManifest manifest;
    manifest.add(node_name, opts);
    return manifest;
  }
};

// =============================================================================
// registerNodes Basic Functionality Tests
// =============================================================================

TEST_F(NodeRegistrationTest, RegisterSingleNode)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, RegisterMultipleNodes)
{
  NodeManifest manifest;
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  manifest.add(LOGGER_NODE_NAME, opts1);

  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  manifest.add(ERROR_NODE_NAME, opts2);

  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, RegisterNodesReturnsSelf)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  TreeDocument & result = doc_->registerNodes(manifest);

  EXPECT_EQ(&result, doc_.get());
}

// =============================================================================
// Duplicate Registration Without Override Tests
// =============================================================================

TEST_F(NodeRegistrationTest, IdenticalReRegistrationIsNoOp)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  // Re-registering the same node with identical options must be silently ignored
  EXPECT_NO_THROW(doc_->registerNodes(manifest, false));

  // The node must still be registered exactly once
  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, ConflictingClassWithoutOverrideThrows)
{
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  auto manifest2 = makeSingleNodeManifest(LOGGER_NODE_NAME, ERROR_CLASS_NAME);

  ASSERT_NO_THROW(doc_->registerNodes(manifest1));

  // Registering a different class under the same name without override must throw
  EXPECT_THROW(doc_->registerNodes(manifest2, false), exceptions::TreeDocumentError);
}

TEST_F(NodeRegistrationTest, IdenticalReRegistrationInSeparateManifestsIsNoOp)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest);

  // Second manifest with the same node + class: must be a no-op
  auto manifest2 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  EXPECT_NO_THROW(doc_->registerNodes(manifest2, false));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, PartialManifestWithIdenticalDuplicateSucceeds)
{
  // Register one node
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest1);

  // Manifest with a new node AND an identical duplicate: the duplicate is a no-op
  NodeManifest manifest2;
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = ERROR_CLASS_NAME;
  manifest2.add(ERROR_NODE_NAME, opts1);

  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = LOGGER_CLASS_NAME;
  manifest2.add(LOGGER_NODE_NAME, opts2);  // identical duplicate — should be silently skipped

  EXPECT_NO_THROW(doc_->registerNodes(manifest2, false));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, PartialManifestWithConflictingClassThrows)
{
  // Register one node
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest1);

  // Manifest with a new node AND a conflicting re-registration
  NodeManifest manifest2;
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = ERROR_CLASS_NAME;
  manifest2.add(ERROR_NODE_NAME, opts1);

  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;  // different class under the same name
  manifest2.add(LOGGER_NODE_NAME, opts2);

  EXPECT_THROW(doc_->registerNodes(manifest2, false), exceptions::TreeDocumentError);
}

// =============================================================================
// Override Registration Tests
// =============================================================================

TEST_F(NodeRegistrationTest, DuplicateRegistrationWithOverrideSucceeds)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  // Second registration with override=true should succeed
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, OverrideWithDifferentClassSucceeds)
{
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest1);

  // Create a manifest using the same registration name but different class
  auto manifest2 = makeSingleNodeManifest(LOGGER_NODE_NAME, ERROR_CLASS_NAME);

  // With override=true, this should succeed
  ASSERT_NO_THROW(doc_->registerNodes(manifest2, true));

  // Verify the node is still registered
  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, OverrideUpdatesFactory)
{
  // Register Logger node under the "MyNode" name
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  NodeManifest manifest1;
  manifest1.add("MyNode", opts1);
  doc_->registerNodes(manifest1);

  // Now override with Error node under the same "MyNode" name
  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  NodeManifest manifest2;
  manifest2.add("MyNode", opts2);

  ASSERT_NO_THROW(doc_->registerNodes(manifest2, true));

  // Create a tree using the overridden node
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <MyNode/>
    </BehaviorTree>
  </root>)");

  // Verify the tree can be verified (node is usable)
  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

TEST_F(NodeRegistrationTest, MultipleOverrides)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  // Register the same node multiple times with override=true
  doc_->registerNodes(manifest);
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, PartialOverrideInManifest)
{
  // Register one node
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest1);

  // Register a manifest containing a new node AND a duplicate, with override=true
  NodeManifest manifest2;
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = ERROR_CLASS_NAME;
  manifest2.add(ERROR_NODE_NAME, opts1);

  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = LOGGER_CLASS_NAME;
  manifest2.add(LOGGER_NODE_NAME, opts2);  // This would be a duplicate

  // With override=true, this should succeed
  ASSERT_NO_THROW(doc_->registerNodes(manifest2, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u);
}

// =============================================================================
// Native Node Name Reservation Tests
// =============================================================================

TEST_F(NodeRegistrationTest, RegisterNativeNodeNameThrows)
{
  // Attempting to register a node with a name reserved for native nodes should throw
  NodeManifest::RegistrationOptions opts;
  opts.class_name = LOGGER_CLASS_NAME;
  NodeManifest manifest;
  manifest.add("Sequence", opts);  // "Sequence" is a native node name

  EXPECT_THROW(doc_->registerNodes(manifest), exceptions::TreeDocumentError);
}

TEST_F(NodeRegistrationTest, RegisterMultipleNodesWithOneNativeNameThrows)
{
  NodeManifest manifest;

  // Valid node
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  manifest.add(LOGGER_NODE_NAME, opts1);

  // Reserved native name
  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  manifest.add("Fallback", opts2);  // "Fallback" is a native node name

  EXPECT_THROW(doc_->registerNodes(manifest), exceptions::TreeDocumentError);
}

// =============================================================================
// Integration Tests with Tree Usage
// =============================================================================

TEST_F(NodeRegistrationTest, RegisteredNodeCanBeUsedInTree)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest);

  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Logger message="Hello"/>
    </BehaviorTree>
  </root>)");

  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

TEST_F(NodeRegistrationTest, UnregisteredNodeInTreeFailsVerification)
{
  // Don't register any nodes
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Logger message="Hello"/>
    </BehaviorTree>
  </root>)");

  auto result = doc_->verify();
  EXPECT_FALSE(result);
}

TEST_F(NodeRegistrationTest, OverriddenNodeWorksInTree)
{
  // First register Logger under custom name
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  NodeManifest manifest1;
  manifest1.add("CustomAction", opts1);
  doc_->registerNodes(manifest1);

  // Override with Error node
  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  NodeManifest manifest2;
  manifest2.add("CustomAction", opts2);
  doc_->registerNodes(manifest2, true);

  // Create tree using the overridden node
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <CustomAction message="Test"/>
    </BehaviorTree>
  </root>)");

  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

// =============================================================================
// Method Chaining Tests
// =============================================================================

TEST_F(NodeRegistrationTest, RegisterNodesMethodChaining)
{
  NodeManifest manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  NodeManifest manifest2 = makeSingleNodeManifest(ERROR_NODE_NAME, ERROR_CLASS_NAME);

  doc_->registerNodes(manifest1).registerNodes(manifest2);

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, RegisterNodesAndBuildTreeChaining)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  doc_->registerNodes(manifest).mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Logger message="Hello"/>
    </BehaviorTree>
  </root>)");

  EXPECT_TRUE(doc_->hasTreeName("TestTree"));
  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

// =============================================================================
// Parent Field Integration Tests (via installed test_node_manifest resource)
//
// The test manifest `test/resource/test_node_manifest.yaml` is registered with
// `auto_apms_behavior_tree_register_nodes` and processed at build time by the
// `create_node_manifest` CLI tool. Parent fields are resolved during that step
// (NodeManifest::add → resolveParent). These tests verify the full pipeline:
// YAML authoring → build-time resolution → installed resource → runtime loading.
// =============================================================================

TEST(ParentFieldIntegrationTest, LoadTestManifestFromResource)
{
  ASSERT_NO_THROW(
    NodeManifest::fromResource(NodeManifestResourceIdentity("auto_apms_behavior_tree::test_node_manifest")));
}

TEST(ParentFieldIntegrationTest, TestLoggerInheritsClassNameFromLogger)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("TestLogger"));
  EXPECT_EQ(manifest["TestLogger"].class_name, "auto_apms_behavior_tree::Logger");
}

TEST(ParentFieldIntegrationTest, TestLoggerInheritsDescriptionFromLogger)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("TestLogger"));
  EXPECT_EQ(manifest["TestLogger"].description, "Logs a message to the ROS2 logging system");
}

TEST(ParentFieldIntegrationTest, TestLoggerInheritsTopicFromLogger)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("TestLogger"));
  EXPECT_EQ(manifest["TestLogger"].topic, "(input:topic)");
}

TEST(ParentFieldIntegrationTest, CustomLoggerInheritsClassNameFromLogger)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("CustomLogger"));
  EXPECT_EQ(manifest["CustomLogger"].class_name, "auto_apms_behavior_tree::Logger");
}

TEST(ParentFieldIntegrationTest, CustomLoggerOverridesDescription)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("CustomLogger"));
  EXPECT_EQ(manifest["CustomLogger"].description, "Custom logger for testing");
}

TEST(ParentFieldIntegrationTest, CustomLoggerOverridesTopic)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("CustomLogger"));
  EXPECT_EQ(manifest["CustomLogger"].topic, "/custom/log");
}

TEST(ParentFieldIntegrationTest, CustomErrorInheritsClassNameFromError)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("CustomError"));
  EXPECT_EQ(manifest["CustomError"].class_name, "auto_apms_behavior_tree::ThrowException");
}

TEST(ParentFieldIntegrationTest, CustomErrorOverridesDescription)
{
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_TRUE(manifest.contains("CustomError"));
  EXPECT_EQ(manifest["CustomError"].description, "Custom error for testing");
}

TEST(ParentFieldIntegrationTest, ResolvedManifestCanRegisterNodes)
{
  TreeDocument doc;
  const auto manifest = NodeManifest::fromResource("auto_apms_behavior_tree::test_node_manifest");
  ASSERT_NO_THROW(doc.registerNodes(manifest));

  const auto registered = doc.getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count("TestLogger"), 1u);
  EXPECT_EQ(registered.count("CustomLogger"), 1u);
  EXPECT_EQ(registered.count("CustomError"), 1u);
}

// =============================================================================
// Inline Registration Options Merge Tests
// =============================================================================

TEST_F(NodeRegistrationTest, MergeStringWithInlineClassRegistersNode)
{
  const std::string class_attr = std::string(TreeDocument::INLINE_NODE_REGISTRATION_OPTIONS_ATTRIBUTE_PREFIX) +
                                 NodeRegistrationOptions::PARAM_NAME_CLASS;
  const std::string xml = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"TestTree\">") +
                          "<BehaviorTree ID=\"TestTree\">" + "<Sequence><" + std::string(LOGGER_NODE_NAME) + " " +
                          class_attr + "=\"" + LOGGER_CLASS_NAME + "\" message=\"hello\"/></Sequence>" +
                          "</BehaviorTree>" + "</root>";

  ASSERT_NO_THROW(doc_->mergeString(xml, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, MergeStringWithConflictingInlineClassThrows)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest);

  const std::string class_attr = std::string(TreeDocument::INLINE_NODE_REGISTRATION_OPTIONS_ATTRIBUTE_PREFIX) +
                                 NodeRegistrationOptions::PARAM_NAME_CLASS;
  const std::string xml = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"TestTree\">") +
                          "<BehaviorTree ID=\"TestTree\">" + "<Sequence><" + std::string(LOGGER_NODE_NAME) + " " +
                          class_attr + "=\"" + ERROR_CLASS_NAME + "\" message=\"hello\"/></Sequence>" +
                          "</BehaviorTree>" + "</root>";

  EXPECT_THROW(doc_->mergeString(xml, true), exceptions::TreeDocumentError);
}

TEST_F(NodeRegistrationTest, MergeStringWithIdenticalInlineClassIsNoOp)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest);

  const std::string class_attr = std::string(TreeDocument::INLINE_NODE_REGISTRATION_OPTIONS_ATTRIBUTE_PREFIX) +
                                 NodeRegistrationOptions::PARAM_NAME_CLASS;
  const std::string xml = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"TestTree\">") +
                          "<BehaviorTree ID=\"TestTree\">" + "<Sequence><" + std::string(LOGGER_NODE_NAME) + " " +
                          class_attr + "=\"" + LOGGER_CLASS_NAME + "\" message=\"hello\"/></Sequence>" +
                          "</BehaviorTree>" + "</root>";

  // Identical inline options → no-op, no throw, node still registered once
  EXPECT_NO_THROW(doc_->mergeString(xml, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}
