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

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"

#include <gtest/gtest.h>

#include "auto_apms_behavior_tree_core/exceptions.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// Helper function to create valid registration options
NodeManifest::RegistrationOptions makeOptions(const std::string & class_name)
{
  NodeManifest::RegistrationOptions opts;
  opts.class_name = class_name;
  return opts;
}

class NodeManifestTest : public ::testing::Test
{
protected:
  void SetUp() override { manifest_ = std::make_unique<NodeManifest>(); }

  std::unique_ptr<NodeManifest> manifest_;
};

// =============================================================================
// Basic Operations Tests
// =============================================================================

TEST_F(NodeManifestTest, DefaultConstructorCreatesEmptyManifest)
{
  EXPECT_TRUE(manifest_->empty());
  EXPECT_EQ(manifest_->size(), 0u);
}

TEST_F(NodeManifestTest, AddSingleNode)
{
  manifest_->add("TestNode", makeOptions("my_package::TestNode"));

  EXPECT_FALSE(manifest_->empty());
  EXPECT_EQ(manifest_->size(), 1u);
  EXPECT_TRUE(manifest_->contains("TestNode"));
  EXPECT_EQ((*manifest_)["TestNode"].class_name, "my_package::TestNode");
}

TEST_F(NodeManifestTest, AddMultipleNodes)
{
  manifest_->add("Node1", makeOptions("pkg::Node1"));
  manifest_->add("Node2", makeOptions("pkg::Node2"));
  manifest_->add("Node3", makeOptions("pkg::Node3"));

  EXPECT_EQ(manifest_->size(), 3u);
  EXPECT_TRUE(manifest_->contains("Node1"));
  EXPECT_TRUE(manifest_->contains("Node2"));
  EXPECT_TRUE(manifest_->contains("Node3"));
}

TEST_F(NodeManifestTest, AddDuplicateNodeThrows)
{
  manifest_->add("TestNode", makeOptions("pkg::TestNode"));

  EXPECT_THROW(manifest_->add("TestNode", makeOptions("pkg::OtherNode")), exceptions::NodeManifestError);
}

TEST_F(NodeManifestTest, AddNodeWithEmptyClassNameThrows)
{
  NodeManifest::RegistrationOptions invalid_opts;
  // class_name is empty by default

  EXPECT_THROW(manifest_->add("TestNode", invalid_opts), exceptions::NodeManifestError);
}

TEST_F(NodeManifestTest, RemoveExistingNode)
{
  manifest_->add("TestNode", makeOptions("pkg::TestNode"));
  EXPECT_TRUE(manifest_->contains("TestNode"));

  manifest_->remove("TestNode");

  EXPECT_FALSE(manifest_->contains("TestNode"));
  EXPECT_TRUE(manifest_->empty());
}

TEST_F(NodeManifestTest, RemoveNonExistingNodeThrows)
{
  EXPECT_THROW(manifest_->remove("NonExistent"), std::out_of_range);
}

TEST_F(NodeManifestTest, AccessNonExistingNodeThrows) { EXPECT_THROW((*manifest_)["NonExistent"], std::out_of_range); }

TEST_F(NodeManifestTest, GetNodeNames)
{
  manifest_->add("Alpha", makeOptions("pkg::Alpha"));
  manifest_->add("Beta", makeOptions("pkg::Beta"));
  manifest_->add("Gamma", makeOptions("pkg::Gamma"));

  std::vector<std::string> names = manifest_->getNodeNames();

  EXPECT_EQ(names.size(), 3u);
  EXPECT_TRUE(std::find(names.begin(), names.end(), "Alpha") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "Beta") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "Gamma") != names.end());
}

// =============================================================================
// Merge Tests (bool replace overload)
// =============================================================================

TEST_F(NodeManifestTest, MergeDisjointManifests)
{
  manifest_->add("Node1", makeOptions("pkg::Node1"));

  NodeManifest other;
  other.add("Node2", makeOptions("pkg::Node2"));

  manifest_->merge(other);

  EXPECT_EQ(manifest_->size(), 2u);
  EXPECT_TRUE(manifest_->contains("Node1"));
  EXPECT_TRUE(manifest_->contains("Node2"));
}

TEST_F(NodeManifestTest, MergeOverlappingManifestsWithoutReplaceThrows)
{
  manifest_->add("SharedNode", makeOptions("pkg::Original"));

  NodeManifest other;
  other.add("SharedNode", makeOptions("pkg::Replacement"));

  EXPECT_THROW(manifest_->merge(other, false), exceptions::NodeManifestError);
}

TEST_F(NodeManifestTest, MergeOverlappingManifestsWithReplaceSucceeds)
{
  manifest_->add("SharedNode", makeOptions("pkg::Original"));

  NodeManifest other;
  other.add("SharedNode", makeOptions("pkg::Replacement"));

  ASSERT_NO_THROW(manifest_->merge(other, true));

  EXPECT_EQ(manifest_->size(), 1u);
  EXPECT_EQ((*manifest_)["SharedNode"].class_name, "pkg::Replacement");
}

TEST_F(NodeManifestTest, MergeEmptyManifest)
{
  manifest_->add("Node1", makeOptions("pkg::Node1"));

  NodeManifest empty;
  manifest_->merge(empty);

  EXPECT_EQ(manifest_->size(), 1u);
  EXPECT_TRUE(manifest_->contains("Node1"));
}

TEST_F(NodeManifestTest, MergeIntoEmptyManifest)
{
  NodeManifest other;
  other.add("Node1", makeOptions("pkg::Node1"));
  other.add("Node2", makeOptions("pkg::Node2"));

  manifest_->merge(other);

  EXPECT_EQ(manifest_->size(), 2u);
  EXPECT_TRUE(manifest_->contains("Node1"));
  EXPECT_TRUE(manifest_->contains("Node2"));
}

// =============================================================================
// Merge Tests (with_namespace overload)
// =============================================================================

TEST_F(NodeManifestTest, MergeWithNamespaceDefaultSeparator)
{
  manifest_->add("ExistingNode", makeOptions("pkg::Existing"));

  NodeManifest other;
  other.add("Node1", makeOptions("pkg::Node1"));
  other.add("Node2", makeOptions("pkg::Node2"));

  manifest_->mergeWithNamespace(other, "my_ns");

  EXPECT_EQ(manifest_->size(), 3u);
  EXPECT_TRUE(manifest_->contains("ExistingNode"));
  EXPECT_TRUE(manifest_->contains("my_ns.Node1"));
  EXPECT_TRUE(manifest_->contains("my_ns.Node2"));
}

TEST_F(NodeManifestTest, MergeWithNamespaceCustomSeparator)
{
  NodeManifest other;
  other.add("Node1", makeOptions("pkg::Node1"));

  manifest_->mergeWithNamespace(other, "ns", "::");

  EXPECT_TRUE(manifest_->contains("ns::Node1"));
}

TEST_F(NodeManifestTest, MergeWithNamespaceSlashSeparator)
{
  NodeManifest other;
  other.add("Node1", makeOptions("pkg::Node1"));

  manifest_->mergeWithNamespace(other, "a/b", "/");

  EXPECT_TRUE(manifest_->contains("a/b/Node1"));
}

TEST_F(NodeManifestTest, MergeWithNamespaceAvoidsDuplicates)
{
  // Add a node that would conflict with the namespaced version
  manifest_->add("my_ns.Node1", makeOptions("pkg::Conflicting"));

  NodeManifest other;
  other.add("Node1", makeOptions("pkg::Node1"));

  EXPECT_THROW(manifest_->mergeWithNamespace(other, "my_ns"), exceptions::NodeManifestError);
}

TEST_F(NodeManifestTest, MergeWithNamespacePreservesOriginalNames)
{
  // Ensure the original names are prefixed correctly
  NodeManifest other;
  other.add("MoveToTarget", makeOptions("pkg::MoveToTarget"));
  other.add("CheckBattery", makeOptions("pkg::CheckBattery"));

  manifest_->mergeWithNamespace(other, "robot1");

  EXPECT_TRUE(manifest_->contains("robot1.MoveToTarget"));
  EXPECT_TRUE(manifest_->contains("robot1.CheckBattery"));
  EXPECT_FALSE(manifest_->contains("MoveToTarget"));
  EXPECT_FALSE(manifest_->contains("CheckBattery"));
}

// =============================================================================
// ApplyNamespace Tests
// =============================================================================

TEST_F(NodeManifestTest, ApplyNamespaceToEmptyManifest)
{
  manifest_->applyNodeNamespace("ns");

  EXPECT_TRUE(manifest_->empty());
}

TEST_F(NodeManifestTest, ApplyNamespaceDefaultSeparator)
{
  manifest_->add("Node1", makeOptions("pkg::Node1"));
  manifest_->add("Node2", makeOptions("pkg::Node2"));

  manifest_->applyNodeNamespace("my_ns");

  EXPECT_EQ(manifest_->size(), 2u);
  EXPECT_TRUE(manifest_->contains("my_ns.Node1"));
  EXPECT_TRUE(manifest_->contains("my_ns.Node2"));
  EXPECT_FALSE(manifest_->contains("Node1"));
  EXPECT_FALSE(manifest_->contains("Node2"));
}

TEST_F(NodeManifestTest, ApplyNamespaceCustomSeparator)
{
  manifest_->add("Node1", makeOptions("pkg::Node1"));

  manifest_->applyNodeNamespace("my_ns", "::");

  EXPECT_TRUE(manifest_->contains("my_ns::Node1"));
  EXPECT_FALSE(manifest_->contains("Node1"));
}

TEST_F(NodeManifestTest, ApplyNamespacePreservesOptions)
{
  auto opts = makeOptions("pkg::TestNode");
  opts.description = "Custom description";
  opts.topic = "/custom/topic";
  manifest_->add("TestNode", opts);

  manifest_->applyNodeNamespace("ns");

  EXPECT_EQ((*manifest_)["ns.TestNode"].class_name, "pkg::TestNode");
  EXPECT_EQ((*manifest_)["ns.TestNode"].description, "Custom description");
  EXPECT_EQ((*manifest_)["ns.TestNode"].topic, "/custom/topic");
}

TEST_F(NodeManifestTest, ApplyNamespaceChained)
{
  manifest_->add("Node1", makeOptions("pkg::Node1"));

  manifest_->applyNodeNamespace("inner").applyNodeNamespace("outer");

  EXPECT_TRUE(manifest_->contains("outer.inner.Node1"));
}

// =============================================================================
// Method Chaining Tests
// =============================================================================

TEST_F(NodeManifestTest, MethodChainingAdd)
{
  manifest_->add("Node1", makeOptions("pkg::Node1"))
    .add("Node2", makeOptions("pkg::Node2"))
    .add("Node3", makeOptions("pkg::Node3"));

  EXPECT_EQ(manifest_->size(), 3u);
}

TEST_F(NodeManifestTest, MethodChainingMerge)
{
  NodeManifest other1, other2;
  other1.add("Node1", makeOptions("pkg::Node1"));
  other2.add("Node2", makeOptions("pkg::Node2"));

  manifest_->merge(other1).merge(other2);

  EXPECT_EQ(manifest_->size(), 2u);
}

TEST_F(NodeManifestTest, MethodChainingMixed)
{
  NodeManifest other;
  other.add("OtherNode", makeOptions("pkg::OtherNode"));

  manifest_->add("Node1", makeOptions("pkg::Node1")).merge(other).applyNodeNamespace("ns");

  EXPECT_EQ(manifest_->size(), 2u);
  EXPECT_TRUE(manifest_->contains("ns.Node1"));
  EXPECT_TRUE(manifest_->contains("ns.OtherNode"));
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST_F(NodeManifestTest, NodeNameWithSpecialCharacters)
{
  // Test that node names with various characters work correctly
  manifest_->add("Node::With::Colons", makeOptions("pkg::Node1"));
  manifest_->add("Node.With.Dots", makeOptions("pkg::Node2"));
  manifest_->add("Node/With/Slashes", makeOptions("pkg::Node3"));

  EXPECT_TRUE(manifest_->contains("Node::With::Colons"));
  EXPECT_TRUE(manifest_->contains("Node.With.Dots"));
  EXPECT_TRUE(manifest_->contains("Node/With/Slashes"));
}

TEST_F(NodeManifestTest, ConstructFromMap)
{
  NodeManifest::Map initial_map;
  initial_map["Node1"] = makeOptions("pkg::Node1");
  initial_map["Node2"] = makeOptions("pkg::Node2");

  NodeManifest manifest(initial_map);

  EXPECT_EQ(manifest.size(), 2u);
  EXPECT_TRUE(manifest.contains("Node1"));
  EXPECT_TRUE(manifest.contains("Node2"));
}
