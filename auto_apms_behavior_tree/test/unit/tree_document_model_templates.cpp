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

#include <vector>

#include "auto_apms_behavior_tree/behavior_tree_nodes.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// =============================================================================
// Test Fixture
// =============================================================================

class TreeDocumentModelTemplateTest : public ::testing::Test
{
protected:
  void SetUp() override { doc_ = std::make_unique<TreeDocument>(); }

  std::unique_ptr<TreeDocument> doc_;
};

// =============================================================================
// insertNode<T> Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, InsertControlNode)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();

  EXPECT_EQ(seq.getRegistrationName(), "Sequence");
}

TEST_F(TreeDocumentModelTemplateTest, InsertDecoratorNode)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto inv = seq.insertNode<model::Inverter>();

  EXPECT_EQ(inv.getRegistrationName(), "Inverter");
}

TEST_F(TreeDocumentModelTemplateTest, InsertLeafNode)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto success = seq.insertNode<model::AlwaysSuccess>();

  EXPECT_EQ(success.getRegistrationName(), "AlwaysSuccess");
}

TEST_F(TreeDocumentModelTemplateTest, InsertNodeWithRegistrationOptions)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto logger = seq.insertNode<model::Logger>();

  // Logger has registrationOptions(), so it should auto-register
  EXPECT_EQ(logger.getRegistrationName(), "Logger");
}

TEST_F(TreeDocumentModelTemplateTest, InsertedNodeRetainsModelType)
{
  auto tree = doc_->newTree("TestTree");
  model::Sequence seq = tree.insertNode<model::Sequence>();
  model::AlwaysSuccess success = seq.insertNode<model::AlwaysSuccess>();

  // Verify the exact model type is returned (not just NodeElement)
  EXPECT_EQ(success.getRegistrationName(), "AlwaysSuccess");
}

TEST_F(TreeDocumentModelTemplateTest, InsertMultipleChildrenWithModels)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();
  seq.insertNode<model::AlwaysFailure>();

  std::vector<std::string> names;
  for (auto child : seq) {
    names.push_back(child.getRegistrationName());
  }

  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "AlwaysSuccess");
  EXPECT_EQ(names[1], "AlwaysFailure");
}

TEST_F(TreeDocumentModelTemplateTest, InsertNodeBeforeAnother)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto second = seq.insertNode<model::AlwaysFailure>();

  // Insert AlwaysSuccess before AlwaysFailure
  seq.insertNode<model::AlwaysSuccess>(&second);

  std::vector<std::string> names;
  for (auto child : seq) {
    names.push_back(child.getRegistrationName());
  }

  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "AlwaysSuccess");
  EXPECT_EQ(names[1], "AlwaysFailure");
}

TEST_F(TreeDocumentModelTemplateTest, InsertNestedControlNodes)
{
  auto tree = doc_->newTree("TestTree");
  auto fallback = tree.insertNode<model::Fallback>();
  auto seq = fallback.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();

  EXPECT_EQ(fallback.getRegistrationName(), "Fallback");
  auto first = fallback.getFirstNode();
  EXPECT_EQ(first.getRegistrationName(), "Sequence");
}

// =============================================================================
// insertNode<SubTree> Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, InsertSubTreeNodeByName)
{
  auto main_tree = doc_->newTree("MainTree");
  auto sub_tree = doc_->newTree("SubTree");
  sub_tree.insertNode<model::AlwaysSuccess>();

  auto seq = main_tree.insertNode<model::Sequence>();
  auto subtree_node = seq.insertNode<model::SubTree>("SubTree");

  EXPECT_EQ(subtree_node.getRegistrationName(), "SubTree");
}

TEST_F(TreeDocumentModelTemplateTest, InsertSubTreeNodeByTreeElement)
{
  auto sub_tree = doc_->newTree("SubTree");
  sub_tree.insertNode<model::AlwaysSuccess>();

  auto main_tree = doc_->newTree("MainTree");
  auto seq = main_tree.insertNode<model::Sequence>();
  auto subtree_node = seq.insertNode<model::SubTree>(sub_tree);

  EXPECT_EQ(subtree_node.getRegistrationName(), "SubTree");
}

// =============================================================================
// Port Methods via Model Types Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, SetPortsViaModelSetters)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto repeat = seq.insertNode<model::Repeat>();
  repeat.set_num_cycles(5);

  EXPECT_EQ(repeat.get_num_cycles(), 5);
  EXPECT_EQ(repeat.get_num_cycles_str(), "5");
}

TEST_F(TreeDocumentModelTemplateTest, SetPortsViaStringSetters)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto repeat = seq.insertNode<model::Repeat>();
  repeat.set_num_cycles("10");

  EXPECT_EQ(repeat.get_num_cycles_str(), "10");
  EXPECT_EQ(repeat.get_num_cycles(), 10);
}

TEST_F(TreeDocumentModelTemplateTest, ModelMethodChaining)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();

  // Model setters return *this for chaining
  auto repeat = seq.insertNode<model::Repeat>();
  auto & ref = repeat.set_num_cycles(3);

  EXPECT_EQ(ref.get_num_cycles(), 3);
  EXPECT_EQ(&ref, &repeat);
}

TEST_F(TreeDocumentModelTemplateTest, SetAndGetMultiplePorts)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto parallel = seq.insertNode<model::Parallel>();
  parallel.set_failure_count(2);
  parallel.set_success_count(3);

  EXPECT_EQ(parallel.get_failure_count(), 2);
  EXPECT_EQ(parallel.get_success_count(), 3);
}

TEST_F(TreeDocumentModelTemplateTest, SetNameViaModel)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.setName("my_sequence");

  EXPECT_EQ(seq.getName(), "my_sequence");
}

TEST_F(TreeDocumentModelTemplateTest, SetPortsViaBaseMethod)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto repeat = seq.insertNode<model::Repeat>();
  repeat.setPorts({{"num_cycles", "7"}});

  EXPECT_EQ(repeat.get_num_cycles(), 7);
}

TEST_F(TreeDocumentModelTemplateTest, ResetPortsViaModel)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto repeat = seq.insertNode<model::Repeat>();
  repeat.set_num_cycles(5);

  repeat.resetPorts();

  // After reset, the port should have its default value (or be empty)
  auto ports = repeat.getPorts();
  EXPECT_TRUE(ports.find("num_cycles") == ports.end() || ports.at("num_cycles").empty());
}

// =============================================================================
// getFirstNode<T> Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeByModelType)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();
  seq.insertNode<model::AlwaysFailure>();

  auto found = seq.getFirstNode<model::AlwaysSuccess>();
  EXPECT_EQ(found.getRegistrationName(), "AlwaysSuccess");
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeByModelTypeReturnsFirst)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto first = seq.insertNode<model::AlwaysSuccess>();
  first.setName("first_instance");
  auto second = seq.insertNode<model::AlwaysSuccess>();
  second.setName("second_instance");

  auto found = seq.getFirstNode<model::AlwaysSuccess>();
  EXPECT_EQ(found.getName(), "first_instance");
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeByModelTypeAndInstanceName)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto first = seq.insertNode<model::AlwaysSuccess>();
  first.setName("first_instance");
  auto second = seq.insertNode<model::AlwaysSuccess>();
  second.setName("second_instance");

  auto found = seq.getFirstNode<model::AlwaysSuccess>("second_instance");
  EXPECT_EQ(found.getName(), "second_instance");
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeByModelTypeThrowsWhenNotFound)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();

  EXPECT_THROW(seq.getFirstNode<model::AlwaysFailure>(), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeByModelTypeShallowDoesNotFindGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode<model::Sequence>();
  auto inner = outer.insertNode<model::Fallback>();
  inner.insertNode<model::AlwaysSuccess>();

  // AlwaysSuccess is a grandchild — shallow search should not find it
  EXPECT_THROW(outer.getFirstNode<model::AlwaysSuccess>(), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeByModelTypeDeepSearchFindsGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode<model::Sequence>();
  auto inner = outer.insertNode<model::Fallback>();
  inner.insertNode<model::AlwaysSuccess>();

  auto found = outer.getFirstNode<model::AlwaysSuccess>("", true);
  EXPECT_EQ(found.getRegistrationName(), "AlwaysSuccess");
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeByModelTypeDeepSearchReturnsFirstInOrder)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode<model::Sequence>();
  auto inner = outer.insertNode<model::Fallback>();
  auto deep = inner.insertNode<model::AlwaysSuccess>();
  deep.setName("deep");
  auto shallow = outer.insertNode<model::AlwaysSuccess>();
  shallow.setName("shallow");

  auto found = outer.getFirstNode<model::AlwaysSuccess>("", true);
  EXPECT_EQ(found.getName(), "deep");
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeReturnsCorrectModelType)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto repeat = seq.insertNode<model::Repeat>();
  repeat.set_num_cycles(42);

  // getFirstNode<model::Repeat> should return a model::Repeat, not just NodeElement
  model::Repeat found = seq.getFirstNode<model::Repeat>();
  EXPECT_EQ(found.get_num_cycles(), 42);
}

TEST_F(TreeDocumentModelTemplateTest, GetFirstNodeOnTreeElement)
{
  auto tree = doc_->newTree("TestTree");
  tree.insertNode<model::Sequence>();

  auto found = tree.getFirstNode<model::Sequence>();
  EXPECT_EQ(found.getRegistrationName(), "Sequence");
}

// =============================================================================
// removeFirstChild<T> Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildByModelType)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();
  seq.insertNode<model::AlwaysFailure>();

  seq.removeFirstChild<model::AlwaysSuccess>();

  std::vector<std::string> names;
  for (auto child : seq) {
    names.push_back(child.getRegistrationName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "AlwaysFailure");
}

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildByModelTypeRemovesFirst)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto first = seq.insertNode<model::AlwaysSuccess>();
  first.setName("first");
  auto second = seq.insertNode<model::AlwaysSuccess>();
  second.setName("second");

  seq.removeFirstChild<model::AlwaysSuccess>();

  auto remaining = seq.getFirstNode<model::AlwaysSuccess>();
  EXPECT_EQ(remaining.getName(), "second");
}

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildByModelTypeAndInstanceName)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto first = seq.insertNode<model::AlwaysSuccess>();
  first.setName("keep");
  auto second = seq.insertNode<model::AlwaysSuccess>();
  second.setName("remove_me");

  seq.removeFirstChild<model::AlwaysSuccess>("remove_me");

  auto remaining = seq.getFirstNode<model::AlwaysSuccess>();
  EXPECT_EQ(remaining.getName(), "keep");
}

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildByModelTypeThrowsWhenNotFound)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();

  EXPECT_THROW(seq.removeFirstChild<model::AlwaysFailure>(), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildByModelTypeShallowDoesNotRemoveGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode<model::Sequence>();
  auto inner = outer.insertNode<model::Fallback>();
  inner.insertNode<model::AlwaysSuccess>();

  EXPECT_THROW(outer.removeFirstChild<model::AlwaysSuccess>(), exceptions::TreeDocumentError);
  EXPECT_TRUE(inner.hasChildren());
}

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildByModelTypeDeepSearchRemovesGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode<model::Sequence>();
  auto inner = outer.insertNode<model::Fallback>();
  inner.insertNode<model::AlwaysSuccess>();

  outer.removeFirstChild<model::AlwaysSuccess>("", true);

  EXPECT_FALSE(inner.hasChildren());
  EXPECT_TRUE(outer.hasChildren());  // inner Fallback is still there
}

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildByModelTypeDeepSearchRemovesFirstInOrder)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode<model::Sequence>();
  auto inner = outer.insertNode<model::Fallback>();
  auto deep = inner.insertNode<model::AlwaysSuccess>();
  deep.setName("deep");
  auto shallow = outer.insertNode<model::AlwaysSuccess>();
  shallow.setName("shallow");

  outer.removeFirstChild<model::AlwaysSuccess>("", true);

  EXPECT_FALSE(inner.hasChildren());
  auto remaining = outer.getFirstNode<model::AlwaysSuccess>();
  EXPECT_EQ(remaining.getName(), "shallow");
}

TEST_F(TreeDocumentModelTemplateTest, RemoveFirstChildReturnsSelfForChaining)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();
  seq.insertNode<model::AlwaysFailure>();

  auto & ref = seq.removeFirstChild<model::AlwaysSuccess>();
  EXPECT_EQ(&ref, &seq);
}

TEST_F(TreeDocumentModelTemplateTest, TreeElementRemoveFirstChildByModelType)
{
  auto tree = doc_->newTree("TestTree");
  tree.insertNode<model::Sequence>();

  tree.removeFirstChild<model::Sequence>();

  EXPECT_FALSE(tree.hasChildren());
}

TEST_F(TreeDocumentModelTemplateTest, TreeElementRemoveFirstChildByModelTypeDeepSearch)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();
  seq.insertNode<model::AlwaysFailure>();

  tree.removeFirstChild<model::AlwaysSuccess>("", true);

  std::vector<std::string> names;
  for (auto child : seq) {
    names.push_back(child.getRegistrationName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "AlwaysFailure");
}

TEST_F(TreeDocumentModelTemplateTest, TreeElementRemoveFirstChildReturnsSelf)
{
  auto tree = doc_->newTree("TestTree");
  tree.insertNode<model::Sequence>();

  auto & ref = tree.removeFirstChild<model::Sequence>();
  EXPECT_EQ(&ref, &tree);
}

// =============================================================================
// toNodeElement Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, ModelToNodeElement)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();

  TreeDocument::NodeElement ele = seq.toNodeElement();
  EXPECT_EQ(ele.getRegistrationName(), "Sequence");
}

TEST_F(TreeDocumentModelTemplateTest, ModelToNodeElementWithPorts)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto repeat = seq.insertNode<model::Repeat>();
  repeat.set_num_cycles(5);

  TreeDocument::NodeElement ele = repeat.toNodeElement();
  auto ports = ele.getPorts();
  EXPECT_EQ(ports.at("num_cycles"), "5");
}

// =============================================================================
// Static Method Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, ModelStaticNameMethod)
{
  EXPECT_EQ(model::Sequence::name(), "Sequence");
  EXPECT_EQ(model::Fallback::name(), "Fallback");
  EXPECT_EQ(model::Inverter::name(), "Inverter");
  EXPECT_EQ(model::AlwaysSuccess::name(), "AlwaysSuccess");
  EXPECT_EQ(model::AlwaysFailure::name(), "AlwaysFailure");
  EXPECT_EQ(model::Repeat::name(), "Repeat");
}

TEST_F(TreeDocumentModelTemplateTest, ModelStaticTypeMethod)
{
  EXPECT_EQ(model::Sequence::type(), BT::NodeType::CONTROL);
  EXPECT_EQ(model::Fallback::type(), BT::NodeType::CONTROL);
  EXPECT_EQ(model::Inverter::type(), BT::NodeType::DECORATOR);
  EXPECT_EQ(model::AlwaysSuccess::type(), BT::NodeType::ACTION);
  EXPECT_EQ(model::AlwaysFailure::type(), BT::NodeType::ACTION);
  EXPECT_EQ(model::Repeat::type(), BT::NodeType::DECORATOR);
}

// =============================================================================
// Conditional Script via Model Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, SetConditionalScriptViaModel)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  auto success = seq.insertNode<model::AlwaysSuccess>();

  success.setConditionalScript(BT::PreCond::FAILURE_IF, Script("port_val == true"));

  // Verify the node is still valid after setting the script
  EXPECT_EQ(success.getRegistrationName(), "AlwaysSuccess");
}

// =============================================================================
// Mixed Template and Non-Template API Tests
// =============================================================================

TEST_F(TreeDocumentModelTemplateTest, InsertWithTemplateGetWithString)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();

  // Use non-template getFirstNode with string name
  auto found = seq.getFirstNode("AlwaysSuccess");
  EXPECT_EQ(found.getRegistrationName(), "AlwaysSuccess");
}

TEST_F(TreeDocumentModelTemplateTest, InsertWithStringGetWithTemplate)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode("AlwaysSuccess");

  // Use template getFirstNode
  auto found = seq.getFirstNode<model::AlwaysSuccess>();
  EXPECT_EQ(found.getRegistrationName(), "AlwaysSuccess");
}

TEST_F(TreeDocumentModelTemplateTest, InsertWithTemplateRemoveWithString)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode<model::AlwaysSuccess>();
  seq.insertNode<model::AlwaysFailure>();

  seq.removeFirstChild("AlwaysSuccess");

  std::vector<std::string> names;
  for (auto child : seq) {
    names.push_back(child.getRegistrationName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "AlwaysFailure");
}

TEST_F(TreeDocumentModelTemplateTest, InsertWithStringRemoveWithTemplate)
{
  auto tree = doc_->newTree("TestTree");
  auto seq = tree.insertNode<model::Sequence>();
  seq.insertNode("AlwaysSuccess");
  seq.insertNode("AlwaysFailure");

  seq.removeFirstChild<model::AlwaysSuccess>();

  std::vector<std::string> names;
  for (auto child : seq) {
    names.push_back(child.getRegistrationName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "AlwaysFailure");
}
