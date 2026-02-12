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

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "testable_tree_document.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// =============================================================================
// Test Fixture
// =============================================================================

class NodeElementChildIteratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    doc_ = std::make_unique<TestableTreeDocument>();
    doc_->addTestNode("TestAction1");
    doc_->addTestNode("TestAction2");
    doc_->addTestNode("TestAction3");
  }

  std::unique_ptr<TestableTreeDocument> doc_;
};

// =============================================================================
// ChildIterator Tests
// =============================================================================

TEST_F(NodeElementChildIteratorTest, EmptyNodeBeginEqualsEnd)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");

  // A node with no children should have begin() == end()
  EXPECT_EQ(sequence.begin(), sequence.end());
}

TEST_F(NodeElementChildIteratorTest, EmptyTreeBeginEqualsEnd)
{
  auto tree = doc_->newTree("TestTree");

  // An empty tree element should have begin() == end()
  EXPECT_EQ(tree.begin(), tree.end());
}

TEST_F(NodeElementChildIteratorTest, NonEmptyNodeBeginNotEqualsEnd)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  EXPECT_NE(sequence.begin(), sequence.end());
}

TEST_F(NodeElementChildIteratorTest, IterateSingleChild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  std::vector<std::string> names;
  for (auto child : sequence) {
    names.push_back(child.getRegistrationName());
  }

  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "TestAction1");
}

TEST_F(NodeElementChildIteratorTest, IterateMultipleChildren)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");
  sequence.insertNode("TestAction3");

  std::vector<std::string> names;
  for (auto child : sequence) {
    names.push_back(child.getRegistrationName());
  }

  ASSERT_EQ(names.size(), 3u);
  EXPECT_EQ(names[0], "TestAction1");
  EXPECT_EQ(names[1], "TestAction2");
  EXPECT_EQ(names[2], "TestAction3");
}

TEST_F(NodeElementChildIteratorTest, IterateFirstLevelOnly)
{
  auto tree = doc_->newTree("TestTree");
  auto outer_seq = tree.insertNode("Sequence");
  auto inner_seq = outer_seq.insertNode("Sequence");
  inner_seq.insertNode("TestAction1");
  inner_seq.insertNode("TestAction2");
  outer_seq.insertNode("TestAction3");

  // Iterating over outer_seq should only yield the inner Sequence and TestAction3,
  // NOT the children of inner Sequence
  std::vector<std::string> names;
  for (auto child : outer_seq) {
    names.push_back(child.getRegistrationName());
  }

  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "Sequence");
  EXPECT_EQ(names[1], "TestAction3");
}

TEST_F(NodeElementChildIteratorTest, IterateTreeElement)
{
  auto tree = doc_->newTree("TestTree");
  tree.insertNode("Sequence");

  // TreeElement inherits from NodeElement, so begin()/end() should work
  std::vector<std::string> names;
  for (auto child : tree) {
    names.push_back(child.getRegistrationName());
  }

  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "Sequence");
}

TEST_F(NodeElementChildIteratorTest, PreIncrementOperator)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  auto it = sequence.begin();
  EXPECT_EQ((*it).getRegistrationName(), "TestAction1");

  auto & ref = ++it;
  EXPECT_EQ((*it).getRegistrationName(), "TestAction2");
  // Pre-increment returns reference to self
  EXPECT_EQ(&ref, &it);

  ++it;
  EXPECT_EQ(it, sequence.end());
}

TEST_F(NodeElementChildIteratorTest, PostIncrementOperator)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  auto it = sequence.begin();
  auto prev = it++;

  // prev should still point to the first child
  EXPECT_EQ((*prev).getRegistrationName(), "TestAction1");
  // it should have advanced
  EXPECT_EQ((*it).getRegistrationName(), "TestAction2");
}

TEST_F(NodeElementChildIteratorTest, EqualityBetweenIterators)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  auto it1 = sequence.begin();
  auto it2 = sequence.begin();

  EXPECT_EQ(it1, it2);
  EXPECT_FALSE(it1 != it2);
}

TEST_F(NodeElementChildIteratorTest, InequalityBetweenIterators)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  auto it1 = sequence.begin();
  auto it2 = sequence.begin();
  ++it2;

  EXPECT_NE(it1, it2);
  EXPECT_FALSE(it1 == it2);
}

TEST_F(NodeElementChildIteratorTest, DefaultConstructedIteratorsAreEqual)
{
  TreeDocument::NodeElement::ChildIterator it1;
  TreeDocument::NodeElement::ChildIterator it2;

  EXPECT_EQ(it1, it2);
}

TEST_F(NodeElementChildIteratorTest, DefaultConstructedIteratorEqualsEnd)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");

  TreeDocument::NodeElement::ChildIterator default_it;
  EXPECT_EQ(default_it, sequence.end());
}

TEST_F(NodeElementChildIteratorTest, IteratorPreservesChildIdentity)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  auto child = sequence.insertNode("TestAction1");
  child.setName("my_instance");

  auto it = sequence.begin();
  auto iterated_child = *it;

  EXPECT_EQ(iterated_child.getRegistrationName(), child.getRegistrationName());
  EXPECT_EQ(iterated_child.getName(), "my_instance");
  EXPECT_EQ(iterated_child, child);
}

TEST_F(NodeElementChildIteratorTest, IterateConstNodeElement)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  // Obtain a const reference
  const TreeDocument::NodeElement & const_seq = sequence;

  std::vector<std::string> names;
  for (auto child : const_seq) {
    names.push_back(child.getRegistrationName());
  }

  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "TestAction1");
  EXPECT_EQ(names[1], "TestAction2");
}

TEST_F(NodeElementChildIteratorTest, ManualIteratorLoop)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");
  sequence.insertNode("TestAction3");

  // Classic iterator loop
  int count = 0;
  for (auto it = sequence.begin(); it != sequence.end(); ++it) {
    ++count;
  }

  EXPECT_EQ(count, 3);
}

// =============================================================================
// getFirstNode Tests
// =============================================================================

class NodeElementGetFirstNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    doc_ = std::make_unique<TestableTreeDocument>();
    doc_->addTestNode("TestAction1");
    doc_->addTestNode("TestAction2");
    doc_->addTestNode("TestAction3");
  }

  std::unique_ptr<TestableTreeDocument> doc_;
};

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchFindsDirectChild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  auto found = sequence.getFirstNode("TestAction2");
  EXPECT_EQ(found.getRegistrationName(), "TestAction2");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchDoesNotFindGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  inner.insertNode("TestAction1");

  // TestAction1 is a grandchild of outer — default (shallow) search should not find it
  EXPECT_THROW(outer.getFirstNode("TestAction1"), exceptions::TreeDocumentError);
}

TEST_F(NodeElementGetFirstNodeTest, DeepSearchFindsGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  inner.insertNode("TestAction1");

  // With deep_search=true, grandchild should be found
  auto found = outer.getFirstNode("TestAction1", "", true);
  EXPECT_EQ(found.getRegistrationName(), "TestAction1");
}

TEST_F(NodeElementGetFirstNodeTest, DeepSearchFindsDirectChildToo)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  auto found = sequence.getFirstNode("TestAction1", "", true);
  EXPECT_EQ(found.getRegistrationName(), "TestAction1");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchByInstanceName)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  auto child = sequence.insertNode("TestAction1");
  child.setName("my_node");

  auto found = sequence.getFirstNode("", "my_node");
  EXPECT_EQ(found.getName(), "my_node");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchByInstanceNameDoesNotFindGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  auto grandchild = inner.insertNode("TestAction1");
  grandchild.setName("deep_node");

  EXPECT_THROW(outer.getFirstNode("", "deep_node"), exceptions::TreeDocumentError);
}

TEST_F(NodeElementGetFirstNodeTest, DeepSearchByInstanceNameFindsGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  auto grandchild = inner.insertNode("TestAction1");
  grandchild.setName("deep_node");

  auto found = outer.getFirstNode("", "deep_node", true);
  EXPECT_EQ(found.getName(), "deep_node");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchByRegistrationAndInstanceName)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  auto child1 = sequence.insertNode("TestAction1");
  child1.setName("first");
  auto child2 = sequence.insertNode("TestAction1");
  child2.setName("second");

  auto found = sequence.getFirstNode("TestAction1", "second");
  EXPECT_EQ(found.getName(), "second");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchNoArgsReturnsFirstChild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  // Both names empty: returns the first child regardless
  auto found = sequence.getFirstNode();
  EXPECT_EQ(found.getRegistrationName(), "TestAction1");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchThrowsWhenNotFound)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  EXPECT_THROW(sequence.getFirstNode("TestAction2"), exceptions::TreeDocumentError);
}

TEST_F(NodeElementGetFirstNodeTest, DeepSearchThrowsWhenNotFound)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  EXPECT_THROW(sequence.getFirstNode("TestAction2", "", true), exceptions::TreeDocumentError);
}

TEST_F(NodeElementGetFirstNodeTest, DeepSearchReturnsFirstInExecutionOrder)
{
  // Build a tree:
  //   outer_seq
  //     inner_seq
  //       TestAction1 (name="deep")
  //     TestAction1 (name="shallow")
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  auto deep = inner.insertNode("TestAction1");
  deep.setName("deep");
  auto shallow = outer.insertNode("TestAction1");
  shallow.setName("shallow");

  // Deep search follows execution order (depth-first), so "deep" comes first
  auto found = outer.getFirstNode("TestAction1", "", true);
  EXPECT_EQ(found.getName(), "deep");

  // Shallow search should find "shallow" since it's the only direct child with that registration name
  // (the inner Sequence is a direct child, but not a TestAction1)
  // Wait - actually inner is Sequence, and shallow is TestAction1. So shallow search finds shallow.
  auto found_shallow = outer.getFirstNode("TestAction1");
  EXPECT_EQ(found_shallow.getName(), "shallow");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchOnTreeElement)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");

  auto found = tree.getFirstNode("Sequence");
  EXPECT_EQ(found.getRegistrationName(), "Sequence");
}

TEST_F(NodeElementGetFirstNodeTest, DefaultSearchOnTreeElementDoesNotFindGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  // TestAction1 is a grandchild of the tree element — shallow search should not find it
  EXPECT_THROW(tree.getFirstNode("TestAction1"), exceptions::TreeDocumentError);
}

TEST_F(NodeElementGetFirstNodeTest, DeepSearchOnTreeElementFindsGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  auto found = tree.getFirstNode("TestAction1", "", true);
  EXPECT_EQ(found.getRegistrationName(), "TestAction1");
}

// =============================================================================
// removeFirstChild Tests
// =============================================================================

class NodeElementRemoveFirstChildTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    doc_ = std::make_unique<TestableTreeDocument>();
    doc_->addTestNode("TestAction1");
    doc_->addTestNode("TestAction2");
    doc_->addTestNode("TestAction3");
  }

  std::unique_ptr<TestableTreeDocument> doc_;
};

TEST_F(NodeElementRemoveFirstChildTest, DefaultRemovesDirectChild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  sequence.removeFirstChild("TestAction1");

  // Only TestAction2 should remain
  std::vector<std::string> names;
  for (auto child : sequence) {
    names.push_back(child.getRegistrationName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "TestAction2");
}

TEST_F(NodeElementRemoveFirstChildTest, DefaultDoesNotRemoveGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  inner.insertNode("TestAction1");

  // TestAction1 is a grandchild — shallow remove should throw
  EXPECT_THROW(outer.removeFirstChild("TestAction1"), exceptions::TreeDocumentError);

  // Verify nothing was removed
  EXPECT_TRUE(inner.hasChildren());
}

TEST_F(NodeElementRemoveFirstChildTest, DeepSearchRemovesGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  inner.insertNode("TestAction1");

  outer.removeFirstChild("TestAction1", "", true);

  // inner should now have no children
  EXPECT_FALSE(inner.hasChildren());
  // outer should still have inner
  EXPECT_TRUE(outer.hasChildren());
}

TEST_F(NodeElementRemoveFirstChildTest, DeepSearchRemovesFirstInExecutionOrder)
{
  // Build:
  //   outer
  //     inner
  //       TestAction1 (name="deep")
  //     TestAction1 (name="shallow")
  auto tree = doc_->newTree("TestTree");
  auto outer = tree.insertNode("Sequence");
  auto inner = outer.insertNode("Sequence");
  auto deep = inner.insertNode("TestAction1");
  deep.setName("deep");
  auto shallow = outer.insertNode("TestAction1");
  shallow.setName("shallow");

  // Deep remove should remove "deep" (depth-first order)
  outer.removeFirstChild("TestAction1", "", true);

  EXPECT_FALSE(inner.hasChildren());
  // "shallow" should still be there
  auto remaining = outer.getFirstNode("TestAction1");
  EXPECT_EQ(remaining.getName(), "shallow");
}

TEST_F(NodeElementRemoveFirstChildTest, DefaultRemovesByInstanceName)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  auto child1 = sequence.insertNode("TestAction1");
  child1.setName("keep");
  auto child2 = sequence.insertNode("TestAction2");
  child2.setName("remove_me");

  sequence.removeFirstChild("", "remove_me");

  std::vector<std::string> names;
  for (auto child : sequence) {
    names.push_back(child.getName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "keep");
}

TEST_F(NodeElementRemoveFirstChildTest, DefaultRemovesByRegistrationAndInstanceName)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  auto child1 = sequence.insertNode("TestAction1");
  child1.setName("first");
  auto child2 = sequence.insertNode("TestAction1");
  child2.setName("second");

  sequence.removeFirstChild("TestAction1", "second");

  std::vector<std::string> names;
  for (auto child : sequence) {
    names.push_back(child.getName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "first");
}

TEST_F(NodeElementRemoveFirstChildTest, DefaultThrowsWhenNotFound)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  EXPECT_THROW(sequence.removeFirstChild("TestAction2"), exceptions::TreeDocumentError);
}

TEST_F(NodeElementRemoveFirstChildTest, DeepSearchThrowsWhenNotFound)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  EXPECT_THROW(sequence.removeFirstChild("TestAction2", "", true), exceptions::TreeDocumentError);
}

TEST_F(NodeElementRemoveFirstChildTest, DefaultRemoveNoArgsRemovesFirstChild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  sequence.removeFirstChild();

  std::vector<std::string> names;
  for (auto child : sequence) {
    names.push_back(child.getRegistrationName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "TestAction2");
}

TEST_F(NodeElementRemoveFirstChildTest, ReturnsSelfForChaining)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  auto & ref = sequence.removeFirstChild("TestAction1");
  EXPECT_EQ(&ref, &sequence);
}

TEST_F(NodeElementRemoveFirstChildTest, TreeElementRemoveFirstChildDefaultShallow)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  // Sequence is a direct child of tree — removable with shallow search
  tree.removeFirstChild("Sequence");

  EXPECT_FALSE(tree.hasChildren());
}

TEST_F(NodeElementRemoveFirstChildTest, TreeElementRemoveFirstChildDoesNotFindGrandchild)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");

  // TestAction1 is a grandchild of tree — shallow remove should throw
  EXPECT_THROW(tree.removeFirstChild("TestAction1"), exceptions::TreeDocumentError);
}

TEST_F(NodeElementRemoveFirstChildTest, TreeElementRemoveFirstChildDeepSearch)
{
  auto tree = doc_->newTree("TestTree");
  auto sequence = tree.insertNode("Sequence");
  sequence.insertNode("TestAction1");
  sequence.insertNode("TestAction2");

  tree.removeFirstChild("TestAction1", "", true);

  // TestAction1 should be gone, TestAction2 and the Sequence should remain
  EXPECT_TRUE(tree.hasChildren());
  std::vector<std::string> names;
  for (auto child : sequence) {
    names.push_back(child.getRegistrationName());
  }
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "TestAction2");
}
