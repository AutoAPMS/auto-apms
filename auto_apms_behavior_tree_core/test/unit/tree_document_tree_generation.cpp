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
#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"
#include "testable_tree_document.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// =============================================================================
// Test Fixture
// =============================================================================

class TreeDocumentTreeGenerationTest : public ::testing::Test
{
protected:
  void SetUp() override { doc_ = std::make_unique<TreeDocument>(); }

  // Helper to create a simple tree XML string
  static std::string makeSimpleTreeXml(const std::string & tree_name, const std::string & node_name = "Sequence")
  {
    return R"(<root BTCPP_format="4" main_tree_to_execute=")" + tree_name + R"(">
      <BehaviorTree ID=")" +
           tree_name + R"(">
        <)" +
           node_name + R"(/>
      </BehaviorTree>
    </root>)";
  }

  // Helper to create tree XML with multiple nodes
  static std::string makeTreeWithNodes(const std::string & tree_name, const std::vector<std::string> & node_names)
  {
    std::string nodes;
    for (const auto & name : node_names) {
      nodes += "<" + name + "/>";
    }
    return R"(<root BTCPP_format="4" main_tree_to_execute=")" + tree_name + R"(">
      <BehaviorTree ID=")" +
           tree_name + R"(">
        <Sequence>)" +
           nodes + R"(</Sequence>
      </BehaviorTree>
    </root>)";
  }

  std::unique_ptr<TreeDocument> doc_;
};

// =============================================================================
// newTree Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, NewTreeCreatesEmptyTree)
{
  auto tree = doc_->newTree("TestTree");

  EXPECT_EQ(tree.getName(), "TestTree");
  EXPECT_TRUE(doc_->hasTreeName("TestTree"));
  EXPECT_EQ(doc_->getAllTreeNames().size(), 1u);
}

TEST_F(TreeDocumentTreeGenerationTest, NewTreeWithEmptyNameThrows)
{
  EXPECT_THROW(doc_->newTree(""), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentTreeGenerationTest, NewTreeDuplicateNameThrows)
{
  doc_->newTree("TestTree");

  EXPECT_THROW(doc_->newTree("TestTree"), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentTreeGenerationTest, NewTreeMultipleTrees)
{
  doc_->newTree("Tree1");
  doc_->newTree("Tree2");
  doc_->newTree("Tree3");

  EXPECT_EQ(doc_->getAllTreeNames().size(), 3u);
  EXPECT_TRUE(doc_->hasTreeName("Tree1"));
  EXPECT_TRUE(doc_->hasTreeName("Tree2"));
  EXPECT_TRUE(doc_->hasTreeName("Tree3"));
}

// =============================================================================
// mergeString Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, MergeStringSingleTree)
{
  doc_->mergeString(makeSimpleTreeXml("MergedTree"));

  EXPECT_TRUE(doc_->hasTreeName("MergedTree"));
}

TEST_F(TreeDocumentTreeGenerationTest, MergeStringAdoptRootTree)
{
  doc_->newTree("ExistingTree").makeRoot();
  EXPECT_EQ(doc_->getRootTreeName(), "ExistingTree");

  doc_->mergeString(makeSimpleTreeXml("NewRoot"), true);

  EXPECT_EQ(doc_->getRootTreeName(), "NewRoot");
}

TEST_F(TreeDocumentTreeGenerationTest, MergeStringWithoutAdoptRootTree)
{
  doc_->newTree("ExistingTree").makeRoot();
  EXPECT_EQ(doc_->getRootTreeName(), "ExistingTree");

  doc_->mergeString(makeSimpleTreeXml("AnotherTree"), false);

  EXPECT_EQ(doc_->getRootTreeName(), "ExistingTree");
  EXPECT_TRUE(doc_->hasTreeName("AnotherTree"));
}

TEST_F(TreeDocumentTreeGenerationTest, MergeStringDuplicateTreeNameThrows)
{
  doc_->newTree("TestTree");

  EXPECT_THROW(doc_->mergeString(makeSimpleTreeXml("TestTree")), exceptions::TreeDocumentError);
}

// =============================================================================
// newTreeFromString Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, NewTreeFromStringCreatesTree)
{
  auto tree = doc_->newTreeFromString(makeSimpleTreeXml("SourceTree"));

  EXPECT_EQ(tree.getName(), "SourceTree");
  EXPECT_TRUE(doc_->hasTreeName("SourceTree"));
}

TEST_F(TreeDocumentTreeGenerationTest, NewTreeFromStringWithSpecificTreeName)
{
  std::string xml = R"(<root BTCPP_format="4">
    <BehaviorTree ID="Tree1"><Sequence/></BehaviorTree>
    <BehaviorTree ID="Tree2"><Fallback/></BehaviorTree>
  </root>)";

  auto tree = doc_->newTreeFromString(xml, "Tree2");

  EXPECT_EQ(tree.getName(), "Tree2");
  EXPECT_TRUE(doc_->hasTreeName("Tree2"));
  // Only Tree2 should be created, not Tree1
  EXPECT_FALSE(doc_->hasTreeName("Tree1"));
}

// =============================================================================
// mergeTree Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, MergeTreeFromAnotherDocument)
{
  TreeDocument other_doc;
  other_doc.mergeString(makeSimpleTreeXml("OtherTree"));

  doc_->mergeTree(other_doc.getTree("OtherTree"));

  EXPECT_TRUE(doc_->hasTreeName("OtherTree"));
}

TEST_F(TreeDocumentTreeGenerationTest, MergeTreeMakesRoot)
{
  doc_->newTree("ExistingTree").makeRoot();

  TreeDocument other_doc;
  other_doc.mergeString(makeSimpleTreeXml("NewRoot"));

  doc_->mergeTree(other_doc.getTree("NewRoot"), true);

  EXPECT_EQ(doc_->getRootTreeName(), "NewRoot");
}

// =============================================================================
// Tree Element Operations
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, TreeElementSetName)
{
  auto tree = doc_->newTree("OriginalName");
  tree.setName("NewName");

  EXPECT_EQ(tree.getName(), "NewName");
  EXPECT_TRUE(doc_->hasTreeName("NewName"));
}

TEST_F(TreeDocumentTreeGenerationTest, TreeElementMakeRoot)
{
  doc_->newTree("Tree1");
  auto tree2 = doc_->newTree("Tree2");

  tree2.makeRoot();

  EXPECT_EQ(doc_->getRootTreeName(), "Tree2");
}

TEST_F(TreeDocumentTreeGenerationTest, GetRootTree)
{
  doc_->newTree("Tree1");
  doc_->newTree("Tree2").makeRoot();

  auto root = doc_->getRootTree();

  EXPECT_EQ(root.getName(), "Tree2");
}

// =============================================================================
// removeTree Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, RemoveTreeByName)
{
  doc_->newTree("Tree1");
  doc_->newTree("Tree2");

  doc_->removeTree("Tree1");

  EXPECT_FALSE(doc_->hasTreeName("Tree1"));
  EXPECT_TRUE(doc_->hasTreeName("Tree2"));
}

TEST_F(TreeDocumentTreeGenerationTest, RemoveTreeByElement)
{
  auto tree = doc_->newTree("TreeToRemove");
  doc_->newTree("TreeToKeep");

  doc_->removeTree(tree);

  EXPECT_FALSE(doc_->hasTreeName("TreeToRemove"));
  EXPECT_TRUE(doc_->hasTreeName("TreeToKeep"));
}

TEST_F(TreeDocumentTreeGenerationTest, RemoveNonExistentTreeThrows)
{
  EXPECT_THROW(doc_->removeTree("NonExistent"), exceptions::TreeDocumentError);
}

// =============================================================================
// setRootTreeName Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, SetRootTreeName)
{
  doc_->newTree("Tree1");
  doc_->newTree("Tree2");

  doc_->setRootTreeName("Tree2");

  EXPECT_EQ(doc_->getRootTreeName(), "Tree2");
}

TEST_F(TreeDocumentTreeGenerationTest, SetRootTreeNameNonExistentThrows)
{
  doc_->newTree("ExistingTree");

  EXPECT_THROW(doc_->setRootTreeName("NonExistent"), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentTreeGenerationTest, SetRootTreeNameEmptyThrows)
{
  EXPECT_THROW(doc_->setRootTreeName(""), exceptions::TreeDocumentError);
}

// =============================================================================
// hasRootTreeName Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, HasRootTreeNameFalseInitially)
{
  doc_->newTree("Tree1");

  EXPECT_FALSE(doc_->hasRootTreeName());
}

TEST_F(TreeDocumentTreeGenerationTest, HasRootTreeNameTrueAfterSetting)
{
  doc_->newTree("Tree1").makeRoot();

  EXPECT_TRUE(doc_->hasRootTreeName());
}

// =============================================================================
// getAllTreeNames Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, GetAllTreeNamesEmpty) { EXPECT_TRUE(doc_->getAllTreeNames().empty()); }

TEST_F(TreeDocumentTreeGenerationTest, GetAllTreeNamesMultiple)
{
  doc_->newTree("Alpha");
  doc_->newTree("Beta");
  doc_->newTree("Gamma");

  auto names = doc_->getAllTreeNames();

  EXPECT_EQ(names.size(), 3u);
  EXPECT_TRUE(std::find(names.begin(), names.end(), "Alpha") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "Beta") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "Gamma") != names.end());
}

// =============================================================================
// applyNodeNamespace Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, ApplyNodeNamespaceEmptyDocument)
{
  // Should not throw on empty document
  ASSERT_NO_THROW(doc_->applyNodeNamespace("ns"));
}

TEST_F(TreeDocumentTreeGenerationTest, ApplyNodeNamespaceDoesNotAffectNativeNodes)
{
  // Don't register any custom nodes
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Sequence>
        <Fallback>
          <AlwaysSuccess/>
        </Fallback>
      </Sequence>
    </BehaviorTree>
  </root>)");

  doc_->applyNodeNamespace("ns");

  std::string xml = doc_->writeToString();
  // Native nodes should remain unchanged
  EXPECT_TRUE(xml.find("<Sequence") != std::string::npos);
  EXPECT_TRUE(xml.find("<Fallback") != std::string::npos);
  EXPECT_TRUE(xml.find("<AlwaysSuccess") != std::string::npos);
  // Should NOT have namespaced versions
  EXPECT_TRUE(xml.find("ns.Sequence") == std::string::npos);
  EXPECT_TRUE(xml.find("ns.Fallback") == std::string::npos);
}

TEST_F(TreeDocumentTreeGenerationTest, ApplyNodeNamespacePreservesTreeStructure)
{
  // Ensure tree structure is preserved after applying namespace
  doc_->mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Sequence>
        <AlwaysSuccess/>
        <AlwaysFailure/>
      </Sequence>
    </BehaviorTree>
  </root>)",
    true);  // adopt_root_tree = true

  doc_->applyNodeNamespace("robot");

  // Tree names and root tree should be preserved
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_EQ(doc_->getRootTreeName(), "MainTree");

  // Document should still be valid
  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

TEST_F(TreeDocumentTreeGenerationTest, ApplyNodeNamespaceWithEmptyNamespace)
{
  // Empty namespace should still work (just prepends separator)
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Sequence/>
    </BehaviorTree>
  </root>)");

  ASSERT_NO_THROW(doc_->applyNodeNamespace(""));
}

TEST_F(TreeDocumentTreeGenerationTest, ApplyNodeNamespaceReturnsSelf)
{
  // Method chaining should work
  auto & result = doc_->applyNodeNamespace("ns");
  EXPECT_EQ(&result, doc_.get());
}

// =============================================================================
// applyNodeNamespace Tests with TestableTreeDocument
// =============================================================================

TEST(TestableTreeDocumentTest, ApplyNodeNamespaceRenamesManifestEntries)
{
  TestableTreeDocument doc;
  doc.addTestNode("MoveAction", "test::MoveAction");
  doc.addTestNode("CheckBattery", "test::CheckBattery");

  // Verify nodes are registered before applying namespace
  auto before = doc.getTestNodeNames();
  EXPECT_EQ(before.count("MoveAction"), 1u);
  EXPECT_EQ(before.count("CheckBattery"), 1u);

  doc.applyNodeNamespace("robot1");

  // Verify nodes are renamed after applying namespace
  auto after = doc.getTestNodeNames();
  EXPECT_EQ(after.count("robot1.MoveAction"), 1u);
  EXPECT_EQ(after.count("robot1.CheckBattery"), 1u);
  EXPECT_EQ(after.count("MoveAction"), 0u);
  EXPECT_EQ(after.count("CheckBattery"), 0u);
}

TEST(TestableTreeDocumentTest, ApplyNodeNamespaceRenamesXmlElements)
{
  TestableTreeDocument doc;
  doc.addTestNode("MyAction", "test::MyAction");

  // Create tree using the registered node
  doc.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <MyAction/>
    </BehaviorTree>
  </root>)",
    true);

  doc.applyNodeNamespace("ns");

  std::string xml = doc.writeToString();
  // The node element should be renamed
  EXPECT_TRUE(xml.find("<ns.MyAction") != std::string::npos);
  EXPECT_TRUE(xml.find("<MyAction") == std::string::npos);
}

TEST(TestableTreeDocumentTest, ApplyNodeNamespaceWithCustomSeparator)
{
  TestableTreeDocument doc;
  doc.addTestNode("TestNode", "test::TestNode");

  doc.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <TestNode/>
    </BehaviorTree>
  </root>)",
    true);

  doc.applyNodeNamespace("prefix", "::");

  std::string xml = doc.writeToString();
  EXPECT_TRUE(xml.find("<prefix::TestNode") != std::string::npos);

  auto names = doc.getTestNodeNames();
  EXPECT_EQ(names.count("prefix::TestNode"), 1u);
}

TEST(TestableTreeDocumentTest, ApplyNodeNamespaceNestedNodes)
{
  TestableTreeDocument doc;
  doc.addTestNode("Action1", "test::Action1");
  doc.addTestNode("Action2", "test::Action2");

  doc.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Sequence>
        <Action1/>
        <Fallback>
          <Action2/>
        </Fallback>
      </Sequence>
    </BehaviorTree>
  </root>)",
    true);

  doc.applyNodeNamespace("robot");

  std::string xml = doc.writeToString();
  // Custom nodes should be renamed
  EXPECT_TRUE(xml.find("<robot.Action1") != std::string::npos);
  EXPECT_TRUE(xml.find("<robot.Action2") != std::string::npos);
  // Native nodes should NOT be renamed
  EXPECT_TRUE(xml.find("<Sequence") != std::string::npos);
  EXPECT_TRUE(xml.find("<Fallback") != std::string::npos);
}

TEST(TestableTreeDocumentTest, ApplyNodeNamespaceMultipleTrees)
{
  TestableTreeDocument doc;
  doc.addTestNode("SharedAction", "test::SharedAction");

  doc.mergeString(R"(<root BTCPP_format="4">
    <BehaviorTree ID="Tree1">
      <SharedAction/>
    </BehaviorTree>
    <BehaviorTree ID="Tree2">
      <Sequence>
        <SharedAction/>
      </Sequence>
    </BehaviorTree>
  </root>)");

  doc.applyNodeNamespace("ns");

  std::string xml = doc.writeToString();
  // Count occurrences - both instances should be renamed
  size_t count = 0;
  size_t pos = 0;
  while ((pos = xml.find("<ns.SharedAction", pos)) != std::string::npos) {
    ++count;
    ++pos;
  }
  EXPECT_EQ(count, 2u);

  // Original name should not exist
  EXPECT_TRUE(xml.find("<SharedAction") == std::string::npos);
}

TEST(TestableTreeDocumentTest, ApplyNodeNamespacePreservesNodeAttributes)
{
  TestableTreeDocument doc;
  doc.addTestNode("ConfigurableAction", "test::ConfigurableAction");

  doc.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <ConfigurableAction name="my_instance" some_port="value"/>
    </BehaviorTree>
  </root>)",
    true);

  doc.applyNodeNamespace("ns");

  std::string xml = doc.writeToString();
  // Node should be renamed but attributes preserved
  EXPECT_TRUE(xml.find("<ns.ConfigurableAction") != std::string::npos);
  EXPECT_TRUE(xml.find("name=\"my_instance\"") != std::string::npos);
  EXPECT_TRUE(xml.find("some_port=\"value\"") != std::string::npos);
}

// =============================================================================
// writeToString Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, WriteToStringEmptyDocument)
{
  std::string xml = doc_->writeToString();

  EXPECT_FALSE(xml.empty());
  EXPECT_TRUE(xml.find("BTCPP_format") != std::string::npos);
}

TEST_F(TreeDocumentTreeGenerationTest, WriteToStringWithTrees)
{
  doc_->newTree("TestTree");

  std::string xml = doc_->writeToString();

  EXPECT_TRUE(xml.find("TestTree") != std::string::npos);
  EXPECT_TRUE(xml.find("BehaviorTree") != std::string::npos);
}

// =============================================================================
// verify Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, VerifyEmptyDocumentSucceeds)
{
  auto result = doc_->verify();

  EXPECT_TRUE(result);
}

// =============================================================================
// Method Chaining Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, MethodChainingMergeAndSetRoot)
{
  doc_->mergeString(makeSimpleTreeXml("Tree1")).mergeString(makeSimpleTreeXml("Tree2")).setRootTreeName("Tree2");

  EXPECT_TRUE(doc_->hasTreeName("Tree1"));
  EXPECT_TRUE(doc_->hasTreeName("Tree2"));
  EXPECT_EQ(doc_->getRootTreeName(), "Tree2");
}

// =============================================================================
// insertSubTreeNode Tests
// =============================================================================

TEST_F(TreeDocumentTreeGenerationTest, InsertSubTreeNodeByName)
{
  // Create a tree first
  doc_->newTree("SubTree");
  auto main_tree = doc_->newTree("MainTree").makeRoot();

  // Insert a subtree node that references the existing tree
  auto sequence = main_tree.insertNode("Sequence");
  sequence.insertSubTreeNode("SubTree");

  std::string xml = doc_->writeToString();
  // SubTree nodes use ID attribute to reference the tree name
  EXPECT_TRUE(xml.find("SubTree") != std::string::npos) << "XML:\n" << xml;
}

TEST_F(TreeDocumentTreeGenerationTest, InsertSubTreeNodeByNameNonExistentThrows)
{
  auto main_tree = doc_->newTree("MainTree").makeRoot();
  auto sequence = main_tree.insertNode("Sequence");

  EXPECT_THROW(sequence.insertSubTreeNode("NonExistent"), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentTreeGenerationTest, InsertSubTreeNodeByTreeElement)
{
  // Create source document with subtree - must have at least one child node
  TreeDocument other_doc;
  other_doc.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="SubTree">
    <BehaviorTree ID="SubTree">
      <AlwaysSuccess/>
    </BehaviorTree>
  </root>)",
    true);

  // Create main document and insert subtree from other document
  auto main_tree = doc_->newTree("MainTree").makeRoot();
  auto sequence = main_tree.insertNode("Sequence");
  sequence.insertSubTreeNode(other_doc.getRootTree());

  // The subtree should be merged into the main document
  EXPECT_TRUE(doc_->hasTreeName("SubTree"));

  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("SubTree") != std::string::npos) << "XML:\n" << xml;
}

TEST_F(TreeDocumentTreeGenerationTest, InsertSubTreeNodeFromExternalDocumentMultipleTimes)
{
  // Test inserting the same external subtree multiple times
  TreeDocument other_doc;
  other_doc.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="ExternalTree">
    <BehaviorTree ID="ExternalTree">
      <Sequence>
        <AlwaysSuccess/>
      </Sequence>
    </BehaviorTree>
  </root>)",
    true);

  auto main_tree = doc_->newTree("MainTree").makeRoot();
  auto sequence = main_tree.insertNode("Sequence");

  auto external_subtree = other_doc.getRootTree();

  // Insert the same external subtree multiple times - should succeed
  ASSERT_NO_THROW(sequence.insertSubTreeNode(external_subtree));
  ASSERT_NO_THROW(sequence.insertSubTreeNode(external_subtree));

  EXPECT_TRUE(doc_->hasTreeName("ExternalTree"));

  std::string xml = doc_->writeToString();
  SCOPED_TRACE("XML:\n" + xml);
  // Count occurrences - should appear: 1 BehaviorTree ID + 2 SubTree IDs
  size_t count = 0;
  size_t pos = 0;
  while ((pos = xml.find("ExternalTree", pos)) != std::string::npos) {
    ++count;
    ++pos;
  }
  EXPECT_GE(count, 3u) << "Expected at least 3 occurrences of 'ExternalTree' in XML";
}

TEST_F(TreeDocumentTreeGenerationTest, InsertSubTreeNodeFromSameDocumentMultipleTimes)
{
  // Create subtree in same document
  doc_->newTree("SubTree");
  auto main_tree = doc_->newTree("MainTree").makeRoot();
  auto sequence = main_tree.insertNode("Sequence");

  // Insert same subtree multiple times - should work fine
  ASSERT_NO_THROW(sequence.insertSubTreeNode("SubTree"));
  ASSERT_NO_THROW(sequence.insertSubTreeNode("SubTree"));
  ASSERT_NO_THROW(sequence.insertSubTreeNode("SubTree"));

  std::string xml = doc_->writeToString();
  // Debug output
  SCOPED_TRACE("XML:\n" + xml);
  // Count occurrences - SubTree element with ID attribute
  size_t count = 0;
  size_t pos = 0;
  while ((pos = xml.find("SubTree", pos)) != std::string::npos) {
    ++count;
    ++pos;
  }
  // Should find at least 4: 1 BehaviorTree + 3 SubTree nodes
  EXPECT_GE(count, 4u) << "Expected at least 4 occurrences of 'SubTree' in XML";
}

TEST_F(TreeDocumentTreeGenerationTest, InsertSubTreeNodeDifferentTreesSameName)
{
  // Create two documents with trees that have the SAME name but DIFFERENT content
  TreeDocument doc1, doc2;
  doc1.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="SharedName">
    <BehaviorTree ID="SharedName">
      <AlwaysSuccess/>
    </BehaviorTree>
  </root>)",
    true);

  doc2.mergeString(
    R"(<root BTCPP_format="4" main_tree_to_execute="SharedName">
    <BehaviorTree ID="SharedName">
      <AlwaysFailure/>
    </BehaviorTree>
  </root>)",
    true);

  auto main_tree = doc_->newTree("MainTree").makeRoot();
  auto sequence = main_tree.insertNode("Sequence");

  // First insert should succeed
  ASSERT_NO_THROW(sequence.insertSubTreeNode(doc1.getRootTree()));
  EXPECT_TRUE(doc_->hasTreeName("SharedName"));

  // Second insert with different external tree but same name SHOULD throw
  // because the XML content is different
  EXPECT_THROW(sequence.insertSubTreeNode(doc2.getRootTree()), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentTreeGenerationTest, InsertSubTreeNodeWithTreeElementFromSameDocument)
{
  // Test using insertSubTreeNode(TreeElement) where the tree is from THIS document
  doc_->newTree("SubTree");
  auto sub_tree_element = doc_->getTree("SubTree");

  auto main_tree = doc_->newTree("MainTree").makeRoot();
  auto sequence = main_tree.insertNode("Sequence");

  // Should work - tree element is from same document
  ASSERT_NO_THROW(sequence.insertSubTreeNode(sub_tree_element));
  ASSERT_NO_THROW(sequence.insertSubTreeNode(sub_tree_element));

  std::string xml = doc_->writeToString();
  SCOPED_TRACE("XML:\n" + xml);
  // Count occurrences - should appear: 1 BehaviorTree ID + 2 SubTree ID refs
  size_t count = 0;
  size_t pos = 0;
  while ((pos = xml.find("SubTree", pos)) != std::string::npos) {
    ++count;
    ++pos;
  }
  EXPECT_GE(count, 3u) << "Expected at least 3 occurrences of 'SubTree' in XML";
}
