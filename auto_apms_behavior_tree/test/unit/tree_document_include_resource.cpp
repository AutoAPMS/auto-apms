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

// End-to-end tests for the <include autoapms="..."/> attribute of TreeDocument, which resolves an included file
// through the AutoAPMS tree resource system (as opposed to a filesystem path). Unlike the unit tests in
// auto_apms_behavior_tree_core (which can only cover argument validation and the failure path, since that package
// cannot register resources for itself), these tests run against real, installed tree resources registered under
// BUILD_TESTING:
//   - auto_apms_behavior_tree::test_tree::TestTree           (single AlwaysSuccess tree)
//   - auto_apms_behavior_tree::include_e2e::{E2ERoot, E2EHelperSuccess, E2EHelperSequence, E2EAlpha, E2EBeta,
//     E2ESharedLeaf}
//
// An autoapms include cherry-picks the individual tree named by the identity plus its transitive <SubTree> dependency
// closure (not the whole file), and a helper shared between two includes is merged only once.
//
// They verify the full round trip: resolve the resource identity, merge the referenced file, and then actually
// instantiate and tick the resulting behavior tree to completion via TreeBuilder.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "behaviortree_cpp/basic_types.h"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// Identity of the simple single-tree resource registered for the tests (see test/resource/test_tree.xml).
constexpr const char * kTestTreeIdentity = "auto_apms_behavior_tree::test_tree::TestTree";

class TreeDocumentIncludeResourceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    temp_dir_ = std::filesystem::temp_directory_path() / "tree_document_include_resource_test";
    std::filesystem::create_directories(temp_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(temp_dir_); }

  void writeFile(const std::string & filename, const std::string & content)
  {
    std::ofstream file(temp_dir_ / filename);
    file << content;
    file.close();
  }

  std::string getFilePath(const std::string & filename) { return (temp_dir_ / filename).string(); }

  std::filesystem::path temp_dir_;
};

// -----------------------------------------------------------------------------
// Resolution & merge
// -----------------------------------------------------------------------------

TEST_F(TreeDocumentIncludeResourceTest, ResolvesRegisteredResourceAndMergesTree)
{
  TreeDocument doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms=")" + std::string(kTestTreeIdentity) +
                              R"("/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="TestTree"/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(doc.mergeString(content));
  EXPECT_TRUE(doc.hasTreeName("MainTree"));
  EXPECT_TRUE(doc.hasTreeName("TestTree"));

  // The <include> element itself must not survive into the merged document.
  EXPECT_EQ(doc.writeToString().find("<include"), std::string::npos);
}

TEST_F(TreeDocumentIncludeResourceTest, ResolvesShortIdentityFormWithEmptyPackage)
{
  // ::<file_stem>::<tree_name> — omit the package name; the resource system searches all packages.
  TreeDocument doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="::test_tree::TestTree"/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="TestTree"/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(doc.mergeString(content));
  EXPECT_TRUE(doc.hasTreeName("TestTree"));
}

TEST_F(TreeDocumentIncludeResourceTest, ResolvesFullyQualifiedIdentityWithCategory)
{
  // tree/<package>::<file_stem>::<tree_name> — include the category token as well.
  TreeDocument doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="tree/auto_apms_behavior_tree::test_tree::TestTree"/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="TestTree"/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(doc.mergeString(content));
  EXPECT_TRUE(doc.hasTreeName("TestTree"));
}

TEST_F(TreeDocumentIncludeResourceTest, ResolvingLeafTreePicksOnlyThatTree)
{
  // The include_e2e resource file defines several trees. Referencing a leaf tree (no SubTree dependencies) by identity
  // must cherry-pick only that individual tree, not the rest of the file.
  TreeDocument doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2EHelperSuccess"/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="E2EHelperSuccess"/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(doc.mergeString(content));
  EXPECT_TRUE(doc.hasTreeName("E2EHelperSuccess"));
  // None of the other trees defined in the same file must have been pulled in.
  EXPECT_FALSE(doc.hasTreeName("E2ERoot"));
  EXPECT_FALSE(doc.hasTreeName("E2EHelperSequence"));
  EXPECT_FALSE(doc.hasTreeName("E2ESharedLeaf"));
}

TEST_F(TreeDocumentIncludeResourceTest, ResolvingTreePullsInItsDependencyClosure)
{
  // Referencing a composed tree must cherry-pick that tree plus the trees it transitively depends on via <SubTree>
  // (E2ERoot -> E2EHelperSuccess, E2EHelperSequence), but nothing else from the file.
  TreeDocument doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2ERoot"/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="E2ERoot"/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(doc.mergeString(content));
  EXPECT_TRUE(doc.hasTreeName("E2ERoot"));
  EXPECT_TRUE(doc.hasTreeName("E2EHelperSuccess"));
  EXPECT_TRUE(doc.hasTreeName("E2EHelperSequence"));
  // Trees unrelated to E2ERoot's closure must not be present.
  EXPECT_FALSE(doc.hasTreeName("E2EAlpha"));
  EXPECT_FALSE(doc.hasTreeName("E2ESharedLeaf"));
}

TEST_F(TreeDocumentIncludeResourceTest, TwoIncludesShareAHelperWithoutDuplicateClash)
{
  // E2EAlpha and E2EBeta both depend on E2ESharedLeaf. Two separate autoapms includes must each pull in their tree and
  // the shared helper, with the helper merged only once instead of colliding as a duplicate.
  TreeDocument doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2EAlpha"/>
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2EBeta"/>
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="E2EAlpha"/>
      <SubTree ID="E2EBeta"/>
    </Sequence>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(doc.mergeString(content));
  EXPECT_TRUE(doc.hasTreeName("E2EAlpha"));
  EXPECT_TRUE(doc.hasTreeName("E2EBeta"));
  EXPECT_TRUE(doc.hasTreeName("E2ESharedLeaf"));
}

TEST_F(TreeDocumentIncludeResourceTest, ResourceIncludeWorksViaMergeFile)
{
  // Same resolution through the mergeFile entry point (the including document lives on disk).
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms=")" + std::string(kTestTreeIdentity) +
                              R"("/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="TestTree"/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", content);

  TreeDocument doc;
  ASSERT_NO_THROW(doc.mergeFile(getFilePath("main.xml")));
  EXPECT_TRUE(doc.hasTreeName("MainTree"));
  EXPECT_TRUE(doc.hasTreeName("TestTree"));
}

TEST_F(TreeDocumentIncludeResourceTest, MixedResourceAndPathIncludes)
{
  // A single document may combine a resource-identity include with a plain path include.
  const std::string path_included = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="PathIncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("path_included.xml", path_included);

  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms=")" + std::string(kTestTreeIdentity) +
                              R"("/>
  <include path=")" + getFilePath("path_included.xml") +
                              R"("/>
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="TestTree"/>
      <SubTree ID="PathIncludedTree"/>
    </Sequence>
  </BehaviorTree>
</root>
)";
  TreeDocument doc;
  ASSERT_NO_THROW(doc.mergeString(content));
  EXPECT_TRUE(doc.hasTreeName("MainTree"));
  EXPECT_TRUE(doc.hasTreeName("TestTree"));
  EXPECT_TRUE(doc.hasTreeName("PathIncludedTree"));
}

TEST_F(TreeDocumentIncludeResourceTest, NonexistentResourceThrows)
{
  TreeDocument doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="nonexistent_pkg_98765::no_such_file::NoSuchTree"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  EXPECT_THROW(doc.mergeString(content), exceptions::TreeDocumentError);
}

// -----------------------------------------------------------------------------
// Instantiate & tick (true end-to-end)
// -----------------------------------------------------------------------------

TEST_F(TreeDocumentIncludeResourceTest, InstantiateAndTickIncludedTree)
{
  // Build a real executable tree whose root delegates to a subtree brought in via a resource include, then tick it to
  // completion. The registered TestTree is a single AlwaysSuccess, so the composed tree must report SUCCESS.
  TreeBuilder builder;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms=")" + std::string(kTestTreeIdentity) +
                              R"("/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="TestTree"/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(builder.mergeString(content));

  Tree tree = builder.instantiate("MainTree");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

TEST_F(TreeDocumentIncludeResourceTest, InstantiateAndTickComposedIncludedTree)
{
  // The include_e2e E2ERoot composes two helper subtrees (all AlwaysSuccess); ticking it must succeed. This exercises
  // whole-file merge plus subtree resolution across the include boundary end-to-end.
  TreeBuilder builder;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2ERoot"/>
  <BehaviorTree ID="MainTree">
    <SubTree ID="E2ERoot"/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(builder.mergeString(content));

  Tree tree = builder.instantiate("MainTree");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

TEST_F(TreeDocumentIncludeResourceTest, InstantiateIncludedResourceAsRootDirectly)
{
  // The included tree can also be used directly as the root, without a wrapping tree of our own.
  TreeBuilder builder;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2ERoot"/>
</root>
)";
  ASSERT_NO_THROW(builder.mergeString(content));

  Tree tree = builder.instantiate("E2ERoot");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

TEST_F(TreeDocumentIncludeResourceTest, InstantiateAndTickTwoIncludesSharingHelper)
{
  // End-to-end counterpart of TwoIncludesShareAHelperWithoutDuplicateClash: build and tick a tree that composes two
  // cherry-picked behaviors which share a helper subtree, proving the deduplicated result is a valid, runnable tree.
  TreeBuilder builder;
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2EAlpha"/>
  <include autoapms="auto_apms_behavior_tree::include_e2e::E2EBeta"/>
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="E2EAlpha"/>
      <SubTree ID="E2EBeta"/>
    </Sequence>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(builder.mergeString(content));

  Tree tree = builder.instantiate("MainTree");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}
