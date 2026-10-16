// Copyright 2025 Robin Müller
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

#include <filesystem>
#include <fstream>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// Tests for the <include> element of TreeDocument. An include may locate the referenced file in three ways:
//   - path                : a file path (absolute, or relative to the current working directory)
//   - path + ros_pkg      : a path relative to the share directory of the given ROS 2 package
//   - autoapms            : a registered tree resource identity, resolved through the AutoAPMS resource system
// The include is processed by mergeTreeDocumentImpl regardless of the entry point (mergeFile / mergeString /
// mergeTreeDocument), which is why the suite exercises each of them.
//
// Note on 'autoapms' coverage: resolving a resource identity to a file requires an installed, discoverable tree
// resource. This package cannot register one for itself (the register_trees CMake macro is only available to
// downstream consumers), so the success path of the 'autoapms' attribute is validated end-to-end elsewhere (e.g. the
// auto_apms_px4 example). Here we exhaustively cover its argument validation and the failure path, which still
// exercises the full resolution attempt.
class TreeDocumentIncludeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    doc_ = std::make_unique<TreeDocument>();

    // Create a temporary directory for test files
    temp_dir_ = std::filesystem::temp_directory_path() / "tree_document_include_test";
    std::filesystem::create_directories(temp_dir_);
  }

  void TearDown() override
  {
    // Clean up temporary files
    std::filesystem::remove_all(temp_dir_);
  }

  void writeFile(const std::string & filename, const std::string & content)
  {
    std::ofstream file(temp_dir_ / filename);
    file << content;
    file.close();
  }

  std::string getFilePath(const std::string & filename) { return (temp_dir_ / filename).string(); }

  std::unique_ptr<TreeDocument> doc_;
  std::filesystem::path temp_dir_;
};

TEST_F(TreeDocumentIncludeTest, MergeFileWithSingleInclude)
{
  // Create the included file with a simple tree
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included_tree.xml", included_content);

  // Create the main file that includes the other file
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included_tree.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main_tree.xml", main_content);

  // Merge the main file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main_tree.xml")));

  // Verify both trees are present
  std::vector<std::string> tree_names = doc_->getAllTreeNames();
  ASSERT_EQ(tree_names.size(), 2);
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("IncludedTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithMultipleIncludes)
{
  // Create first included file
  const std::string included1_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="FirstIncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included1.xml", included1_content);

  // Create second included file
  const std::string included2_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="SecondIncludedTree">
    <AlwaysFailure/>
  </BehaviorTree>
</root>
)";
  writeFile("included2.xml", included2_content);

  // Create the main file that includes both files
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included1.xml") +
                                   R"("/>
  <include path=")" + getFilePath("included2.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <Sequence>
      <AlwaysSuccess/>
    </Sequence>
  </BehaviorTree>
</root>
)";
  writeFile("main_tree.xml", main_content);

  // Merge the main file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main_tree.xml")));

  // Verify all three trees are present
  std::vector<std::string> tree_names = doc_->getAllTreeNames();
  ASSERT_EQ(tree_names.size(), 3);
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("FirstIncludedTree"));
  EXPECT_TRUE(doc_->hasTreeName("SecondIncludedTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithNestedIncludes)
{
  // Create the deepest nested file
  const std::string deep_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="DeepTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("deep.xml", deep_content);

  // Create middle file that includes the deep file
  const std::string middle_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("deep.xml") +
                                     R"("/>
  <BehaviorTree ID="MiddleTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("middle.xml", middle_content);

  // Create main file that includes the middle file
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("middle.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merge the main file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main.xml")));

  // Verify all three trees are present (recursive include worked)
  std::vector<std::string> tree_names = doc_->getAllTreeNames();
  ASSERT_EQ(tree_names.size(), 3);
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("MiddleTree"));
  EXPECT_TRUE(doc_->hasTreeName("DeepTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithRelativePath)
{
  // Create a subdirectory
  std::filesystem::create_directories(temp_dir_ / "subdir");

  // Create included file in subdirectory
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="SubdirTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  std::ofstream file(temp_dir_ / "subdir" / "tree.xml");
  file << included_content;
  file.close();

  // Create main file with absolute path (since relative paths need ros_pkg context)
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + (temp_dir_ / "subdir" / "tree.xml").string() +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merge the main file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main.xml")));

  // Verify both trees are present
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("SubdirTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithEmptyPath)
{
  // Create a file with an empty path attribute
  const std::string invalid_content = R"(
<root BTCPP_format="4">
  <include path=""/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("invalid.xml", invalid_content);

  // Merging should fail due to empty path
  EXPECT_THROW(doc_->mergeFile(getFilePath("invalid.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithMissingPathAttribute)
{
  // Create a file with an include element without path attribute
  const std::string invalid_content = R"(
<root BTCPP_format="4">
  <include/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("invalid.xml", invalid_content);

  // Merging should fail due to missing path attribute
  EXPECT_THROW(doc_->mergeFile(getFilePath("invalid.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithNonExistentIncludedFile)
{
  // Create a file that tries to include a non-existent file
  const std::string invalid_content = R"(
<root BTCPP_format="4">
  <include path="/non/existent/file.xml"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("invalid.xml", invalid_content);

  // Merging should fail due to non-existent included file
  EXPECT_THROW(doc_->mergeFile(getFilePath("invalid.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithEmptyRosPkg)
{
  // Create a file with an empty ros_pkg attribute
  const std::string invalid_content = R"(
<root BTCPP_format="4">
  <include path="some/path.xml" ros_pkg=""/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("invalid.xml", invalid_content);

  // Merging should fail due to empty ros_pkg
  EXPECT_THROW(doc_->mergeFile(getFilePath("invalid.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithAbsolutePathAndRosPkg)
{
  // Create a file with both absolute path and ros_pkg attribute (not allowed)
  const std::string invalid_content = R"(
<root BTCPP_format="4">
  <include path="/absolute/path.xml" ros_pkg="some_package"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("invalid.xml", invalid_content);

  // Merging should fail due to absolute path with ros_pkg
  EXPECT_THROW(doc_->mergeFile(getFilePath("invalid.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithNonExistentRosPkg)
{
  // Create a file with a non-existent ROS package
  const std::string invalid_content = R"(
<root BTCPP_format="4">
  <include path="trees/my_tree.xml" ros_pkg="non_existent_package_12345"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("invalid.xml", invalid_content);

  // Merging should fail due to non-existent ROS package
  EXPECT_THROW(doc_->mergeFile(getFilePath("invalid.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithDuplicateTreeNames)
{
  // Create included file with a tree name
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="DuplicateName">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included.xml", included_content);

  // Create main file with the same tree name
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included.xml") +
                                   R"("/>
  <BehaviorTree ID="DuplicateName">
    <AlwaysFailure/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merging should fail due to duplicate tree names
  EXPECT_THROW(doc_->mergeFile(getFilePath("main.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithDuplicateTreeNamesInIncludes)
{
  // Create first included file
  const std::string included1_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="DuplicateName">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included1.xml", included1_content);

  // Create second included file with the same tree name
  const std::string included2_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="DuplicateName">
    <AlwaysFailure/>
  </BehaviorTree>
</root>
)";
  writeFile("included2.xml", included2_content);

  // Create main file that includes both
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included1.xml") +
                                   R"("/>
  <include path=")" + getFilePath("included2.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merging should fail due to duplicate tree names in includes
  EXPECT_THROW(doc_->mergeFile(getFilePath("main.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileIncludesOnlyTrees)
{
  // Create included file with a tree
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included.xml", included_content);

  // Create main file
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merge the main file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main.xml")));

  // Verify the document structure - should have exactly the tree elements
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<BehaviorTree ID=\"MainTree\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<BehaviorTree ID=\"IncludedTree\"") != std::string::npos);

  // The include element should NOT be in the final document
  EXPECT_TRUE(xml.find("<include") == std::string::npos);
}

TEST_F(TreeDocumentIncludeTest, MergeFilePreservesRootTreeAttribute)
{
  // Create included file
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included.xml", included_content);

  // Create main file with root tree attribute
  const std::string main_content = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <include path=")" + getFilePath("included.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merge with adopt_root_tree = true
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main.xml"), true));

  // Verify the root tree name is set correctly
  EXPECT_TRUE(doc_->hasRootTreeName());
  EXPECT_EQ(doc_->getRootTreeName(), "MainTree");
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithIncludedFileHavingMultipleTrees)
{
  // Create included file with multiple trees
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="FirstIncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
  <BehaviorTree ID="SecondIncludedTree">
    <AlwaysFailure/>
  </BehaviorTree>
</root>
)";
  writeFile("included.xml", included_content);

  // Create main file
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merge the main file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main.xml")));

  // Verify all trees are present
  std::vector<std::string> tree_names = doc_->getAllTreeNames();
  ASSERT_EQ(tree_names.size(), 3);
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("FirstIncludedTree"));
  EXPECT_TRUE(doc_->hasTreeName("SecondIncludedTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeStringWithIncludeElement)
{
  // Create the file to be included
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included.xml", included_content);

  // Create a string with an include element
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";

  // Merge the string
  ASSERT_NO_THROW(doc_->mergeString(main_content));

  // Verify both trees are present
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("IncludedTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithNoTreesInIncludedFile)
{
  // Create included file without any trees (just root element)
  const std::string included_content = R"(
<root BTCPP_format="4">
</root>
)";
  writeFile("included.xml", included_content);

  // Create main file
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merge the main file - should succeed even if included file has no trees
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main.xml")));

  // Verify only the main tree is present
  std::vector<std::string> tree_names = doc_->getAllTreeNames();
  ASSERT_EQ(tree_names.size(), 1);
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeFileIncludeOrderPreserved)
{
  // Create first included file
  const std::string included1_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TreeA">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included1.xml", included1_content);

  // Create second included file
  const std::string included2_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TreeB">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included2.xml", included2_content);

  // Create main file with includes in specific order
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included1.xml") +
                                   R"("/>
  <include path=")" + getFilePath("included2.xml") +
                                   R"("/>
  <BehaviorTree ID="TreeC">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merge the main file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("main.xml")));

  // Get all tree names and verify they are all present
  std::vector<std::string> tree_names = doc_->getAllTreeNames();
  ASSERT_EQ(tree_names.size(), 3);

  // The included trees should come before the main tree in the document
  // (due to how the implementation inserts them)
  std::string xml = doc_->writeToString();
  size_t pos_a = xml.find("TreeA");
  size_t pos_b = xml.find("TreeB");
  size_t pos_c = xml.find("TreeC");

  EXPECT_LT(pos_a, pos_c);
  EXPECT_LT(pos_b, pos_c);
}

TEST_F(TreeDocumentIncludeTest, MergeFileInvalidXmlInIncludedFile)
{
  // Create included file with invalid XML
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess>
  </BehaviorTree>
</root>
)";  // Missing closing tag for AlwaysSuccess
  writeFile("invalid.xml", included_content);

  // Create main file
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("invalid.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merging should fail due to invalid XML in included file
  EXPECT_THROW(doc_->mergeFile(getFilePath("main.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithWrongFormatInIncludedFile)
{
  // Create included file with wrong format version
  const std::string included_content = R"(
<root BTCPP_format="3">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("wrong_format.xml", included_content);

  // Create main file (with format 4)
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("wrong_format.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merging should fail due to incompatible format version
  EXPECT_THROW(doc_->mergeFile(getFilePath("main.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithDeeplyNestedIncludes)
{
  // Create a chain of 5 nested includes to test deep recursion
  const int depth = 5;

  for (int i = depth; i >= 1; --i) {
    std::string content;
    if (i == depth) {
      // Deepest file has no includes
      content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Tree)" +
                std::to_string(i) + R"(">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
    } else {
      // Other files include the next file
      content = R"(
<root BTCPP_format="4">
  <include path=")" +
                getFilePath("level" + std::to_string(i + 1) + ".xml") +
                R"("/>
  <BehaviorTree ID="Tree)" +
                std::to_string(i) + R"(">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
    }
    writeFile("level" + std::to_string(i) + ".xml", content);
  }

  // Merge the top-level file
  ASSERT_NO_THROW(doc_->mergeFile(getFilePath("level1.xml")));

  // Verify all trees are present
  std::vector<std::string> tree_names = doc_->getAllTreeNames();
  ASSERT_EQ(tree_names.size(), static_cast<size_t>(depth));

  for (int i = 1; i <= depth; ++i) {
    EXPECT_TRUE(doc_->hasTreeName("Tree" + std::to_string(i)));
  }
}

TEST_F(TreeDocumentIncludeTest, MergeFileWithIncludeDuplicatingExistingTree)
{
  // First, add a tree to the document
  const std::string initial_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="ExistingTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  ASSERT_NO_THROW(doc_->mergeString(initial_content));
  EXPECT_TRUE(doc_->hasTreeName("ExistingTree"));

  // Create included file with the same tree name
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="ExistingTree">
    <AlwaysFailure/>
  </BehaviorTree>
</root>
)";
  writeFile("duplicate.xml", included_content);

  // Create main file that includes the duplicate
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("duplicate.xml") +
                                   R"("/>
  <BehaviorTree ID="NewTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", main_content);

  // Merging should fail because the included file has a tree that already exists in this document
  EXPECT_THROW(doc_->mergeFile(getFilePath("main.xml")), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeFileCircularIncludePrevented)
{
  // This test verifies that circular includes are detected and prevented
  // The implementation tracks the include stack and throws when a circular include is detected

  // Create file A that includes file B
  const std::string file_a_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_b.xml") +
                                     R"("/>
  <BehaviorTree ID="TreeA">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("file_a.xml", file_a_content);

  // Create file B that includes file A (circular!)
  const std::string file_b_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_a.xml") +
                                     R"("/>
  <BehaviorTree ID="TreeB">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("file_b.xml", file_b_content);

  // Merging should fail with a circular include error
  EXPECT_THROW(doc_->mergeFile(getFilePath("file_a.xml")), exceptions::TreeDocumentError);

  // Verify the error message mentions circular include
  try {
    TreeDocument doc2;
    doc2.mergeFile(getFilePath("file_a.xml"));
    FAIL() << "Expected TreeDocumentError";
  } catch (const exceptions::TreeDocumentError & e) {
    EXPECT_TRUE(std::string(e.what()).find("Circular include") != std::string::npos)
      << "Error message should mention 'Circular include', but got: " << e.what();
  }
}

TEST_F(TreeDocumentIncludeTest, MergeFileSelfIncludePrevented)
{
  // Test that a file including itself is detected as circular
  const std::string self_include_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("self.xml") +
                                           R"("/>
  <BehaviorTree ID="SelfTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("self.xml", self_include_content);

  EXPECT_THROW(doc_->mergeFile(getFilePath("self.xml")), exceptions::TreeDocumentError);
}

// ============================================================================
// Tests using mergeString
// ============================================================================

TEST_F(TreeDocumentIncludeTest, MergeStringWithSingleInclude)
{
  // Create the included file
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included.xml", included_content);

  // Merge using a string that includes the file
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";

  ASSERT_NO_THROW(doc_->mergeString(main_content));

  // Verify both trees are present
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("IncludedTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeStringCircularIncludePrevented)
{
  // Create file A that includes file B
  const std::string file_a_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_b.xml") +
                                     R"("/>
  <BehaviorTree ID="TreeA">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("file_a.xml", file_a_content);

  // Create file B that includes file A (circular!)
  const std::string file_b_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_a.xml") +
                                     R"("/>
  <BehaviorTree ID="TreeB">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("file_b.xml", file_b_content);

  // Create a string that includes file A - should detect the circular include
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_a.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";

  EXPECT_THROW(doc_->mergeString(main_content), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeStringWithNestedIncludes)
{
  // Create deeply nested includes via mergeString entry point
  const std::string deep_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="DeepTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("deep.xml", deep_content);

  const std::string middle_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("deep.xml") +
                                     R"("/>
  <BehaviorTree ID="MiddleTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("middle.xml", middle_content);

  // Use mergeString to start the chain
  const std::string main_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("middle.xml") +
                                   R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";

  ASSERT_NO_THROW(doc_->mergeString(main_content));

  // Verify all trees are present
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("MiddleTree"));
  EXPECT_TRUE(doc_->hasTreeName("DeepTree"));
}

// ============================================================================
// Tests using mergeTreeDocument
// ============================================================================

TEST_F(TreeDocumentIncludeTest, MergeTreeDocumentWithInclude)
{
  // Create the included file
  const std::string included_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="IncludedTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("included.xml", included_content);

  // Create a TreeDocument with an include
  TreeDocument other_doc;
  const std::string other_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("included.xml") +
                                    R"("/>
  <BehaviorTree ID="OtherTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  other_doc.mergeString(other_content);

  // Now merge the other document into this one
  // Note: The include was already processed when we called mergeString on other_doc
  ASSERT_NO_THROW(doc_->mergeTreeDocument(other_doc));

  // Verify all trees are present
  EXPECT_TRUE(doc_->hasTreeName("OtherTree"));
  EXPECT_TRUE(doc_->hasTreeName("IncludedTree"));
}

TEST_F(TreeDocumentIncludeTest, MergeTreeDocumentCircularIncludePrevented)
{
  // Create file A that includes file B
  const std::string file_a_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_b.xml") +
                                     R"("/>
  <BehaviorTree ID="TreeA">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("file_a.xml", file_a_content);

  // Create file B that includes file A (circular!)
  const std::string file_b_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_a.xml") +
                                     R"("/>
  <BehaviorTree ID="TreeB">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("file_b.xml", file_b_content);

  // Create an XMLDocument (low-level) that includes file_a
  tinyxml2::XMLDocument xml_doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("file_a.xml") +
                              R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  xml_doc.Parse(content.c_str());

  // Merging should detect the circular include
  EXPECT_THROW(doc_->mergeTreeDocument(xml_doc), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, MergeTreeDocumentXMLDocumentWithNestedIncludes)
{
  // Test mergeTreeDocument(const XMLDocument&, ...) with nested includes
  const std::string deep_content = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="DeepTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("deep.xml", deep_content);

  const std::string middle_content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("deep.xml") +
                                     R"("/>
  <BehaviorTree ID="MiddleTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("middle.xml", middle_content);

  // Create an XMLDocument that includes middle.xml
  tinyxml2::XMLDocument xml_doc;
  const std::string content = R"(
<root BTCPP_format="4">
  <include path=")" + getFilePath("middle.xml") +
                              R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  xml_doc.Parse(content.c_str());

  ASSERT_NO_THROW(doc_->mergeTreeDocument(xml_doc));

  // Verify all trees are present
  EXPECT_TRUE(doc_->hasTreeName("MainTree"));
  EXPECT_TRUE(doc_->hasTreeName("MiddleTree"));
  EXPECT_TRUE(doc_->hasTreeName("DeepTree"));
}

// ============================================================================
// Tests for the 'autoapms' attribute (resource-identity based includes)
// ============================================================================

// Helper: merge the given XML string and return the message of the TreeDocumentError it is expected to raise.
// Fails the test if no such exception is thrown.
namespace
{
std::string expectMergeStringError(TreeDocument & doc, const std::string & content)
{
  try {
    doc.mergeString(content);
  } catch (const exceptions::TreeDocumentError & e) {
    return e.what();
  }
  ADD_FAILURE() << "Expected a TreeDocumentError to be thrown, but none was.";
  return {};
}
}  // namespace

TEST_F(TreeDocumentIncludeTest, AutoapmsIncludeEmptyIdentityThrows)
{
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms=""/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  const std::string msg = expectMergeStringError(*doc_, content);
  EXPECT_NE(msg.find("autoapms"), std::string::npos) << "Message should name the offending attribute: " << msg;
}

TEST_F(TreeDocumentIncludeTest, AutoapmsIncludeCombinedWithPathThrows)
{
  // Specifying both a file path and a resource identity is ambiguous and must be rejected.
  const std::string content = R"(
<root BTCPP_format="4">
  <include path="some/file.xml" autoapms="some_package::some_file::SomeTree"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  const std::string msg = expectMergeStringError(*doc_, content);
  EXPECT_NE(msg.find("path"), std::string::npos) << msg;
  EXPECT_NE(msg.find("autoapms"), std::string::npos) << msg;
}

TEST_F(TreeDocumentIncludeTest, AutoapmsIncludeCombinedWithRosPkgThrows)
{
  // 'ros_pkg' only makes sense together with 'path' (the package is already encoded in the resource identity). This is
  // rejected before any resolution is attempted, so the identity need not exist.
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="some_package::some_file::SomeTree" ros_pkg="some_package"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  const std::string msg = expectMergeStringError(*doc_, content);
  EXPECT_NE(msg.find("ros_pkg"), std::string::npos) << msg;
  EXPECT_NE(msg.find("autoapms"), std::string::npos) << msg;
}

TEST_F(TreeDocumentIncludeTest, AutoapmsIncludeUnresolvableIdentityThrows)
{
  // A well-formed but non-existent identity must surface a TreeDocumentError. This drives the full resolution attempt
  // (constructing a TreeResource), proving the 'autoapms' path is wired to the resource system.
  const std::string identity = "nonexistent_pkg_12345::no_such_file::NoSuchTree";
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms=")" + identity +
                              R"("/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  const std::string msg = expectMergeStringError(*doc_, content);
  EXPECT_NE(msg.find(identity), std::string::npos) << "Message should echo the offending identity: " << msg;
}

TEST_F(TreeDocumentIncludeTest, AutoapmsIncludeMalformedIdentityThrows)
{
  // An identity that is missing the required <tree_name> token is malformed and cannot even be parsed into a
  // TreeResourceIdentity. It must still be reported as a TreeDocumentError rather than escaping as another type.
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="missing_tree_name_alias"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  EXPECT_THROW(doc_->mergeString(content), exceptions::TreeDocumentError);
}

TEST_F(TreeDocumentIncludeTest, AutoapmsIncludeUnresolvableThrowsViaMergeFile)
{
  // Same failure surfaces through the mergeFile entry point (the include is processed in the shared implementation).
  const std::string content = R"(
<root BTCPP_format="4">
  <include autoapms="nonexistent_pkg_12345::no_such_file::NoSuchTree"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  writeFile("main.xml", content);
  EXPECT_THROW(doc_->mergeFile(getFilePath("main.xml")), exceptions::TreeDocumentError);
}

// ============================================================================
// Generic include validation (independent of the locating attribute)
// ============================================================================

TEST_F(TreeDocumentIncludeTest, IncludeWithoutLocatingAttributeThrows)
{
  // An <include> that specifies neither 'path' nor 'autoapms' cannot locate anything and must be rejected with a
  // message naming both accepted attributes.
  const std::string content = R"(
<root BTCPP_format="4">
  <include/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  const std::string msg = expectMergeStringError(*doc_, content);
  EXPECT_NE(msg.find("path"), std::string::npos) << msg;
  EXPECT_NE(msg.find("autoapms"), std::string::npos) << msg;
}

TEST_F(TreeDocumentIncludeTest, IncludeWithOnlyRosPkgThrows)
{
  // 'ros_pkg' on its own does not locate a file (it only qualifies a 'path'), so this must be rejected.
  const std::string content = R"(
<root BTCPP_format="4">
  <include ros_pkg="some_package"/>
  <BehaviorTree ID="MainTree">
    <AlwaysSuccess/>
  </BehaviorTree>
</root>
)";
  EXPECT_THROW(doc_->mergeString(content), exceptions::TreeDocumentError);
}