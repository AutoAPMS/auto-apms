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

#include "auto_apms_behavior_tree_core/behavior.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_util/exceptions.hpp"

using namespace auto_apms_behavior_tree::core;

// =============================================================================
// BehaviorResource Discovery
// =============================================================================

// The test tree is registered via auto_apms_behavior_tree_register_trees("test/resource/test_tree.xml")
// which internally also registers a behavior resource with category "tree".
// The behavior alias for a tree resource is "<file_stem>::<tree_name>", so "test_tree::TestTree".

TEST(BehaviorResourceDiscovery, FindByPackageAndAlias)
{
  // Fully specified with package name
  EXPECT_NO_THROW(BehaviorResource resource("auto_apms_behavior_tree::test_tree::TestTree"));
}

TEST(BehaviorResourceDiscovery, FindByCategoryPackageAndAlias)
{
  // Fully qualified: category/package::alias
  EXPECT_NO_THROW(BehaviorResource resource("tree/auto_apms_behavior_tree::test_tree::TestTree"));
}

TEST(BehaviorResourceDiscovery, FindUsingFindMethod)
{
  // Use the static find() method
  EXPECT_NO_THROW(BehaviorResource::find("test_tree::TestTree", "auto_apms_behavior_tree"));
}

TEST(BehaviorResourceDiscovery, VerifyIdentityFields)
{
  BehaviorResource resource("auto_apms_behavior_tree::test_tree::TestTree");
  const auto & id = resource.getIdentity();
  EXPECT_EQ(id.package_name, "auto_apms_behavior_tree");
  EXPECT_EQ(id.category_name, "tree");
  EXPECT_EQ(id.behavior_alias, "test_tree::TestTree");
}

TEST(BehaviorResourceDiscovery, NonexistentResourceThrows)
{
  EXPECT_THROW(BehaviorResource resource("nonexistent_resource_xyz"), auto_apms_util::exceptions::ResourceError);
}

// =============================================================================
// TreeResource Discovery
// =============================================================================

// TreeResource uses the same identity resolution as BehaviorResource.
// The behavior alias for a tree resource is "<file_stem>::<tree_name>", so "test_tree::TestTree".
// Both <file_stem> and <tree_name> must always be provided.

TEST(TreeResourceDiscovery, FindByPackageAndAlias)
{
  // package::file_stem::tree_name
  EXPECT_NO_THROW(TreeResource resource("auto_apms_behavior_tree::test_tree::TestTree"));
}

TEST(TreeResourceDiscovery, FindFullyQualified)
{
  // category/package::file_stem::tree_name
  EXPECT_NO_THROW(TreeResource resource("tree/auto_apms_behavior_tree::test_tree::TestTree"));
}

TEST(TreeResourceDiscovery, VerifyTreeIdentityFields)
{
  TreeResource resource("auto_apms_behavior_tree::test_tree::TestTree");
  const auto & id = resource.getIdentity();
  EXPECT_EQ(id.package_name, "auto_apms_behavior_tree");
  EXPECT_EQ(id.file_stem, "test_tree");
  EXPECT_EQ(id.tree_name, "TestTree");
}

TEST(TreeResourceDiscovery, NonexistentTreeThrows)
{
  EXPECT_THROW(TreeResource resource("nonexistent_pkg::no_file::NoTree"), auto_apms_util::exceptions::ResourceError);
}

TEST(TreeResourceDiscovery, PartialIdentityFileStemOnlyThrows)
{
  // TreeResource does not support tree-specific short forms
  EXPECT_THROW(TreeResource resource("test_tree"), auto_apms_util::exceptions::ResourceIdentityFormatError);
}

TEST(TreeResourceDiscovery, PartialIdentityTreeNameOnlyThrows)
{
  EXPECT_THROW(TreeResource resource("::::TestTree"), auto_apms_util::exceptions::ResourceIdentityFormatError);
}

TEST(TreeResourceDiscovery, PartialIdentityMissingTreeNameThrows)
{
  EXPECT_THROW(
    TreeResource resource("auto_apms_behavior_tree::test_tree::"),
    auto_apms_util::exceptions::ResourceIdentityFormatError);
}

TEST(TreeResourceDiscovery, FindByTreeNameMethod)
{
  TreeResource resource = TreeResource::findByTreeName("TestTree");
  EXPECT_EQ(resource.getIdentity().tree_name, "TestTree");
  EXPECT_EQ(resource.getIdentity().file_stem, "test_tree");
  EXPECT_EQ(resource.getIdentity().package_name, "auto_apms_behavior_tree");
}

TEST(TreeResourceDiscovery, FindByTreeNameWithPackage)
{
  TreeResource resource = TreeResource::findByTreeName("TestTree", "auto_apms_behavior_tree");
  EXPECT_EQ(resource.getIdentity().tree_name, "TestTree");
  EXPECT_EQ(resource.getIdentity().package_name, "auto_apms_behavior_tree");
}

TEST(TreeResourceDiscovery, FindByTreeNameNonexistentThrows)
{
  EXPECT_THROW(TreeResource::findByTreeName("NoSuchTree"), auto_apms_util::exceptions::ResourceError);
}

TEST(TreeResourceDiscovery, FindByFileStemMethod)
{
  TreeResource resource = TreeResource::findByFileStem("test_tree");
  EXPECT_EQ(resource.getIdentity().file_stem, "test_tree");
  EXPECT_EQ(resource.getIdentity().tree_name, "TestTree");
  EXPECT_EQ(resource.getIdentity().package_name, "auto_apms_behavior_tree");
}

TEST(TreeResourceDiscovery, FindByFileStemWithPackage)
{
  TreeResource resource = TreeResource::findByFileStem("test_tree", "auto_apms_behavior_tree");
  EXPECT_EQ(resource.getIdentity().file_stem, "test_tree");
  EXPECT_EQ(resource.getIdentity().package_name, "auto_apms_behavior_tree");
}

TEST(TreeResourceDiscovery, FindByFileStemNonexistentThrows)
{
  EXPECT_THROW(TreeResource::findByFileStem("no_such_file"), auto_apms_util::exceptions::ResourceError);
}
