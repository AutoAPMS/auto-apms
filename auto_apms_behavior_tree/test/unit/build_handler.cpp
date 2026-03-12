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

#include "auto_apms_behavior_tree/build_handler/build_handler.hpp"

#include <gtest/gtest.h>

#include "auto_apms_behavior_tree/exceptions.hpp"

using namespace auto_apms_behavior_tree;

// =============================================================================
// TreeBasedEntryPoint Decoding
// =============================================================================

TEST(TreeBasedEntryPointTest, NameOnly)
{
  TreeBasedEntryPoint ep("MyTree");
  EXPECT_EQ(ep.root_tree_name, "MyTree");
  EXPECT_TRUE(ep.inital_blackboard.empty());
}

TEST(TreeBasedEntryPointTest, NameWithEmptyParens)
{
  TreeBasedEntryPoint ep("MyTree()");
  EXPECT_EQ(ep.root_tree_name, "MyTree");
  EXPECT_TRUE(ep.inital_blackboard.empty());
}

TEST(TreeBasedEntryPointTest, SingleKeyValue)
{
  TreeBasedEntryPoint ep("MyTree(var1=value1)");
  EXPECT_EQ(ep.root_tree_name, "MyTree");
  ASSERT_EQ(ep.inital_blackboard.size(), 1u);
  EXPECT_EQ(ep.inital_blackboard.at("var1"), "value1");
}

TEST(TreeBasedEntryPointTest, MultipleKeyValues)
{
  TreeBasedEntryPoint ep("MyTree(var1=value1,var2=value2,var3=value3)");
  EXPECT_EQ(ep.root_tree_name, "MyTree");
  ASSERT_EQ(ep.inital_blackboard.size(), 3u);
  EXPECT_EQ(ep.inital_blackboard.at("var1"), "value1");
  EXPECT_EQ(ep.inital_blackboard.at("var2"), "value2");
  EXPECT_EQ(ep.inital_blackboard.at("var3"), "value3");
}

TEST(TreeBasedEntryPointTest, EmptyValueAllowed)
{
  TreeBasedEntryPoint ep("MyTree(key=)");
  EXPECT_EQ(ep.root_tree_name, "MyTree");
  ASSERT_EQ(ep.inital_blackboard.size(), 1u);
  EXPECT_EQ(ep.inital_blackboard.at("key"), "");
}

TEST(TreeBasedEntryPointTest, EmptyStringThrows)
{
  EXPECT_THROW(TreeBasedEntryPoint(""), exceptions::TreeBuildHandlerError);
}

TEST(TreeBasedEntryPointTest, MissingClosingParenThrows)
{
  EXPECT_THROW(TreeBasedEntryPoint("MyTree(var1=value1"), exceptions::TreeBuildHandlerError);
}

TEST(TreeBasedEntryPointTest, EmptyTreeNameThrows)
{
  EXPECT_THROW(TreeBasedEntryPoint("(var1=value1)"), exceptions::TreeBuildHandlerError);
}

TEST(TreeBasedEntryPointTest, MissingEqualsThrows)
{
  EXPECT_THROW(TreeBasedEntryPoint("MyTree(var1)"), exceptions::TreeBuildHandlerError);
}

TEST(TreeBasedEntryPointTest, EmptyKeyThrows)
{
  EXPECT_THROW(TreeBasedEntryPoint("MyTree(=value1)"), exceptions::TreeBuildHandlerError);
}
