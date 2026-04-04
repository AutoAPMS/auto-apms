# Copyright 2026 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for BehaviorResource and TreeResource discovery in Python."""

import pytest
from auto_apms_behavior_tree_core.resources import (
    BehaviorResource,
    BehaviorResourceIdentity,
    ResourceError,
    ResourceIdentityFormatError,
    TreeResource,
    TreeResourceIdentity,
)

# The test tree is registered via auto_apms_behavior_tree_register_trees("test/resource/test_tree.xml")
# which internally creates a behavior resource with category "tree" and alias "test_tree::TestTree".

PACKAGE = "auto_apms_behavior_tree"


# =============================================================================
# BehaviorResource Discovery
# =============================================================================


class TestBehaviorResourceDiscovery:
    def test_find_by_package_and_alias(self):
        resource = BehaviorResource(f"{PACKAGE}::test_tree::TestTree")
        assert resource.identity.package_name == PACKAGE

    def test_find_by_category_package_and_alias(self):
        resource = BehaviorResource(f"tree/{PACKAGE}::test_tree::TestTree")
        assert resource.identity.category_name == "tree"
        assert resource.identity.package_name == PACKAGE
        assert resource.identity.behavior_alias == "test_tree::TestTree"

    def test_find_using_find_method(self):
        resource = BehaviorResource.find("test_tree::TestTree", package_name=PACKAGE)
        assert resource.identity.package_name == PACKAGE

    def test_verify_identity_fields(self):
        resource = BehaviorResource(f"{PACKAGE}::test_tree::TestTree")
        assert resource.identity.package_name == PACKAGE
        assert resource.identity.category_name == "tree"
        assert resource.identity.behavior_alias == "test_tree::TestTree"

    def test_nonexistent_resource_raises(self):
        with pytest.raises(ResourceError):
            BehaviorResource("nonexistent_resource_xyz")


# =============================================================================
# TreeResource Discovery
# =============================================================================

# TreeResource uses the same identity resolution as BehaviorResource.
# The behavior alias for a tree resource is "<file_stem>::<tree_name>", so "test_tree::TestTree".
# Both <file_stem> and <tree_name> must always be provided.


class TestTreeResourceDiscovery:
    def test_find_by_package_and_alias(self):
        resource = TreeResource(f"{PACKAGE}::test_tree::TestTree")
        assert resource.identity.package_name == PACKAGE

    def test_find_fully_qualified(self):
        resource = TreeResource(f"tree/{PACKAGE}::test_tree::TestTree")
        assert resource.identity.category_name == "tree"
        assert resource.identity.package_name == PACKAGE

    def test_verify_tree_identity_fields(self):
        resource = TreeResource(f"{PACKAGE}::test_tree::TestTree")
        identity = resource.identity
        assert identity.package_name == PACKAGE
        assert identity.behavior_alias == "test_tree::TestTree"

    def test_nonexistent_tree_raises(self):
        with pytest.raises(LookupError):
            TreeResource("nonexistent_pkg::no_file::NoTree")

    def test_partial_identity_file_stem_only_raises(self):
        with pytest.raises(ResourceIdentityFormatError):
            TreeResource("test_tree")

    def test_partial_identity_tree_name_only_raises(self):
        with pytest.raises(ResourceIdentityFormatError):
            TreeResource("::::TestTree")

    def test_partial_identity_missing_tree_name_raises(self):
        with pytest.raises(ResourceIdentityFormatError):
            TreeResource(f"{PACKAGE}::test_tree::")

    def test_find_by_tree_name(self):
        resource = TreeResource.find_by_tree_name("TestTree")
        assert resource.identity.package_name == PACKAGE
        assert resource.identity.behavior_alias == "test_tree::TestTree"

    def test_find_by_tree_name_with_package(self):
        resource = TreeResource.find_by_tree_name("TestTree", package_name=PACKAGE)
        assert resource.identity.package_name == PACKAGE

    def test_find_by_tree_name_nonexistent_raises(self):
        with pytest.raises(ResourceError):
            TreeResource.find_by_tree_name("NoSuchTree")

    def test_find_by_file_stem(self):
        resource = TreeResource.find_by_file_stem("test_tree")
        assert resource.identity.package_name == PACKAGE
        assert resource.identity.behavior_alias == "test_tree::TestTree"

    def test_find_by_file_stem_with_package(self):
        resource = TreeResource.find_by_file_stem("test_tree", package_name=PACKAGE)
        assert resource.identity.package_name == PACKAGE

    def test_find_by_file_stem_nonexistent_raises(self):
        with pytest.raises(ResourceError):
            TreeResource.find_by_file_stem("no_such_file")
