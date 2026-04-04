# Copyright 2025 Robin Müller
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

import ament_index_python
import os
import yaml

from auto_apms_util.resources import *
from .tree.node_model import NodeModelMap


_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP = "\\n"
_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP = "|"
_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP = "/"
_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP = "::"

_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR = "auto_apms_behavior_tree_core__behavior"
_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST = "auto_apms_behavior_tree_core__node_manifest"

_AUTO_APMS_BEHAVIOR_TREE_CORE__INTERNAL_BEHAVIOR_CATEGORY_SUFFIX = "__internal"
_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY = "default"
_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE = "tree"

BASE_CLASS_TYPE__BEHAVIOR_TREE_NODE = "auto_apms_behavior_tree::core::NodeRegistrationInterface"


class ResourceIdentityFormatError(Exception):
    pass


class ResourceError(Exception):
    pass


class NodeManifestError(Exception):
    pass


class NodeManifestResourceIdentity:
    """
    Base class that encapsulates the identity string for a registered behavior tree node manifest.

    This is the Python equivalent of auto_apms_behavior_tree::core::NodeManifestResourceIdentity.
    """

    def __init__(self, identity: str = None):
        """
        Initialize a node manifest resource identity from an identity string.

        Identity must be formatted like `<package_name>::<metadata_id>`. `package_name` is optional.

        Args:
            identity: Identity string for a specific node manifest resource, or None for manual creation.
        """
        self.package_name = ""
        self.metadata_id = ""

        if identity is None:
            return

        # Parse package and resource alias
        if _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP in identity:
            separator_pos = identity.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP)
            self.package_name = identity[:separator_pos]
            self.metadata_id = identity[
                separator_pos + len(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP) :
            ]
        else:
            # If only a single token is given, assume it's metadata_id
            self.package_name = ""
            self.metadata_id = identity

        if not self.metadata_id:
            raise ResourceIdentityFormatError(
                f"Node manifest resource identity string '{identity}' is invalid. Metadata ID must not be empty."
            )

    def __str__(self) -> str:
        """Create the corresponding identity string."""
        return self.package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + self.metadata_id

    def __repr__(self):
        return f'{self.__class__.__name__}("{str(self)}")'

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other: "NodeManifestResourceIdentity") -> bool:
        return str(self) == str(other)

    def __lt__(self, other: "NodeManifestResourceIdentity") -> bool:
        return str(self) < str(other)

    @property
    def empty(self) -> bool:
        """Determine whether this behavior resource identity object is considered empty."""
        return not self.package_name and not self.metadata_id


class NodeManifest:
    """
    Data structure holding information about which behavior tree node plugin to load and how to configure them.

    This is the Python equivalent of auto_apms_behavior_tree::core::NodeManifest.
    A node manifest is a collection of dictionaries mapped by node names.
    """

    def __init__(self, node_dict: dict[str, dict] = None):
        """
        Initialize a node manifest from a dictionary.

        Args:
            node_dict: Dictionary mapping node names to node registration options for manual creation
        """
        self._node_dict: dict[str, dict] = {}
        if node_dict:
            if not isinstance(node_dict, dict):
                raise TypeError("node_dict must be a dictionary")
            self._node_dict = node_dict.copy()

    @property
    def data(self) -> dict[str, dict]:
        """Get the manifest data as a dictionary of node option dictionaries."""
        return self._node_dict.copy()

    def get_node_names(self) -> list[str]:
        """Get all node names defined in this manifest."""
        return list(self._node_dict.keys())

    def get_node_registration_options(self, node_name: str) -> dict:
        """
        Get registration options for a specific node.

        Args:
            node_name: The name of the node to get options for.

        Returns:
            Dictionary containing node options.

        Raises:
            KeyError: If the node name is not found in the manifest.
        """
        if node_name not in self._node_dict:
            raise KeyError(f"Node '{node_name}' not found in manifest")
        return self._node_dict[node_name].copy()

    def has_node(self, node_name: str) -> bool:
        """Check if a node is defined in this manifest."""
        return node_name in self._node_dict

    def add_node(self, node_name: str, options: dict) -> "NodeManifest":
        """
        Add registration options for a behavior tree node to the manifest.

        Args:
            node_name: Name of the behavior tree node.
            options: Dictionary of registration options to be used when loading the behavior tree node.

        Returns:
            Modified node manifest (self).

        Raises:
            NodeManifestError: If the registration options are invalid or node_name already exists.
        """
        if not isinstance(options, dict):
            raise NodeManifestError(f"Options for node '{node_name}' must be a dictionary")
        if not options.get("class_name", "").strip():
            raise NodeManifestError(f"Invalid registration options for node '{node_name}': class_name is required")
        if node_name in self._node_dict.keys():
            raise NodeManifestError(f"Node '{node_name}' already exists in manifest")

        self._node_dict[node_name] = options.copy()
        return self

    def remove_node(self, node_name: str) -> "NodeManifest":
        """
        Remove registration options for a behavior tree node.

        Args:
            node_name: Name of the behavior tree node.

        Returns:
            Modified node manifest (self).

        Raises:
            NodeManifestError: If node_name doesn't exist.
        """
        if node_name not in self._node_dict:
            raise NodeManifestError(f"Node '{node_name}' not found in manifest")
        del self._node_dict[node_name]
        return self

    def merge(self, other: "NodeManifest", replace: bool = False) -> "NodeManifest":
        """
        Merge another NodeManifest with this one.

        Args:
            other: Other node manifest.
            replace: True for automatically replacing entries with the same key.
                    If False, raises error when other contains existing keys.

        Returns:
            Modified node manifest (self).

        Raises:
            NodeManifestError: If other shares entries and replace is False.
        """
        for node_name, options in other._node_dict.items():
            if node_name in self._node_dict and not replace:
                raise NodeManifestError(f"Node '{node_name}' already exists and replace=False")
            self._node_dict[node_name] = options
        return self

    @property
    def size(self) -> int:
        """Get the number of behavior tree nodes this manifest holds registration options for."""
        return len(self._node_dict)

    @property
    def empty(self) -> bool:
        """Determine whether any node registration options have been added to the manifest."""
        return len(self._node_dict) == 0

    def to_file(self, file_path: str) -> None:
        """
        Write the node manifest to a YAML file.

        Args:
            file_path: Path to the target file.

        Raises:
            IOError: If file cannot be opened or written.
        """
        try:
            with open(file_path, "w") as f:
                yaml.dump(self._node_dict, f, default_flow_style=False)
        except Exception as e:
            raise IOError(f"Failed to write manifest to file '{file_path}': {e}")

    def dump(self):
        """
        Dump the node manifest as a YAML string.
        """
        return yaml.safe_dump(self._node_dict)

    @staticmethod
    def from_file(file_path: str) -> "NodeManifest":
        """
        Create a node plugin manifest from a YAML file.

        Args:
            file_path: Path to the manifest file.

        Returns:
            NodeManifest created from the file.

        Raises:
            yaml.error.YAMLError: If the file does not contain a valid YAML mapping.
            OSError: If the file cannot be read.
        """
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)
            if not isinstance(data, dict):
                raise yaml.error.YAMLError(f"File '{file_path}' does not contain a valid YAML mapping")

            manifest = NodeManifest()
            for node_name, node_data in data.items():
                if isinstance(node_data, dict):
                    manifest.add_node(node_name, node_data)
                else:
                    raise yaml.error.YAMLError(f"Invalid node data for '{node_name}' in file '{file_path}'")

            return manifest

    @staticmethod
    def from_files(file_paths: list[str]) -> "NodeManifest":
        """
        Create a node plugin manifest from multiple files. They are loaded in the given order.

        Args:
            file_paths: Paths to the manifest files.

        Returns:
            NodeManifest created by merging all files.

        Raises:
            NodeManifestError: If merge fails for any file.
        """
        manifest = NodeManifest()
        for file_path in file_paths:
            try:
                manifest.merge(NodeManifest.from_file(file_path), replace=True)
            except Exception as e:
                raise NodeManifestError(
                    f"Error creating node manifest from multiple files (parsing file {file_path}): {e}"
                ) from e
        return manifest

    @staticmethod
    def from_resource(search_identity: NodeManifestResourceIdentity | str) -> "NodeManifest":
        """
        Create a node manifest from an installed resource.

        The resource identity must be specified in the format `<package_name>::<metadata_id>` or simply `<metadata_id>`.

        Args:
            search_identity: Node manifest resource identity used for searching the corresponding resource.

        Returns:
            Node manifest created from the corresponding resource.


        """
        if isinstance(search_identity, str):
            search_identity = NodeManifestResourceIdentity(search_identity)
        return NodeManifestResource(search_identity).node_manifest


class NodeManifestResource:
    """
    Class containing behavior tree node manifest resource data.

    This is the Python equivalent of auto_apms_behavior_tree::core::NodeManifestResource.
    """

    def __init__(self, search_identity: NodeManifestResourceIdentity | str):
        """
        Initialize a node manifest resource using an identity.

        Args:
            search_identity: Node manifest resource identity object used for searching the corresponding resource.
        """
        if isinstance(search_identity, str):
            search_identity = NodeManifestResourceIdentity(search_identity)
        elif not isinstance(search_identity, NodeManifestResourceIdentity):
            raise TypeError("Identity must be a NodeManifestResourceIdentity object or a string.")

        # Load resource data from ament index
        self._unique_identity = NodeManifestResourceIdentity()
        self._node_manifest_file_path = ""
        self._node_manifest = NodeManifest()
        self._node_model_file_path = ""

        search_packages: set[str] = set()
        if search_identity.package_name:
            search_packages.add(search_identity.package_name)
        else:
            search_packages = set(
                ament_index_python.get_resources(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST)
            )

        matching_count = 0
        for package in search_packages:
            content, base_path = ament_index_python.get_resource(
                _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST, package
            )
            lines = content.splitlines()
            for line in lines:
                parts = line.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP)
                if len(parts) != 3:
                    raise ResourceError(
                        f"Invalid node manifest resource file (Package: '{package}'). Invalid line: {line}."
                    )

                found_metadata_id = parts[0]
                if found_metadata_id != search_identity.metadata_id:
                    continue

                # Found matching resource - Increase counter
                matching_count += 1

                # Now fill the other member variables in case the resource matches (if match is not unique, error is thrown later and the object is discarded)

                self._unique_identity.package_name = package
                self._unique_identity.metadata_id = found_metadata_id
                self._node_manifest_file_path = os.path.join(base_path, parts[1])
                self._node_manifest = NodeManifest.from_file(self._node_manifest_file_path)
                self._node_model_file_path = os.path.join(base_path, parts[2])
                self._node_model = NodeModelMap(self._node_model_file_path)

        if matching_count == 0:
            raise ResourceError(f"No node manifest resource was found using identity '{search_identity}'.")
        if matching_count > 1:
            raise ResourceError(f"There are multiple node manifest resources with metadata ID '{search_identity}'.")

    @staticmethod
    def find(metadata_id: str, package_name: str = "") -> "NodeManifestResource":
        """
        Find a node manifest resource by metadata ID.

        Args:
            metadata_id: Metadata ID of the resource to find.
            package_name: Optional package name to narrow search.

        Returns:
            NodeManifestResource instance for the found resource.

        Raises:
            ResourceError: If resource cannot be found.
        """
        return NodeManifestResource(
            package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + metadata_id
        )

    @property
    def identity(self) -> "NodeManifestResource":
        """
        Get the unique identity for this resource.
        """
        return self._unique_identity

    @property
    def node_manifest(self) -> NodeManifest:
        """
        Get the behavior tree node manifest object for this resource.
        """
        return self._node_manifest

    @property
    def node_model(self) -> NodeModelMap:
        """
        Get the behavior tree node model associated with this resource.
        """
        return self._node_model


class BehaviorResourceIdentity:
    """
    Base class that encapsulates the identity string for a registered behavior.

    This is the Python equivalent of auto_apms_behavior_tree::core::BehaviorResourceIdentity.
    """

    def __init__(self, identity: str = None, default_category: str = None):
        """
        Initialize a behavior resource identity from an identity string.

        Identity must be formatted like `<category_name>/<package_name>::<resource_name>`.
        Both category_name and package_name are optional.

        Args:
            identity: Identity string for a specific behavior resource, or None for manual creation.
        """
        self.category_name = ""
        self.package_name = ""
        self.behavior_alias = ""

        if identity is None:
            return

        # Parse and separate category
        behavior_alias_part = identity
        if _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP in identity:
            category_pos = identity.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP)
            self.category_name = identity[:category_pos]
            behavior_alias_part = identity[
                category_pos + len(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP) :
            ]

        # If no category is explicitly specified, use the given default one
        if default_category:
            if (not self.category_name) or (
                self.category_name == _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY
            ):
                self.category_name = default_category

        # Parse package and alias
        if _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP in behavior_alias_part:
            separator_pos = behavior_alias_part.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP)
            self.package_name = behavior_alias_part[:separator_pos]
            self.behavior_alias = behavior_alias_part[
                separator_pos + len(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP) :
            ]
        else:
            # If only a single token is given, assume it's behavior_alias
            self.package_name = ""
            self.behavior_alias = behavior_alias_part

        if not self.package_name and not self.behavior_alias:
            raise ResourceIdentityFormatError(
                f"Behavior resource identity string '{identity}' is invalid. Package and behavior alias must not be empty."
            )

    def __str__(self) -> str:
        """Create the corresponding identity string."""
        result = ""
        if self.category_name:
            result += self.category_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP
        result += self.package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + self.behavior_alias
        return result

    def __repr__(self):
        return f'{self.__class__.__name__}("{str(self)}")'

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other: "BehaviorResourceIdentity") -> bool:
        return str(self) == str(other)

    def __lt__(self, other: "BehaviorResourceIdentity") -> bool:
        return str(self) < str(other)

    @property
    def empty(self) -> bool:
        """Determine whether this behavior resource identity object is considered empty."""
        return not self.package_name and not self.behavior_alias


class BehaviorResource:
    """
    Class containing behavior resource data.

    This is the Python equivalent of auto_apms_behavior_tree::core::BehaviorResource.
    """

    def __init__(self, search_identity: BehaviorResourceIdentity | str):
        """
        Initialize a behavior resource using an identity.

        Args:
            search_identity: Behavior resource identity object used for searching the corresponding resource.
        """
        if isinstance(search_identity, str):
            search_identity = BehaviorResourceIdentity(search_identity)
        elif not isinstance(search_identity, BehaviorResourceIdentity):
            raise TypeError("Identity must be a BehaviorResourceIdentity object or a string.")

        # Load resource data from ament index
        self._unique_identity = BehaviorResourceIdentity()
        self._build_request = ""
        self._build_request_file_path = ""
        self._default_build_handler = ""
        self._entry_point = ""
        self._node_manifest = NodeManifest()

        # Find the resource in the ament index - search across all packages if needed
        search_packages: set[str] = set()
        if search_identity.package_name:
            search_packages.add(search_identity.package_name)
        else:
            search_packages = get_packages_with_resource_type(
                _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR
            )

        matching_count = 0
        for package in search_packages:
            content, base_path = ament_index_python.get_resource(
                _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, package
            )

            # Parse content to find the specific resource
            for line in content.splitlines():
                parts = line.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP)
                if len(parts) != 6:
                    raise ResourceError(f"Invalid behavior resource file (Package: '{package}'). Invalid line: {line}.")

                found_category = parts[0]
                found_alias = parts[1]
                if not search_identity.category_name:
                    # Disregard the category if not provided with the identity
                    if found_alias != search_identity.behavior_alias:
                        continue
                elif found_category != search_identity.category_name or found_alias != search_identity.behavior_alias:
                    continue

                # Found matching resource - Increase counter
                matching_count += 1

                # Now fill the other member variables in case the resource matches (if match is not unique, error is thrown later and the object is discarded)

                # Store behavior category
                self._unique_identity.category_name = found_category

                # Store behavior alias
                self._unique_identity.behavior_alias = found_alias

                # Store package name
                self._unique_identity.package_name = package

                # Store default build handler
                self._default_build_handler = parts[2]

                # Store build request
                self._build_request_file_path = os.path.join(base_path, parts[3])
                if os.path.isfile(self._build_request_file_path):
                    with open(self._build_request_file_path, "r") as f:
                        self._build_request = f.read()
                else:
                    self._build_request_file_path = ""
                    self._build_request = parts[3]

                # Store entry_point
                self._entry_point = parts[4]

                # Store node manifest
                node_manifest_paths = []
                for path in parts[5].split(";"):
                    if not path:
                        continue
                    if os.path.isabs(path):
                        node_manifest_paths.append(path)
                    else:
                        node_manifest_paths.append(os.path.join(base_path, path))
                self._node_manifest = NodeManifest.from_files(node_manifest_paths)

        if matching_count == 0:
            raise ResourceError(f"No behavior resource with identity '{search_identity}' was registered.")
        if matching_count > 1:
            raise ResourceError(
                f"Behavior resource identity '{search_identity}' is ambiguous. You must be more precise."
            )

    @staticmethod
    def find(behavior_alias: str, package_name: str = "", category_name: str = "") -> "BehaviorResource":
        """
        Find a behavior resource by name.

        Args:
            behavior_alias: Name of the resource to find.
            package_name: Optional package name to narrow search.
            category_name: Optional category name to narrow search.

        Returns:
            BehaviorResource instance for the found resource.

        Raises:
            ResourceError: If resource cannot be found.
        """
        return BehaviorResource(
            category_name
            + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP
            + package_name
            + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP
            + behavior_alias
        )

    @property
    def identity(self) -> BehaviorResourceIdentity:
        """
        Get the unique identity for this resource.
        """
        return self._unique_identity

    @property
    def build_request(self) -> str:
        """
        Get the behavior build request of this resource.
        """
        return self._build_request

    @property
    def default_build_handler(self) -> str:
        """
        Get the default behavior build handler of this resource.
        """
        return self._default_build_handler

    @property
    def entry_point(self) -> str:
        """
        Get the behavior entry point for this resource.
        """
        return self._entry_point

    @property
    def node_manifest(self) -> NodeManifest:
        """
        Get the behavior tree node manifest of this resource.
        """
        return self._node_manifest


class TreeResourceIdentity(BehaviorResourceIdentity):
    """
    Struct that encapsulates the identity string for a registered behavior tree.

    This is the Python equivalent of auto_apms_behavior_tree::core::TreeResourceIdentity.

    The identity string is formatted like ``<package_name>::<tree_file_stem>::<tree_name>``.
    Since TreeResource uses the same identity resolution as BehaviorResource
    (where ``<behavior_alias>`` = ``<tree_file_stem>::<tree_name>``), all standard short forms apply.
    Both ``<tree_file_stem>`` and ``<tree_name>`` must always be provided.
    """

    def __init__(self, identity: str = None):
        """
        Initialize a tree resource identity from an identity string.

        Identity must be formatted like ``<package_name>::<tree_file_stem>::<tree_name>``.
        Both ``<tree_file_stem>`` and ``<tree_name>`` must be provided.

        Args:
            identity: Identity string for a specific behavior tree resource, or None for manual creation.
        """
        self.file_stem = ""
        self.tree_name = ""

        if identity is None:
            super().__init__()
            return

        # First, let the parent class parse the basic structure
        super().__init__(identity, _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE)

        # Parse the behavior alias part to extract file_stem and tree_name
        tokens = self.behavior_alias.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP)

        if len(tokens) == 2:
            self.file_stem = tokens[0]
            self.tree_name = tokens[1]
        elif len(tokens) == 1:
            self.file_stem = tokens[0]
            self.tree_name = ""

        if not self.file_stem or not self.tree_name:
            raise ResourceIdentityFormatError(
                f"Behavior tree resource identity string '{identity}' is invalid. "
                "Both <tree_file_stem> and <tree_name> must be provided."
            )


class TreeResource(BehaviorResource):
    """
    Class containing behavior tree resource data.

    This is the Python equivalent of auto_apms_behavior_tree::core::TreeResource.

    TreeResource inherits from BehaviorResource and uses the same identity resolution for its constructor.
    The underlying behavior alias for a tree resource is ``<tree_file_stem>::<tree_name>``,
    so all identity formats supported by BehaviorResource apply uniformly.

    For convenience, the static methods :meth:`find_by_tree_name` and :meth:`find_by_file_stem` allow searching
    by just one component of the alias without requiring the full ``<tree_file_stem>::<tree_name>``.
    """

    def __init__(self, identity: TreeResourceIdentity | str):
        """Initialize a tree resource from an identity object or string."""
        if isinstance(identity, str):
            identity = TreeResourceIdentity(identity)
        elif not isinstance(identity, TreeResourceIdentity):
            raise TypeError("Identity must be a TreeResourceIdentity object or a string.")

        # Initialize the base class
        super().__init__(identity)

        # Fill the tree specific fields for the unique identity
        tokens = self._unique_identity.behavior_alias.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP)
        if len(tokens) != 2:
            raise ResourceIdentityFormatError(
                f"Unique tree resource identity string '{self._unique_identity}' is invalid. "
                f"Behavior alias must be <tree_file_stem>{_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP}<tree_name>."
            )

        self._unique_identity.file_stem = tokens[0]
        self._unique_identity.tree_name = tokens[1]

    @staticmethod
    def find_by_tree_name(tree_name: str, package_name: str = "") -> "TreeResource":
        """
        Find an installed behavior tree resource using a specific behavior tree name.

        This method searches the ament resource index for a tree resource whose ``<tree_name>`` component matches.
        Unlike the identity-string constructor, this accepts just the tree name without requiring ``<tree_file_stem>``.

        Args:
            tree_name: The name of the behavior tree.
            package_name: Optional package name to narrow the search. If empty, all packages are searched.

        Returns:
            The matching behavior tree resource.

        Raises:
            ResourceError: If no matching resource is found or if the match is ambiguous.
        """
        if not tree_name:
            raise ResourceError("Cannot find tree resource: tree_name must not be empty.")

        search_packages: set[str] = set()
        if package_name:
            search_packages.add(package_name)
        else:
            search_packages = get_packages_with_resource_type(
                _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR
            )

        match_identity = None
        matching_count = 0
        for pkg in search_packages:
            try:
                content, _ = ament_index_python.get_resource(
                    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, pkg
                )
            except (LookupError, ament_index_python.InvalidResourceNameError):
                continue

            for line in content.splitlines():
                parts = line.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP)
                if len(parts) != 6:
                    continue

                found_category = parts[0]
                found_alias = parts[1]

                if found_category != _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE:
                    continue

                alias_tokens = found_alias.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP)
                if len(alias_tokens) != 2:
                    continue

                if alias_tokens[1] == tree_name:
                    matching_count += 1
                    match_identity = TreeResourceIdentity()
                    match_identity.category_name = found_category
                    match_identity.package_name = pkg
                    match_identity.behavior_alias = found_alias
                    match_identity.file_stem = alias_tokens[0]
                    match_identity.tree_name = alias_tokens[1]

        if matching_count == 0:
            suffix = "." if not package_name else f" in package '{package_name}'."
            raise ResourceError(f"No tree resource with tree name '{tree_name}' was found{suffix}")
        if matching_count > 1:
            raise ResourceError(
                f"Tree name '{tree_name}' is ambiguous ({matching_count} matches). You must be more precise."
            )

        return TreeResource(match_identity)

    @staticmethod
    def find_by_file_stem(file_stem: str, package_name: str = "") -> "TreeResource":
        """
        Find an installed behavior tree resource using an XML file stem.

        This method searches the ament resource index for a tree resource whose ``<tree_file_stem>`` component matches.
        Unlike the identity-string constructor, this accepts just the file stem without requiring ``<tree_name>``.

        Args:
            file_stem: The stem (filename without extension) of the behavior tree XML file.
            package_name: Optional package name to narrow the search. If empty, all packages are searched.

        Returns:
            The matching behavior tree resource.

        Raises:
            ResourceError: If no matching resource is found or if the match is ambiguous.
        """
        if not file_stem:
            raise ResourceError("Cannot find tree resource: file_stem must not be empty.")

        search_packages: set[str] = set()
        if package_name:
            search_packages.add(package_name)
        else:
            search_packages = get_packages_with_resource_type(
                _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR
            )

        match_identity = None
        matching_count = 0
        for pkg in search_packages:
            try:
                content, _ = ament_index_python.get_resource(
                    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, pkg
                )
            except (LookupError, ament_index_python.InvalidResourceNameError):
                continue

            for line in content.splitlines():
                parts = line.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP)
                if len(parts) != 6:
                    continue

                found_category = parts[0]
                found_alias = parts[1]

                if found_category != _AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY__TREE:
                    continue

                alias_tokens = found_alias.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP)
                if len(alias_tokens) != 2:
                    continue

                if alias_tokens[0] == file_stem:
                    matching_count += 1
                    match_identity = TreeResourceIdentity()
                    match_identity.category_name = found_category
                    match_identity.package_name = pkg
                    match_identity.behavior_alias = found_alias
                    match_identity.file_stem = alias_tokens[0]
                    match_identity.tree_name = alias_tokens[1]

        if matching_count == 0:
            suffix = "." if not package_name else f" in package '{package_name}'."
            raise ResourceError(f"No tree resource with file stem '{file_stem}' was found{suffix}")
        if matching_count > 1:
            raise ResourceError(
                f"File stem '{file_stem}' is ambiguous ({matching_count} matches). You must be more precise."
            )

        return TreeResource(match_identity)


def get_node_manifest_resource_identities(exclude_packages: set[str] = None) -> set[NodeManifestResourceIdentity]:
    """
    Retrieves all available/installed behavior tree node manifest resource identities.

    This function scans all packages that have registered node manifest resources,
    parses their resource files, and constructs a list of `NodeManifestResourceIdentity` objects
    representing each available node manifest resource.

    Args:
        exclude_packages: Packages to exclude when searching for resources. If empty, all packages are included.

    Returns:
        List of node manifest resource identities.
    """
    identities = set()
    for package in get_packages_with_resource_type(
        _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST, exclude_packages
    ):
        content, _ = ament_index_python.get_resource(
            _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST, package
        )
        for line in content.splitlines():
            parts = line.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP)
            if len(parts) > 0:
                i = NodeManifestResourceIdentity()
                i.package_name = package
                i.metadata_id = parts[0]
                identities.add(i)
    return sorted(identities)


def get_behavior_resource_identities(
    include_categories: set[str] = None, include_internal=False, exclude_packages: set[str] = None
) -> set[BehaviorResourceIdentity]:
    """
    Retrieves all available/installed behavior resource identities.

    This function scans all packages that have registered behavior resources,
    parses their resource files, and constructs a list of `BehaviorResourceIdentity` objects
    representing each available behavior resource.

    Args:
        include_categories: Optional set of categories to include in the results. If empty, all categories are included.
        include_internal: Flag whether to include behaviors marked as internal.
        exclude_packages: Packages to exclude when searching for resources. If empty, all packages are included.

    Returns:
        List of behavior resource identities.
    """
    identities = set()
    for package in get_packages_with_resource_type(
        _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, exclude_packages
    ):
        content, _ = ament_index_python.get_resource(
            _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, package
        )
        for line in content.splitlines():
            parts = line.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP)
            if len(parts) > 1:
                i = BehaviorResourceIdentity()
                i.category_name = parts[0]
                i.package_name = package
                i.behavior_alias = parts[1]
                if include_categories:
                    if i.category_name not in include_categories:
                        continue
                elif not include_internal and i.category_name.endswith(
                    _AUTO_APMS_BEHAVIOR_TREE_CORE__INTERNAL_BEHAVIOR_CATEGORY_SUFFIX
                ):
                    continue
                identities.add(i)
    return sorted(identities)


def get_behavior_categories():
    """Get all behavior categories that are registered in the system."""
    return {i.category_name for i in get_behavior_resource_identities()}


def get_behavior_tree_node_plugins(exclude_packages: set[str] = None, *, per_package=False):
    """
    Get all behavior tree node plugin names.

    This is a convenience function that finds all plugins with the behavior tree node
    base class type.

    Args:
        exclude_packages: Packages to exclude when searching for plugins.
        per_package: If True, returns a dictionary with package names as keys and lists of plugin names as values.

    Returns:
        List of all behavior tree node plugin names.

    Raises:
        ResourceError: If failed to find or parse plugin manifest files.
    """
    return get_plugin_names_with_base_type(
        BASE_CLASS_TYPE__BEHAVIOR_TREE_NODE, exclude_packages, per_package=per_package
    )
