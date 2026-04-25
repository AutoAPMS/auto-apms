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


from rclpy.logging import LoggingSeverity, get_logging_severity_from_string
from auto_apms_behavior_tree_core.resources import NodeManifestResource, get_node_manifest_resource_identities
from auto_apms_behavior_tree.scripting import sync_run_tree_node_locally
from ...verb import VerbExtension
from ...api import (
    PrefixFilteredChoicesCompleter,
    NodeManifestFilteredRegistrationNameCompleter,
    NodePortValuesCompleter,
    add_dynamic_node_help_action,
    parse_key_value_args,
)


class CallVerb(VerbExtension):
    """Execute a single behavior tree node."""

    def add_arguments(self, parser, cli_name):
        add_dynamic_node_help_action(parser, "manifest", "node_name")
        manifest_arg = parser.add_argument(
            "manifest",
            type=NodeManifestResource,
            help="Identity string of the node manifest to use for registering node_name.",
        )
        manifest_arg.completer = PrefixFilteredChoicesCompleter(get_node_manifest_resource_identities())
        node_name_arg = parser.add_argument(
            "node_name",
            type=str,
            help="Registration name of the node to call.",
        )
        node_name_arg.completer = NodeManifestFilteredRegistrationNameCompleter("manifest")
        port_values_arg = parser.add_argument(
            "port_values",
            nargs="*",
            metavar="port:=value",
            help="Port values to pass to the node (use -h after specifying manifest and node_name to see available ports)",
            default=[],
        )
        port_values_arg.completer = NodePortValuesCompleter("manifest", "node_name")
        logging_level_names = [enum.name.lower() for enum in LoggingSeverity]
        logging_arg = parser.add_argument(
            "--logging",
            type=get_logging_severity_from_string,
            choices=LoggingSeverity,
            help="Set the logger level for the executor node before starting the tree",
            metavar=logging_level_names,
        )
        logging_arg.completer = PrefixFilteredChoicesCompleter(logging_level_names)
        parser.add_argument(
            "--namespace",
            "-n",
            type=str,
            help="ROS 2 node namespace for the tree node",
        )

    def main(self, *, args):
        registration_options = args.manifest.node_manifest.get_node_registration_options(args.node_name)
        print(f"--- Running behavior tree node {args.node_name} ({registration_options["class_name"]})")
        return sync_run_tree_node_locally(
            node_name=args.node_name,
            registration_options=registration_options,
            port_values=parse_key_value_args(args.port_values),
            logging_level=args.logging,
            namespace=args.namespace,
        )
