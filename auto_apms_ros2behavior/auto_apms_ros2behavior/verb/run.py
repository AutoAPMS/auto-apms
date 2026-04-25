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

import argparse
import inspect

from rclpy.logging import LoggingSeverity, get_logging_severity_from_string
from auto_apms_behavior_tree.resources import get_behavior_build_handler_plugins
from auto_apms_behavior_tree_core.resources import NodeManifestResource, get_node_manifest_resource_identities
from auto_apms_behavior_tree.scripting import sync_run_generic_behavior_locally
from ..verb import VerbExtension
from ..api import (
    add_behavior_resource_argument_to_parser,
    parse_key_value_args,
    PrefixFilteredChoicesCompleter,
)


class RunVerb(VerbExtension):
    """
    Execute a behavior locally.

    There are multiple ways to define the behavior to execute:

    1. Use an existing behavior resource
    2. Manually define the behavior using keyword arguments
    3. Combine both approaches for overriding individual components of the given behavior resource.
    """

    def add_arguments(self, parser, cli_name):
        """Add arguments for the run verb."""
        parser.description = inspect.cleandoc(self.__doc__)

        behavior_arg = add_behavior_resource_argument_to_parser(parser)
        behavior_arg.nargs = "?"  # Make the behavior argument optional
        parser.add_argument(
            "--build-request",
            type=str,
            help="Build request to be passed to the build handler. If a behavior resource identity is given, override the associated build request",
        )
        build_handler_arg = parser.add_argument(
            "--build-handler",
            type=str,
            help="Build handler to load. If a behavior resource identity is given as a positional argument, override the associated build handler",
            metavar="<namespace>::<class_name>",
        )
        build_handler_arg.completer = PrefixFilteredChoicesCompleter(get_behavior_build_handler_plugins())
        # New canonical option name with hyphen, consistent with other CLI arguments
        parser.add_argument(
            "--entry-point",
            dest="entry_point",
            type=str,
            help="Entry point to pass to the build handler. If a behavior resource identity is given as a positional argument, override the associated entry point",
        )
        # Backwards-compatible alias using the old underscore style, hidden from help output
        parser.add_argument(
            "--entry_point",
            dest="entry_point",
            type=str,
            help=argparse.SUPPRESS,
        )
        manifest_arg = parser.add_argument(
            "--node-manifest",
            type=NodeManifestResource,
            help="Node manifest resource to pass to the build handler. If a behavior resource identity is given, override the associated node manifest",
            metavar="IDENTITY",
        )
        manifest_arg.completer = PrefixFilteredChoicesCompleter(get_node_manifest_resource_identities())
        parser.add_argument(
            "--blackboard",
            nargs="*",
            metavar="key:=value",
            help="Blackboard variables to pass to the behavior tree",
            default=[],
        )
        parser.add_argument(
            "--tick-rate",
            type=float,
            help="Tick rate for the behavior tree in seconds",
        )
        parser.add_argument(
            "--groot2-port",
            type=int,
            help="Port for Groot2 (disabled by default)",
        )
        parser.add_argument(
            "--state-change-logger",
            action="store_true",
            help="Enable the state change logger",
        )
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
            help="ROS 2 node namespace for the executor",
        )

    def main(self, *, args):
        """Main function for the run verb."""
        if not (args.behavior or args.build_handler):
            raise argparse.ArgumentError(None, "Either a behavior resource or a build handler must be specified.")

        build_request = args.behavior.build_request if args.behavior else None
        if args.build_request:
            build_request = args.build_request
        build_handler = args.behavior.default_build_handler if args.behavior else None
        if args.build_handler:
            build_handler = args.build_handler
        entry_point = args.behavior.entry_point if args.behavior else None
        if args.entry_point:
            entry_point = args.entry_point
        node_manifest = args.behavior.node_manifest if args.behavior else None
        if args.node_manifest:
            node_manifest = args.node_manifest

        static_params = {}
        if args.tick_rate is not None:
            static_params["tick_rate"] = args.tick_rate
        if args.groot2_port is not None:
            static_params["groot2_port"] = args.groot2_port
        if args.state_change_logger is not None:
            static_params["state_change_logger"] = args.state_change_logger

        # Parse blackboard key-value pairs from args.blackboard
        blackboard_params = parse_key_value_args(args.blackboard)

        print(
            f"--- Running behavior '{args.behavior.identity}'"
            if args.behavior
            else "--- Running behavior (no identity provided)"
        )
        return sync_run_generic_behavior_locally(
            build_request=build_request,
            build_handler=build_handler,
            entry_point=entry_point,
            node_manifest=node_manifest,
            static_params=static_params,
            blackboard_params=blackboard_params,
            logging_level=args.logging,
            namespace=args.namespace,
        )
