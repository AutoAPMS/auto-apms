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
from auto_apms_behavior_tree.scripting import sync_run_generic_behavior_with_executor, find_start_tree_executor_actions
from ..verb import VerbExtension
from ..api import (
    add_behavior_resource_argument_to_parser,
    parse_key_value_args,
    PrefixFilteredChoicesCompleter,
)


class SendVerb(VerbExtension):
    """
    Send a behavior to a running executor and start executing.

    There are multiple ways to define the behavior to send:

    1. Use an existing behavior resource
    2. Manually define the behavior using keyword arguments
    3. Combine both approaches for overriding individual components of the given behavior resource.
    """

    def add_arguments(self, parser, cli_name):
        """Add arguments for the send verb."""
        parser.description = inspect.cleandoc(self.__doc__)

        # Discover available StartTreeExecutor actions and try to guess executor name
        executor_actions = find_start_tree_executor_actions()
        self.action_executor_mapping = {}
        for action in executor_actions:
            stripped = action.strip("/")
            if "/" in stripped:
                # Guess the executor name from the action name by taking the first part
                # of the action name before the first slash
                self.action_executor_mapping[action] = stripped.split("/")[0]

        action_arg = parser.add_argument(
            "action_name",
            type=str,
            help="Name of the StartTreeExecutor action to send the behavior to",
        )
        action_arg.completer = PrefixFilteredChoicesCompleter(executor_actions)
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
        parser.add_argument(
            "--entry-point",
            type=str,
            help="Entry point to pass to the build handler. If a behavior resource identity is given as a positional argument, override the associated entry point",
        )
        manifest_arg = parser.add_argument(
            "--node-manifest",
            type=NodeManifestResource,
            help="Node manifest resource to pass to the build handler. If a behavior resource identity is given, override the associated node manifest",
            metavar="IDENTITY",
        )
        manifest_arg.completer = PrefixFilteredChoicesCompleter(get_node_manifest_resource_identities())
        parser.add_argument(
            "-e",
            "--executor",
            type=str,
            help="Name of the behavior executor node that implements the StartTreeExecutor action (guessed from action_name if omitted)",
        )
        parser.add_argument(
            "--blackboard",
            nargs="*",
            metavar="key:=value",
            help="Blackboard variables to pass to the behavior tree",
            default=[],
        )
        parser.add_argument(
            "--keep-blackboard",
            action="store_true",
            help="Do not explicitly clean the blackboard of the executor before execution",
        )
        parser.add_argument(
            "--tick-rate",
            type=float,
            help="Tick rate for the behavior tree in seconds (keeps current setting if omitted)",
        )
        parser.add_argument(
            "--groot2-port",
            type=int,
            help="Port for Groot2 (keeps current setting if omitted)",
        )
        parser.add_argument(
            "--state-change-logger",
            type=bool,
            help="Enable/Disable the state change logger (keeps current setting if omitted)",
            metavar="true/false 1/0 yes/no",
        )
        logging_level_names = [enum.name.lower() for enum in LoggingSeverity]
        logging_arg = parser.add_argument(
            "--logging",
            type=get_logging_severity_from_string,
            choices=LoggingSeverity,
            help="Set the logger level for the executor node before starting the tree (keeps current setting if omitted)",
            metavar=logging_level_names,
        )
        logging_arg.completer = PrefixFilteredChoicesCompleter(logging_level_names)

    def main(self, *, args):
        """Main function for the send verb."""
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

        # Collect static parameters if specified
        static_params = {}
        if args.tick_rate is not None:
            static_params["tick_rate"] = args.tick_rate
        if args.groot2_port is not None:
            static_params["groot2_port"] = args.groot2_port
        if args.state_change_logger is not None:
            static_params["state_change_logger"] = args.state_change_logger

        # Parse blackboard key-value pairs
        blackboard_params = parse_key_value_args(args.blackboard)

        # If executor_node_name is omitted, populate it by guessing from the action name (if possible).
        # If the action name does not follow the typical pattern of '/executor_name/start_tree_executor',
        # executor_node_name will be None and the function will proceed without attempting to configure the
        # executor node.
        # If the executor node configuration step shall be skipped explicitly, the user may set the executor
        # argument to an empty string.
        action_name = args.action_name
        if args.executor is None:
            executor_node_name = self.action_executor_mapping.get(action_name, None)
        elif args.executor == "":
            executor_node_name = None
        else:
            executor_node_name = args.executor

        return sync_run_generic_behavior_with_executor(
            action_name=action_name,
            build_request=build_request,
            build_handler=build_handler,
            entry_point=entry_point,
            node_manifest=node_manifest,
            executor_node_name=executor_node_name,
            static_params=static_params,
            blackboard_params=blackboard_params,
            keep_blackboard=args.keep_blackboard,
            logging_level=args.logging,
        )
