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

import yaml
import rclpy
import signal
import threading

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, CancelGoal, GoalStatus
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import SetLoggerLevels
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from auto_apms_interfaces.action import StartTreeExecutor
from rclpy.logging import LoggingSeverity
from ros2cli.node.strategy import DirectNode, NodeStrategy
from ros2cli.node import HIDDEN_NODE_PREFIX
from ros2param.api import call_get_parameters, call_set_parameters, get_value
from ros2run.api import run_executable, get_executable_path
from .resources import BehaviorResource, NodeManifest

GENERIC_NODE_NAME = HIDDEN_NODE_PREFIX + "auto_apms_behavior_tree"


def find_start_tree_executor_actions() -> list[str]:
    """
    Returns:
        list: list of action names for all available StartTreeExecutor actions.
    """
    action_type = "auto_apms_interfaces/action/StartTreeExecutor"
    actions = []
    with NodeStrategy(None) as node:
        action_list = node.get_action_names_and_types()
        for action_name, types in action_list:
            if action_type in types:
                actions.append(action_name)
    return actions


def call_clear_blackboard_for_executor(node: Node, node_name: str):
    """
    Clears the behavior tree blackboard of an executor.
    """
    client = node.create_client(Trigger, f"{node_name}/clear_blackboard")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"clear_blackboard service for node '{node_name}' not available after 5 seconds")
    request = Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.done():
        response: Trigger.Response = future.result()
        if not response.success:
            node.get_logger().error(f"Failed to clear blackboard for node '{node_name}': {response.message}")
    else:
        raise RuntimeError(f"clear_blackboard service call for node '{node_name}' did not complete in time")


def call_set_logger_level(node: Node, node_name: str, logging: LoggingSeverity):
    """
    Set the logger level for the executor node.
    """
    client = node.create_client(SetLoggerLevels, f"{node_name}/set_logger_levels")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"set_logger_levels service for node '{node_name}' not available after 5 seconds")
    request = SetLoggerLevels.Request()
    request.levels = [LoggerLevel(name=node_name, level=logging)]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.done():
        response: SetLoggerLevels.Response = future.result()
        for res in response.results:
            if res.successful:
                node.get_logger().info(f"Logger level set to {logging.name} for node '{node_name}'")
            else:
                node.get_logger().error(f"Failed to set logger level for node '{node_name}': {res.reason}")
    else:
        raise RuntimeError(f"set_logger_levels service call for node '{node_name}' did not complete in time")


def call_set_parameters_for_executor(node: Node, node_name: str, static_params, blackboard_params):
    """
    Set static and blackboard parameters on the executor node using ros2param.api.
    """
    parameters = []
    # Static params
    for k, v in (static_params or {}).items():
        parameters.append(Parameter(name=k, value=v))
    # Blackboard params (with bb. prefix)
    for k, v in (blackboard_params or {}).items():
        parameters.append(Parameter(name=f"bb.{k}", value=v))

    try:
        result = call_set_parameters(
            node=node,
            node_name=node_name,
            parameters=[p.to_parameter_msg() for p in parameters],
        )
        # Evaluate and print result for each parameter
        for idx, res in enumerate(result.results):
            pname = parameters[idx].name if idx < len(parameters) else "<unknown>"
            if not res.successful:
                msg = f"Set parameter '{pname}' failed"
                if res.reason:
                    msg += f": {res.reason}"
                node.get_logger().warning(msg)
    except Exception as e:
        raise RuntimeError(f"Exception while setting parameters on node '{node_name}': {e}") from e


def call_start_tree_action(
    node: Node,
    action_name: str,
    build_request: str,
    build_handler: str = None,
    entry_point: str = None,
    node_manifest: NodeManifest = None,
    clear_blackboard: bool = False,
    timeout_sec=5.0,
):
    """
    Send the tree XML to the specified executor node using the StartTreeExecutor action.
    """
    action_client = ActionClient(node, StartTreeExecutor, action_name)
    last_running_action_ts = -1.0
    last_running_action_name = ""

    def feedback_callback(feedback_msg):
        nonlocal last_running_action_ts, last_running_action_name
        feedback: StartTreeExecutor.Feedback = feedback_msg.feedback
        if last_running_action_ts < 0:
            node.get_logger().info(f"Root Behavior Tree is '{feedback.running_tree_identity}'")
            last_running_action_ts = 0.0
        if feedback.running_action_timestamp > last_running_action_ts:
            if feedback.running_action_name != last_running_action_name:
                last_running_action_name = feedback.running_action_name
                node.get_logger().info(f"» Current action: {feedback.running_action_name}")
            last_running_action_ts = feedback.running_action_timestamp

    if not action_client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError(
            f"Action server '{action_name}' not available after 5 seconds. " "Make sure the executor node is running!"
        )

    goal_msg = StartTreeExecutor.Goal(check_fields=True)
    goal_msg.build_request = build_request or ""
    if build_handler:
        goal_msg.build_handler = build_handler
    if entry_point:
        goal_msg.entry_point = entry_point
    if node_manifest:
        goal_msg.node_manifest = node_manifest.dump()
    goal_msg.clear_blackboard = clear_blackboard
    goal_msg.attach = True

    send_goal_future = action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=timeout_sec)
    if not send_goal_future.done():
        node.get_logger().error("Timed out waiting for goal response from executor")
        return -1
    goal_handle: ClientGoalHandle = send_goal_future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Goal was rejected by executor!")
        return -1

    get_result_future = goal_handle.get_result_async()
    cancel_requested = threading.Event()

    def signal_handler(sig, frame):
        cancel_requested.set()

    # Register signal handler for Ctrl+C
    original_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not get_result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if cancel_requested.is_set():
                node.get_logger().warning("Requesting cancellation...")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=timeout_sec)
                if cancel_future.done():
                    cancel_response: CancelGoal.Response = cancel_future.result()
                    if cancel_response is None:
                        node.get_logger().error(
                            "Exception while canceling goal: {!r}".format(cancel_future.exception())
                        )
                    if len(cancel_response.goals_canceling) == 0:
                        node.get_logger().error("Failed to cancel goal")
                else:
                    node.get_logger().error("Timed out waiting for cancel response from executor")
                    return -1
                break
    finally:
        # Restore original signal handler
        signal.signal(signal.SIGINT, original_handler)

    rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=timeout_sec)
    if not get_result_future.done():
        node.get_logger().error("Timed out waiting for result response from executor")
        return -1

    result: StartTreeExecutor.Result = get_result_future.result().result
    status: GoalStatus = get_result_future.result().status

    return_code = 0
    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info(f"Behavior Tree SUCCEEDED!")
    elif status == GoalStatus.STATUS_CANCELED:
        node.get_logger().info("Behavior Tree CANCELED!")
    else:
        node.get_logger().info(f"Behavior Tree FAILED!")
        return_code = 1
    if result.message:
        node.get_logger().info(result.message)

    return return_code


def sync_run_generic_behavior_with_executor(
    action_name: str,
    build_request: str | None,
    build_handler: str | None = None,
    entry_point: str | None = None,
    node_manifest: NodeManifest = None,
    executor_node_name: str | None = None,
    static_params: dict = None,
    blackboard_params: dict = None,
    keep_blackboard: bool = False,
    logging_level: LoggingSeverity = None,
):
    """
    Execute a behavior on a remote executor node.

    Args:
        action_name: Name of the StartTreeExecutor action to call for starting the behavior execution
            (e.g. '/my_executor/start_tree_executor).
        build_request: A behavior build request string or None for letting the build handler
            decide how to build the tree without a specific build request
        build_handler: Optional build handler to use for building the behavior
        entry_point: Optional entry point to use for building the behavior
        node_manifest: Optional node manifest that provides registration options for the behavior tree nodes required
            by the behavior.
        executor_node_name: Name of the behavior tree executor node (e.g. 'my_executor').
            If specified, the function will attempt to configure the node before execution.
            If None, `static_params`, `blackboard_params`, `keep_blackboard` and `logging_level` arguments will be
            ignored.
        static_params: Static parameters to set on the executor
        blackboard_params: Blackboard parameters to set on the executor
        keep_blackboard: Do not clear the blackboard before execution
        logging_level: Logger level to set on the executor
    """
    with DirectNode(None, node_name=GENERIC_NODE_NAME) as node:
        node.get_logger().info(f"Targeting behavior execution action '{action_name}'")

        timeout_sec = 5.0
        if executor_node_name:
            node.get_logger().info(f"Configuring executor node '{executor_node_name}'")
            # Set logger level if requested
            if logging_level is not None:
                call_set_logger_level(node, executor_node_name, logging_level)
            # Clear blackboard if not instructed otherwise
            if not keep_blackboard:
                call_clear_blackboard_for_executor(node, executor_node_name)
            # Set parameters before sending the goal
            if static_params or blackboard_params:
                call_set_parameters_for_executor(node, executor_node_name, static_params, blackboard_params)
            if static_params and "tick_rate" in static_params:
                tick_rate = static_params["tick_rate"]
            else:
                response: GetParameters.Response = call_get_parameters(
                    node=node, node_name=executor_node_name, parameter_names=["tick_rate"]
                )
                tick_rate = get_value(parameter_value=response.values[0])
            timeout_sec = max(tick_rate * 2.5, 5.0)

        return call_start_tree_action(
            node,
            action_name,
            build_request=build_request,
            build_handler=build_handler,
            entry_point=entry_point,
            node_manifest=node_manifest,
            clear_blackboard=False,
            timeout_sec=timeout_sec,
        )


def sync_run_behavior_resource_with_executor(
    executor_name: str,
    behavior: BehaviorResource | None,
    static_params: dict = None,
    blackboard_params: dict = None,
    keep_blackboard: bool = False,
    logging_level: LoggingSeverity = None,
):
    """
    Execute a behavior on a remote executor node.

    Args:
        executor_name: Name of the behavior tree executor
        behavior: A valid behavior resource or None for letting the build handler
            decide how to build the tree without a specific build request
        static_params: Static parameters to set on the executor
        blackboard_params: Blackboard parameters to set on the executor
        keep_blackboard: Do not clear the blackboard before execution
        logging_level: Logger level to set on the executor
    """
    return sync_run_generic_behavior_with_executor(
        executor_name=executor_name,
        build_request=behavior.build_request if behavior else None,
        build_handler=behavior.default_build_handler if behavior else None,
        entry_point=behavior.entry_point if behavior else None,
        node_manifest=behavior.node_manifest if behavior else None,
        static_params=static_params,
        blackboard_params=blackboard_params,
        keep_blackboard=keep_blackboard,
        logging_level=logging_level,
    )


def sync_run_generic_behavior_locally(
    build_request: str | None,
    build_handler: str | None = None,
    entry_point: str | None = None,
    node_manifest: NodeManifest = None,
    static_params: dict = None,
    blackboard_params: dict = None,
    logging_level: LoggingSeverity = None,
):
    """
    Execute a behavior locally using the run_behavior executable.

    Args:
        build_request: A behavior build request string or None for letting the build handler
            decide how to build the tree without a specific build request
        build_handler: Build handler to use for building the behavior
        entry_point: Optional entry point to use for building the behavior
        node_manifest: Optional node manifest that provides registration options for the behavior tree nodes required
            by the behavior.
        static_params: Static parameters to set on the executor
        blackboard_params: Blackboard parameters to set on the executor
        logging_level: Logger level to set on the executor
    """
    required_package = "auto_apms_behavior_tree"
    required_command = "run_behavior"
    static_params = static_params or {}
    argv = []
    argv.append(build_request if build_request else "")
    argv.append(entry_point if entry_point else "")
    argv.append(node_manifest.dump() if node_manifest else "")

    # Add ros args
    argv.append("--ros-args")

    def add_ros_argument(arg_name: str, arg_tuple: tuple[str, str]):
        argv.append(f"--{arg_name}")
        argv.append(f"{arg_tuple[0]}:={arg_tuple[1]}")

    if logging_level:
        add_ros_argument("log-level", (required_command, logging_level.name))

    # Add static parameter for setting the build handler
    if build_handler:
        # We override the build handler that might be passed via this dict with the value of the dedicated argument
        static_params["build_handler"] = build_handler

    if static_params or blackboard_params:
        for tup in static_params.items():
            add_ros_argument("param", tup)
        for k, v in (blackboard_params or {}).items():
            add_ros_argument("param", (f"bb.{k}", v))

    return run_executable(
        path=get_executable_path(
            package_name=required_package,
            executable_name=required_command,
        ),
        argv=argv,
    )


def sync_run_behavior_resource_locally(
    behavior: BehaviorResource | None,
    static_params: dict = None,
    blackboard_params: dict = None,
    logging_level: LoggingSeverity = None,
):
    """
    Execute a behavior locally using the run_behavior executable.

    Args:
        behavior: A valid behavior resource
        static_params: Static parameters to set on the executor
        blackboard_params: Blackboard parameters to set on the executor
        logging_level: Logger level to set on the executor
    """
    return sync_run_generic_behavior_locally(
        build_request=behavior.build_request if behavior else None,
        build_handler=behavior.default_build_handler if behavior else None,
        entry_point=behavior.entry_point if behavior else None,
        node_manifest=behavior.node_manifest if behavior else None,
        static_params=static_params,
        blackboard_params=blackboard_params,
        logging_level=logging_level,
    )


def sync_run_tree_node_locally(
    node_name: str,
    registration_options: dict,
    port_values: dict = None,
    logging_level: LoggingSeverity = None,
):
    """
    Run a tree node locally using the run_tree_node executable.

    Args:
        node_name: The registration name of the node to run
        registration_params: YAML representation of NodeRegistrationOptions encoded in a string
        port_values: Optional YAML map of specific node port values encoded in a string
        logging_level: Logger level to set on the executor
    """
    required_package = "auto_apms_behavior_tree"
    required_command = "run_tree_node"

    argv = [yaml.dump(registration_options)]
    if port_values:
        argv.append(yaml.dump(port_values))

    # Add ros args
    argv.append("--ros-args")

    def add_ros_argument(arg_name: str, arg_tuple: tuple[str, str]):
        argv.append(f"--{arg_name}")
        argv.append(f"{arg_tuple[0]}:={arg_tuple[1]}")

    add_ros_argument("remap", ("__node", node_name))

    if logging_level:
        add_ros_argument("log-level", (node_name, logging_level.name))

    return run_executable(
        path=get_executable_path(
            package_name=required_package,
            executable_name=required_command,
        ),
        argv=argv,
    )
