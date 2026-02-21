// Copyright 2026 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_behavior_tree/executor/generic_event_based_executor.hpp"

#include <algorithm>
#include <functional>
#include <regex>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/util/parameter.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"
#include "pluginlib/exceptions.hpp"

namespace auto_apms_behavior_tree
{

GenericEventBasedTreeExecutor::GenericEventBasedTreeExecutor(const std::string & name, Options options)
: TreeExecutorBase(std::make_shared<rclcpp::Node>(name, options.getROSNodeOptions())),
  executor_options_(options),
  executor_param_listener_(node_ptr_)
{
  // Set custom parameter default values.
  std::vector<rclcpp::Parameter> new_default_parameters;
  std::map<std::string, rclcpp::ParameterValue> effective_param_overrides =
    node_ptr_->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : executor_options_.custom_default_parameters_) {
    if (effective_param_overrides.find(name) == effective_param_overrides.end()) {
      new_default_parameters.push_back(rclcpp::Parameter(name, value));
    }
  }
  if (!new_default_parameters.empty()) node_ptr_->set_parameters_atomically(new_default_parameters);

  const ExecutorParameters initial_params = executor_param_listener_.get_params();

  // Create behavior tree node loader
  tree_node_loader_ptr_ = core::NodeRegistrationLoader::make_shared(
    std::set<std::string>(initial_params.node_exclude_packages.begin(), initial_params.node_exclude_packages.end()));

  // Create behavior tree build handler loader
  build_handler_loader_ptr_ = TreeBuildHandlerLoader::make_unique(
    std::set<std::string>(
      initial_params.build_handler_exclude_packages.begin(), initial_params.build_handler_exclude_packages.end()));

  // Instantiate behavior tree build handler
  if (
    initial_params.build_handler != PARAM_VALUE_NO_BUILD_HANDLER &&
    !build_handler_loader_ptr_->isClassAvailable(initial_params.build_handler)) {
    throw exceptions::TreeExecutorError(
      "Cannot load build handler '" + initial_params.build_handler +
      "' because no corresponding ament_index resource was found. Make sure that you spelled the build handler's "
      "name correctly "
      "and registered it by calling auto_apms_behavior_tree_register_build_handlers() in the CMakeLists.txt of the "
      "corresponding package.");
  }
  loadBuildHandler(initial_params.build_handler);

  // Collect scripting enum and blackboard parameters from initial parameters
  const auto initial_scripting_enums = getParameterValuesWithPrefix(SCRIPTING_ENUM_PARAM_PREFIX);
  if (!initial_scripting_enums.empty()) {
    if (executor_options_.scripting_enum_parameters_from_overrides_) {
      updateScriptingEnumsWithParameterValues(initial_scripting_enums);
    } else {
      RCLCPP_WARN(
        logger_,
        "Initial scripting enums have been provided, but the 'Scripting enums from overrides' option is disabled. "
        "Ignoring.");
    }
  }
  const auto initial_blackboard = getParameterValuesWithPrefix(BLACKBOARD_PARAM_PREFIX);
  if (!initial_blackboard.empty()) {
    if (executor_options_.blackboard_parameters_from_overrides_) {
      updateGlobalBlackboardWithParameterValues(initial_blackboard);
    } else {
      RCLCPP_WARN(
        logger_,
        "Initial blackboard entries have been provided, but the 'Blackboard from overrides' option is disabled. "
        "Ignoring.");
    }
  }

  using namespace std::placeholders;

  // Determine action/service names
  const std::string command_action_name =
    executor_options_.command_action_name_.empty()
      ? std::string(node_ptr_->get_name()) + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX
      : executor_options_.command_action_name_;
  const std::string clear_bb_service_name =
    executor_options_.clear_blackboard_service_name_.empty()
      ? std::string(node_ptr_->get_name()) + _AUTO_APMS_BEHAVIOR_TREE__CLEAR_BLACKBOARD_SERVICE_NAME_SUFFIX
      : executor_options_.clear_blackboard_service_name_;

  // Command action server (optional)
  if (executor_options_.enable_command_action_) {
    command_action_ptr_ = rclcpp_action::create_server<CommandActionContext::Type>(
      node_ptr_, command_action_name, std::bind(&GenericEventBasedTreeExecutor::handle_command_goal_, this, _1, _2),
      std::bind(&GenericEventBasedTreeExecutor::handle_command_cancel_, this, _1),
      std::bind(&GenericEventBasedTreeExecutor::handle_command_accept_, this, _1));
  }

  // Clear blackboard service (optional)
  if (executor_options_.enable_clear_blackboard_service_) {
    clear_blackboard_service_ptr_ = node_ptr_->create_service<std_srvs::srv::Trigger>(
      clear_bb_service_name, [this](
                               const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = this->clearGlobalBlackboard();
        if (response->success) {
          response->message = "Blackboard was cleared successfully";
        } else {
          response->message = "Blackboard cannot be cleared, because executor is in state " +
                              toStr(this->getExecutionState()) + " but must be idling";
        }
        RCLCPP_DEBUG_STREAM(this->logger_, response->message);
      });
  }

  // Parameter callbacks (only if parameter sync is enabled)
  if (
    executor_options_.scripting_enum_parameters_from_overrides_ ||
    executor_options_.scripting_enum_parameters_dynamic_ || executor_options_.blackboard_parameters_from_overrides_ ||
    executor_options_.blackboard_parameters_dynamic_) {
    on_set_parameters_callback_handle_ptr_ =
      node_ptr_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
        return this->on_set_parameters_callback_(parameters);
      });

    parameter_event_handler_ptr_ = std::make_shared<rclcpp::ParameterEventHandler>(node_ptr_);
    parameter_event_callback_handle_ptr_ = parameter_event_handler_ptr_->add_parameter_event_callback(
      [this](const rcl_interfaces::msg::ParameterEvent & event) { this->parameter_event_callback_(event); });
  }
}

GenericEventBasedTreeExecutor::GenericEventBasedTreeExecutor(rclcpp::NodeOptions options)
: GenericEventBasedTreeExecutor("event_based_tree_executor", Options(options))
{
}

void GenericEventBasedTreeExecutor::preBuild(
  core::TreeBuilder & /*builder*/, const std::string & /*build_request*/, const std::string & /*entry_point*/,
  const core::NodeManifest & /*node_manifest*/, TreeBlackboard & /*bb*/)
{
}

void GenericEventBasedTreeExecutor::postBuild(Tree & /*tree*/) {}

std::shared_future<GenericEventBasedTreeExecutor::ExecutionResult> GenericEventBasedTreeExecutor::startExecution(
  const std::string & build_request, const std::string & entry_point, const core::NodeManifest & node_manifest)
{
  const ExecutorParameters params = executor_param_listener_.get_params();
  return startExecution(
    makeTreeConstructor(build_request, entry_point, node_manifest), params.tick_rate, params.groot2_port);
}

GenericEventBasedTreeExecutor::ExecutorParameters GenericEventBasedTreeExecutor::getExecutorParameters() const
{
  return executor_param_listener_.get_params();
}

std::map<std::string, rclcpp::ParameterValue> GenericEventBasedTreeExecutor::getParameterValuesWithPrefix(
  const std::string & prefix)
{
  const auto res = node_ptr_->list_parameters({prefix}, 2);
  std::map<std::string, rclcpp::ParameterValue> value_map;
  for (const std::string & name_with_prefix : res.names) {
    if (const std::string suffix = stripPrefixFromParameterName(prefix, name_with_prefix); !suffix.empty()) {
      value_map[suffix] = node_ptr_->get_parameter(name_with_prefix).get_parameter_value();
    }
  }
  return value_map;
}

std::string GenericEventBasedTreeExecutor::stripPrefixFromParameterName(
  const std::string & prefix, const std::string & param_name)
{
  const std::regex reg("^" + prefix + "\\.(\\S+)");
  if (std::smatch match; std::regex_match(param_name, match, reg)) return match[1].str();
  return "";
}

bool GenericEventBasedTreeExecutor::updateScriptingEnumsWithParameterValues(
  const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate)
{
  std::map<std::string, std::string> set_successfully_map;
  for (const auto & [enum_key, pval] : value_map) {
    try {
      switch (pval.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          if (simulate) continue;
          scripting_enums_[enum_key] = static_cast<int>(pval.get<bool>());
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          if (simulate) continue;
          scripting_enums_[enum_key] = static_cast<int>(pval.get<int>());
          break;
        default:
          if (simulate) return false;
          throw exceptions::ParameterConversionError("Parameter to scripting enum conversion is not allowed.");
      }
      set_successfully_map[enum_key] = rclcpp::to_string(pval);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Error setting scripting enum from parameter %s=%s (Type: %s): %s", enum_key.c_str(),
        rclcpp::to_string(pval).c_str(), rclcpp::to_string(pval.get_type()).c_str(), e.what());
      return false;
    }
  }
  if (!set_successfully_map.empty()) {
    RCLCPP_DEBUG(
      logger_, "Updated scripting enums from parameters: { %s }",
      auto_apms_util::printMap(set_successfully_map).c_str());
  }
  return true;
}

bool GenericEventBasedTreeExecutor::updateGlobalBlackboardWithParameterValues(
  const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate)
{
  TreeBlackboard & bb = *getGlobalBlackboardPtr();
  std::map<std::string, std::string> set_successfully_map;
  for (const auto & [entry_key, pval] : value_map) {
    try {
      if (const BT::Expected<BT::Any> expected = createAnyFromParameterValue(pval)) {
        BT::Any any(expected.value());
        if (simulate) {
          if (const BT::TypeInfo * entry_info = bb.entryInfo(entry_key)) {
            if (entry_info->isStronglyTyped() && entry_info->type() != any.type()) return false;
          }
          continue;
        } else {
          bb.set(entry_key, any);
        }
      } else {
        throw exceptions::ParameterConversionError(expected.error());
      }
      translated_global_blackboard_entries_[entry_key] = pval;
      set_successfully_map[entry_key] = rclcpp::to_string(pval);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Error updating blackboard from parameter %s=%s (Type: %s): %s", entry_key.c_str(),
        rclcpp::to_string(pval).c_str(), rclcpp::to_string(pval.get_type()).c_str(), e.what());
      return false;
    }
  }
  if (!set_successfully_map.empty()) {
    RCLCPP_DEBUG(
      logger_, "Updated blackboard from parameters: { %s }", auto_apms_util::printMap(set_successfully_map).c_str());
  }
  return true;
}

void GenericEventBasedTreeExecutor::loadBuildHandler(const std::string & name)
{
  if (build_handler_ptr_ && !executor_param_listener_.get_params().allow_other_build_handlers) {
    throw std::logic_error(
      "Executor option 'Allow other build handlers' is disabled, but loadBuildHandler() was called again after "
      "instantiating '" +
      current_build_handler_name_ + "'.");
  }
  if (current_build_handler_name_ == name) return;
  if (name == PARAM_VALUE_NO_BUILD_HANDLER) {
    build_handler_ptr_.reset();
  } else {
    try {
      build_handler_ptr_ =
        build_handler_loader_ptr_->createUniqueInstance(name)->makeUnique(node_ptr_, tree_node_loader_ptr_);
    } catch (const pluginlib::CreateClassException & e) {
      throw exceptions::TreeExecutorError(
        "An error occurred when trying to create an instance of tree build handler class '" + name +
        "'. This might be because you forgot to call the AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER macro "
        "in the source file: " +
        e.what());
    } catch (const std::exception & e) {
      throw exceptions::TreeExecutorError(
        "An error occurred when trying to create an instance of tree build handler class '" + name + "': " + e.what());
    }
  }
  current_build_handler_name_ = name;
}

TreeConstructor GenericEventBasedTreeExecutor::makeTreeConstructor(
  const std::string & build_request, const std::string & entry_point, const core::NodeManifest & node_manifest)
{
  // Request the tree identity
  if (build_handler_ptr_ && !build_handler_ptr_->setBuildRequest(build_request, entry_point, node_manifest)) {
    throw exceptions::TreeBuildError(
      "Build request '" + build_request + "' was denied by '" + current_build_handler_name_ +
      "' (setBuildRequest() returned false).");
  }

  // Capture member references and copy value-type variables for the lambda
  auto & builder_ptr = builder_ptr_;
  auto & build_handler_ptr = build_handler_ptr_;
  rclcpp::Node::SharedPtr node_ptr = node_ptr_;
  core::NodeRegistrationLoader::SharedPtr tree_node_loader_ptr = tree_node_loader_ptr_;
  rclcpp::CallbackGroup::SharedPtr tree_node_waitables_cb_group = getTreeNodeWaitablesCallbackGroupPtr();
  rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor = getTreeNodeWaitablesExecutorPtr();
  std::map<std::string, int> scripting_enums = scripting_enums_;

  return [&builder_ptr, &build_handler_ptr, node_ptr, tree_node_loader_ptr, tree_node_waitables_cb_group,
          tree_node_waitables_executor, scripting_enums, build_request, entry_point, node_manifest,
          this](TreeBlackboardSharedPtr bb_ptr) {
    // Currently, BehaviorTree.CPP requires the memory allocated by the factory to persist even after the tree has
    // been created, so we make the builder a unique pointer that is only reset when a new tree is to be created. See
    // https://github.com/BehaviorTree/BehaviorTree.CPP/issues/890
    builder_ptr.reset(new core::TreeBuilder(
      node_ptr, tree_node_waitables_cb_group, tree_node_waitables_executor, tree_node_loader_ptr));

    // Allow executor to make modifications prior to building the tree
    this->preBuild(*builder_ptr, build_request, entry_point, node_manifest, *bb_ptr);

    // Make scripting enums available to tree instance
    for (const auto & [enum_key, val] : scripting_enums) builder_ptr->setScriptingEnum(enum_key, val);

    // If a build handler is specified, let it configure the builder and determine which tree is to be instantiated
    std::string instantiate_name = "";
    if (build_handler_ptr) {
      instantiate_name = build_handler_ptr->buildTree(*builder_ptr, *bb_ptr).getName();
    }

    // Finally, instantiate the tree
    Tree tree =
      instantiate_name.empty() ? builder_ptr->instantiate(bb_ptr) : builder_ptr->instantiate(instantiate_name, bb_ptr);

    // Allow executor to make modifications after building the tree, but before execution starts
    this->postBuild(tree);
    return tree;
  };
}

bool GenericEventBasedTreeExecutor::clearGlobalBlackboard()
{
  if (TreeExecutorBase::clearGlobalBlackboard()) {
    if (executor_options_.blackboard_parameters_from_overrides_ || executor_options_.blackboard_parameters_dynamic_) {
      const auto res = node_ptr_->list_parameters({BLACKBOARD_PARAM_PREFIX}, 2);
      for (const std::string & name : res.names) {
        node_ptr_->undeclare_parameter(name);
      }
    }
    return true;
  }
  return false;
}

rcl_interfaces::msg::SetParametersResult GenericEventBasedTreeExecutor::on_set_parameters_callback_(
  const std::vector<rclcpp::Parameter> & parameters)
{
  const ExecutorParameters params = executor_param_listener_.get_params();

  for (const rclcpp::Parameter & p : parameters) {
    auto create_rejected = [&p](const std::string msg) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Rejected to set " + p.get_name() + " = " + p.value_to_string() + " (Type: " + p.get_type_name() +
                      "): " + msg + ".";
      return result;
    };
    const std::string param_name = p.get_name();

    // Check if parameter is a scripting enum
    if (const std::string enum_key = stripPrefixFromParameterName(SCRIPTING_ENUM_PARAM_PREFIX, param_name);
        !enum_key.empty()) {
      if (isBusy()) {
        return create_rejected("Scripting enums cannot change while tree executor is running");
      }
      if (!executor_options_.scripting_enum_parameters_dynamic_ || !params.allow_dynamic_scripting_enums) {
        return create_rejected(
          "Cannot set scripting enum '" + enum_key + "', because the 'Dynamic scripting enums' option is disabled");
      }
      if (!updateScriptingEnumsWithParameterValues({{enum_key, p.get_parameter_value()}}, true)) {
        return create_rejected(
          "Type of scripting enum must be bool or int. Tried to set enum '" + enum_key + "' with value '" +
          p.value_to_string() + "' (Type: " + p.get_type_name() + ")");
      }
      continue;
    }

    // Check if parameter is a blackboard parameter
    if (const std::string entry_key = stripPrefixFromParameterName(BLACKBOARD_PARAM_PREFIX, param_name);
        !entry_key.empty()) {
      if (!executor_options_.blackboard_parameters_dynamic_ || !params.allow_dynamic_blackboard) {
        return create_rejected(
          "Cannot set blackboard entry '" + entry_key + "', because the 'Dynamic blackboard' option is disabled");
      }
      if (!updateGlobalBlackboardWithParameterValues({{entry_key, p.get_parameter_value()}}, true)) {
        return create_rejected(
          "Type of blackboard entries must not change. Tried to set entry '" + entry_key +
          "' (Type: " + getGlobalBlackboardPtr()->getEntry(entry_key)->info.typeName() + ") with value '" +
          p.value_to_string() + "' (Type: " + p.get_type_name() + ")");
      }
      continue;
    }

    // Check if parameter is known
    if (!auto_apms_util::contains(TREE_EXECUTOR_EXPLICITLY_ALLOWED_PARAMETERS, param_name)) {
      return create_rejected("Parameter is unkown");
    }

    // Check if the parameter is allowed to change during execution
    if (isBusy() && !auto_apms_util::contains(TREE_EXECUTOR_EXPLICITLY_ALLOWED_PARAMETERS_WHILE_BUSY, param_name)) {
      return create_rejected("Parameter is not allowed to change while tree executor is running");
    }

    // Check if build handler is allowed to change and valid
    if (param_name == _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER) {
      if (!params.allow_other_build_handlers) {
        return create_rejected(
          "This executor operates with tree build handler '" + executor_param_listener_.get_params().build_handler +
          "' and doesn't allow other build handlers to be loaded since the 'Allow other build handlers' option is "
          "disabled");
      }
      const std::string class_name = p.as_string();
      if (class_name != PARAM_VALUE_NO_BUILD_HANDLER && !build_handler_loader_ptr_->isClassAvailable(class_name)) {
        return create_rejected(
          "Cannot load build handler '" + class_name +
          "' because no corresponding ament_index resource was found. Make sure that you spelled the build handler's "
          "name correctly "
          "and registered it by calling auto_apms_behavior_tree_register_build_handlers() in the CMakeLists.txt of "
          "the "
          "corresponding package");
      }
    }

    // At this point, if the parameter hasn't been declared, we do not support it.
    if (!node_ptr_->has_parameter(param_name)) {
      return create_rejected("Parameter '" + param_name + "' is not supported");
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void GenericEventBasedTreeExecutor::parameter_event_callback_(const rcl_interfaces::msg::ParameterEvent & event)
{
  std::regex re(node_ptr_->get_fully_qualified_name());
  if (std::regex_match(event.node, re)) {
    for (const rclcpp::Parameter & p : rclcpp::ParameterEventHandler::get_parameters_from_event(event)) {
      const std::string param_name = p.get_name();

      if (const std::string enum_key = stripPrefixFromParameterName(SCRIPTING_ENUM_PARAM_PREFIX, param_name);
          !enum_key.empty()) {
        updateScriptingEnumsWithParameterValues({{enum_key, p.get_parameter_value()}});
      }

      if (const std::string entry_key = stripPrefixFromParameterName(BLACKBOARD_PARAM_PREFIX, param_name);
          !entry_key.empty()) {
        updateGlobalBlackboardWithParameterValues({{entry_key, p.get_parameter_value()}}, false);
      }

      if (param_name == _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER) {
        loadBuildHandler(p.as_string());
      }
    }
  }
}

rclcpp_action::GoalResponse GenericEventBasedTreeExecutor::handle_command_goal_(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const CommandActionContext::Goal> goal_ptr)
{
  if (command_timer_ptr_ && !command_timer_ptr_->is_canceled()) {
    RCLCPP_WARN(logger_, "Request for setting tree executor command rejected, because previous one is still busy.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto execution_state = getExecutionState();
  switch (goal_ptr->command) {
    case CommandActionContext::Goal::COMMAND_RESUME:
      if (execution_state == ExecutionState::PAUSED || execution_state == ExecutionState::HALTED) {
        RCLCPP_INFO(logger_, "Tree with ID '%s' will RESUME.", getTreeName().c_str());
      } else {
        RCLCPP_WARN(
          logger_, "Requested to RESUME with executor being in state %s. Rejecting request.",
          toStr(execution_state).c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      break;
    case CommandActionContext::Goal::COMMAND_PAUSE:
      if (execution_state == ExecutionState::STARTING || execution_state == ExecutionState::RUNNING) {
        RCLCPP_INFO(logger_, "Tree with ID '%s' will PAUSE", getTreeName().c_str());
      } else {
        RCLCPP_INFO(
          logger_, "Requested to PAUSE with executor already being inactive (State: %s).",
          toStr(execution_state).c_str());
      }
      break;
    case CommandActionContext::Goal::COMMAND_HALT:
      if (
        execution_state == ExecutionState::STARTING || execution_state == ExecutionState::RUNNING ||
        execution_state == ExecutionState::PAUSED) {
        RCLCPP_INFO(logger_, "Tree with ID '%s' will HALT.", getTreeName().c_str());
      } else {
        RCLCPP_INFO(
          logger_, "Requested to HALT with executor already being inactive (State: %s).",
          toStr(execution_state).c_str());
      }
      break;
    case CommandActionContext::Goal::COMMAND_TERMINATE:
      if (isBusy()) {
        RCLCPP_INFO(logger_, "Executor will TERMINATE tree '%s'.", getTreeName().c_str());
      } else {
        RCLCPP_INFO(
          logger_, "Requested to TERMINATE with executor already being inactive (State: %s).",
          toStr(execution_state).c_str());
      }
      break;
    default:
      RCLCPP_WARN(logger_, "Executor command %i is undefined. Rejecting request.", goal_ptr->command);
      return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GenericEventBasedTreeExecutor::handle_command_cancel_(
  std::shared_ptr<CommandActionContext::GoalHandle> /*goal_handle_ptr*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GenericEventBasedTreeExecutor::handle_command_accept_(
  std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr)
{
  const auto command_request = goal_handle_ptr->get_goal()->command;
  ExecutionState requested_state;
  switch (command_request) {
    case CommandActionContext::Goal::COMMAND_RESUME:
      setControlCommand(ControlCommand::RUN);
      requested_state = ExecutionState::RUNNING;
      break;
    case CommandActionContext::Goal::COMMAND_PAUSE:
      setControlCommand(ControlCommand::PAUSE);
      requested_state = ExecutionState::PAUSED;
      break;
    case CommandActionContext::Goal::COMMAND_HALT:
      setControlCommand(ControlCommand::HALT);
      requested_state = ExecutionState::HALTED;
      break;
    case CommandActionContext::Goal::COMMAND_TERMINATE:
      setControlCommand(ControlCommand::TERMINATE);
      requested_state = ExecutionState::IDLE;
      break;
    default:
      throw std::logic_error("command_request is unkown");
  }

  command_timer_ptr_ = node_ptr_->create_wall_timer(
    std::chrono::duration<double>(executor_param_listener_.get_params().tick_rate),
    [this, requested_state, goal_handle_ptr, action_result_ptr = std::make_shared<CommandActionContext::Result>()]() {
      if (goal_handle_ptr->is_canceling()) {
        goal_handle_ptr->canceled(action_result_ptr);
        command_timer_ptr_->cancel();
        return;
      }

      const auto current_state = getExecutionState();

      if (requested_state != ExecutionState::IDLE && current_state == ExecutionState::IDLE) {
        RCLCPP_ERROR(
          logger_, "Failed to reach requested state %s due to cancellation of execution timer. Aborting.",
          toStr(requested_state).c_str());
        goal_handle_ptr->abort(action_result_ptr);
        command_timer_ptr_->cancel();
        return;
      }

      if (current_state != requested_state) return;

      goal_handle_ptr->succeed(action_result_ptr);
      command_timer_ptr_->cancel();
    });
}

bool GenericEventBasedTreeExecutor::onTick()
{
  const ExecutorParameters params = executor_param_listener_.get_params();
  getStateObserver().setLogging(params.state_change_logger);
  return true;
}

bool GenericEventBasedTreeExecutor::afterTick()
{
  const ExecutorParameters params = executor_param_listener_.get_params();

  // Synchronize parameters with new blackboard entries if enabled
  if (executor_options_.blackboard_parameters_dynamic_ && params.allow_dynamic_blackboard) {
    TreeBlackboardSharedPtr bb_ptr = getGlobalBlackboardPtr();
    std::vector<rclcpp::Parameter> new_parameters;
    for (const BT::StringView & str : bb_ptr->getKeys()) {
      const std::string key = std::string(str);
      const BT::TypeInfo * type_info = bb_ptr->entryInfo(key);
      const BT::Any * any = bb_ptr->getAnyLocked(key).get();

      if (any->empty()) continue;

      if (translated_global_blackboard_entries_.find(key) == translated_global_blackboard_entries_.end()) {
        const BT::Expected<rclcpp::ParameterValue> expected =
          createParameterValueFromAny(*any, rclcpp::PARAMETER_NOT_SET);
        if (expected) {
          new_parameters.push_back(rclcpp::Parameter(BLACKBOARD_PARAM_PREFIX + "." + key, expected.value()));
          translated_global_blackboard_entries_[key] = expected.value();
        } else {
          RCLCPP_WARN(
            logger_, "Failed to translate new blackboard entry '%s' (Type: %s) to parameters: %s", key.c_str(),
            type_info->typeName().c_str(), expected.error().c_str());
        }
      } else {
        const BT::Expected<rclcpp::ParameterValue> expected =
          createParameterValueFromAny(*any, translated_global_blackboard_entries_[key].get_type());
        if (expected) {
          if (expected.value() != translated_global_blackboard_entries_[key]) {
            new_parameters.push_back(rclcpp::Parameter(BLACKBOARD_PARAM_PREFIX + "." + key, expected.value()));
          }
        } else {
          RCLCPP_WARN(
            logger_, "Failed to translate blackboard entry '%s' (Type: %s) to parameters: %s", key.c_str(),
            type_info->typeName().c_str(), expected.error().c_str());
        }
      }
    }
    if (!new_parameters.empty()) {
      const rcl_interfaces::msg::SetParametersResult result = node_ptr_->set_parameters_atomically(new_parameters);
      if (!result.successful) {
        throw exceptions::TreeExecutorError(
          "Unexpectedly failed to set parameters inferred from global blackboard. Reason: " + result.reason);
      }
    }
  }

  return true;
}

}  // namespace auto_apms_behavior_tree
