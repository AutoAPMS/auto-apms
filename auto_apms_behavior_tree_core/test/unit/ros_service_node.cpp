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

#include "auto_apms_behavior_tree_core/node/ros_service_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace std::chrono_literals;

using TestServiceT = example_interfaces::srv::AddTwoInts;

class TestRosServiceNode : public RosServiceNode<TestServiceT>
{
public:
  TestRosServiceNode(const std::string & instance_name, const Config & config, RosNodeContext context)
  : RosServiceNode<TestServiceT>(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  bool setRequest(typename Request::SharedPtr & request) override
  {
    request->a = 1;
    request->b = 2;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr &) override { return BT::NodeStatus::SUCCESS; }

  // Tick like a tree does: the returned status is stored and a parent resets it before the next tick
  BT::NodeStatus tickOnce()
  {
    resetStatus();
    return BT::TreeNode::executeTick();
  }
};

class RosServiceNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_service_node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // No server is offering this service
    options_.topic = "/unreachable_test_service";
    options_.wait_timeout = 100ms;
    config_.blackboard = BT::Blackboard::create();
  }

  void TearDown() override { rclcpp::shutdown(); }

  RosNodeContext createContext() { return RosNodeContext(node_, callback_group_, executor_, options_); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  NodeRegistrationOptions options_;
  BT::NodeConfig config_;
};

TEST_F(RosServiceNodeTest, UnreachableServiceInterruptsTheTreeByDefault)
{
  EXPECT_THROW(
    {
      TestRosServiceNode service_node("test_service", config_, createContext());
      service_node.tickOnce();
    },
    auto_apms_behavior_tree::exceptions::RosNodeError);
}

TEST_F(RosServiceNodeTest, UnreachableServiceFailsIfAllowed)
{
  options_.allow_unreachable = true;
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  EXPECT_NO_THROW({
    TestRosServiceNode service_node("test_service", config_, createContext());
    status = service_node.tickOnce();
  });
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(RosServiceNodeTest, FailsAgainOnTheNextTickIfStillUnreachable)
{
  options_.allow_unreachable = true;
  TestRosServiceNode service_node("test_service", config_, createContext());
  EXPECT_EQ(service_node.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(service_node.tickOnce(), BT::NodeStatus::FAILURE);
}
