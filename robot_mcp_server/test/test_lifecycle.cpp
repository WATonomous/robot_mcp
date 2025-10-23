// Copyright (c) 2025-present WATonomous. All rights reserved.
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

// Copyright 2025 WATonomous
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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_mcp_test/robot_mcp_test.hpp>

#include "robot_mcp_server/robot_mcp_server_node.hpp"

namespace robot_mcp::test
{

TEST_CASE_METHOD(robot_mcp::test::TestExecutorFixture, "MCPServerNode - Lifecycle Smoke Test", "[lifecycle]")
{
  auto node = std::make_shared<robot_mcp::MCPServerNode>();
  add_node(node);
  start_spinning();

  SECTION("Initial state is UNCONFIGURED")
  {
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    REQUIRE(node->get_current_state().label() == "unconfigured");
  }

  SECTION("Full lifecycle: UNCONFIGURED -> INACTIVE -> ACTIVE -> INACTIVE -> UNCONFIGURED")
  {
    // UNCONFIGURED -> INACTIVE (configure)
    auto result = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    REQUIRE(result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    REQUIRE(node->get_current_state().label() == "inactive");

    // INACTIVE -> ACTIVE (activate)
    result = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    REQUIRE(result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    REQUIRE(node->get_current_state().label() == "active");

    // Let it run for a moment
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ACTIVE -> INACTIVE (deactivate)
    result = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    REQUIRE(result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    // INACTIVE -> UNCONFIGURED (cleanup)
    result = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    REQUIRE(result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("Can shutdown from any state")
  {
    // From UNCONFIGURED
    auto result = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    REQUIRE(result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  }
}

}  // namespace robot_mcp::test
