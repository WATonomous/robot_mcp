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

#ifndef ROBOT_MCP_SERVER__ROS2__LIFECYCLE_NODE_HPP_
#define ROBOT_MCP_SERVER__ROS2__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace robot_mcp_server
{

/**
 * @brief Main lifecycle node for the robot_mcp HTTP server
 *
 * This node manages the lifecycle of the MCP server:
 * - Unconfigured -> Inactive: Load configuration, initialize plugins
 * - Inactive -> Active: Start HTTP server
 * - Active -> Inactive: Stop accepting new requests
 * - Inactive -> Unconfigured: Shutdown and cleanup
 */
class RobotMcpLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RobotMcpLifecycleNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobotMcpLifecycleNode() override;

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  // Private implementation (pimpl idiom)
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_mcp_server

#endif  // ROBOT_MCP_SERVER__ROS2__LIFECYCLE_NODE_HPP_
