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

#ifndef ROBOT_MCP__MCP_SERVER_NODE_HPP_
#define ROBOT_MCP__MCP_SERVER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_mcp_server/mcp_config/config_types.hpp"

namespace robot_mcp
{

/**
 * @brief Main MCP server node
 *
 * This node consolidates a Model Context Protocol (MCP) server over HTTP
 * that allows AI assistants to interact with ROS2 robots.
 */
class MCPServerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for components
   * @param options Node options for configuration
   */
  explicit MCPServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Constructor with custom node name
   * @param node_name Node name
   * @param options Node options for configuration
   */
  explicit MCPServerNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~MCPServerNode() override;

  // Delete copy/move constructors and assignment operators
  // There should never be a direct copy of an instance of MCPServerNode
  MCPServerNode(const MCPServerNode &) = delete;
  MCPServerNode & operator=(const MCPServerNode &) = delete;
  MCPServerNode(MCPServerNode &&) = delete;
  MCPServerNode & operator=(MCPServerNode &&) = delete;

protected:
  /**
   * @brief On Configure
   *
   * Loads and validates configuration from parameters:
   * - Server settings (host, port, API key)
   * - Resource groups and conflict resolution policies
   * - Topics, services, actions to expose
   *
   * @param state Current lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief On Activate
   *
   * Instantiates all configured plugins (topics, services, actions).
   * Starts the HTTP server and begins accepting MCP requests.
   *
   * @param state Current lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief On Deactivate
   *
   * Stops the HTTP server and cancels all active operations.
   * Keeps configuration loaded for potential reactivation.
   *
   * @param state Current lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief On Cleanup
   *
   * Releases all resources, unloads plugins, clears configuration.
   * Returns to UNCONFIGURED state.
   *
   * @param state Current lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief On Shutdown
   *
   * Emergency shutdown from any state. Stops all operations immediately.
   *
   * @param state Current lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

private:
  config::MCPServerConfig config_;  ///< Parsed configuration

  // TODO(eddy): Add plugin system components
  // TODO(eddy): Add HTTP server
  // TODO(eddy): Add router
};

}  // namespace robot_mcp

#endif  // ROBOT_MCP__MCP_SERVER_NODE_HPP_
