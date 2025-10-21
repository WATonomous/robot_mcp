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

#include "robot_mcp_server/robot_mcp_server_node.hpp"

#include <memory>
#include <string>

#include "robot_mcp_server/mcp_config/config_parser.hpp"

namespace robot_mcp
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

MCPServerNode::MCPServerNode(const rclcpp::NodeOptions & options)
: LifecycleNode("mcp_http_server", options)
{
  RCLCPP_INFO(get_logger(), "MCP HTTP Server node created");
}

MCPServerNode::MCPServerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options)
{
  RCLCPP_INFO(get_logger(), "MCP HTTP Server node created");
}

MCPServerNode::~MCPServerNode()
{
  RCLCPP_INFO(get_logger(), "MCP HTTP Server node destroyed");
}

CallbackReturn MCPServerNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring MCP server...");

  try {
    // Parse configuration from ROS2 parameters
    config_ = config::ConfigParser::parse(shared_from_this());

    RCLCPP_INFO(get_logger(), "Configuration loaded successfully:");
    RCLCPP_INFO(get_logger(), "  Server: %s:%d", config_.server.host.c_str(), config_.server.port);
    RCLCPP_INFO(
      get_logger(),
      "  Topics: %zu, Services: %zu, Actions: %zu",
      config_.topics.size(),
      config_.services.size(),
      config_.actions.size());

    if (config_.server.api_key.has_value()) {
      RCLCPP_INFO(get_logger(), "  Authentication: ENABLED");
    } else {
      RCLCPP_WARN(get_logger(), "  Authentication: DISABLED (not recommended for production)");
    }

    return CallbackReturn::SUCCESS;
  } catch (const config::ConfigParseException & e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse configuration: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Unexpected error during configuration: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn MCPServerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating MCP server...");

  try {
    // TODO(eddy): Initialize plugin system
    // TODO(eddy): Start HTTP server
    // TODO(eddy): Initialize router

    RCLCPP_INFO(
      get_logger(), "MCP server activated successfully on %s:%d", config_.server.host.c_str(), config_.server.port);

    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate MCP server: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn MCPServerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating MCP server...");

  try {
    // TODO(eddy): Stop HTTP server
    // TODO(eddy): Cancel all active operations
    // TODO(eddy): Keep plugins loaded

    RCLCPP_INFO(get_logger(), "MCP server deactivated successfully");

    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error during deactivation: %s", e.what());
    // Continue with deactivation even on error
    return CallbackReturn::SUCCESS;
  }
}

CallbackReturn MCPServerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up MCP server...");

  try {
    // TODO(eddy): Unload all plugins
    // TODO(eddy): Clear router
    // TODO(eddy): Release HTTP server resources

    // Clear configuration
    config_ = config::MCPServerConfig{};

    RCLCPP_INFO(get_logger(), "MCP server cleanup completed");

    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error during cleanup: %s", e.what());
    // Continue with cleanup even on error
    return CallbackReturn::SUCCESS;
  }
}

CallbackReturn MCPServerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down MCP server...");

  try {
    // Perform emergency stop
    // TODO(eddy): Stop HTTP server immediately
    // TODO(eddy): Cancel all operations
    // TODO(eddy): Unload plugins

    RCLCPP_INFO(get_logger(), "MCP server shutdown completed");

    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error during shutdown: %s", e.what());
    // Always return success for shutdown
    return CallbackReturn::SUCCESS;
  }
}

}  // namespace robot_mcp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mcp::MCPServerNode)
