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

#ifndef ROBOT_MCP_MSG_PLUGINLIB__MESSAGE_PLUGIN_HPP_
#define ROBOT_MCP_MSG_PLUGINLIB__MESSAGE_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>

namespace robot_mcp_msg_pluginlib
{

// Forward declarations for config types
namespace config
{
struct TopicConfig;
struct ServiceConfig;
struct ActionConfig;
}  // namespace config

/**
 * @brief Tag dispatch types for service/action message parts
 *
 * These empty structs are used for compile-time function overload resolution
 * while maintaining runtime polymorphism through virtual functions.
 */
namespace message_part
{
struct Request {};   // For service requests and action goals
struct Response {};  // For service responses and action results
struct Feedback {};  // For action feedback only
}  // namespace message_part

/**
 * @brief Base interface for all MCP message plugins
 *
 * This is the minimal interface that all plugins must implement for pluginlib discovery.
 * Derived plugins (BasicTopicPlugin, BasicServicePlugin, BasicActionPlugin) extend this
 * with specific methods for their operation type.
 *
 * Plugins are discovered automatically at runtime using pluginlib.
 */
class MessagePlugin
{
public:
  virtual ~MessagePlugin() = default;

  /**
   * @brief Initialize plugin with lifecycle node and primitive configuration
   *
   * This is called once during startup after the plugin is loaded.
   * Plugins should create ROS2 clients (subscribers, service clients, etc.) here.
   *
   * @param node Shared pointer to the lifecycle node
   * @param primitive_name Name of the primitive (e.g., "battery", "navigate")
   * @param description Human-readable description for MCP tools/resources list
   */
  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & primitive_name,
    const std::string & description) = 0;

  /**
   * @brief Get list of supported message types
   *
   * @return Empty vector = supports ALL types (generic plugin)
   *         Non-empty vector = only supports specific types (custom plugin)
   *
   * Used by MCPPrimitiveFactory to validate plugin compatibility at startup.
   */
  virtual std::vector<std::string> getSupportedTypes() const = 0;

  /**
   * @brief Get human-readable description for MCP protocol
   *
   * This description is shown to Claude in tools/list and resources/list responses,
   * allowing Claude to reason about what each primitive does.
   *
   * @return Description string (from config)
   */
  virtual std::string getDescription() const = 0;

  /**
   * @brief Get the primitive name
   *
   * @return Primitive name (e.g., "battery", "cmd_vel", "navigate")
   */
  virtual std::string getPrimitiveName() const = 0;
};

}  // namespace robot_mcp_msg_pluginlib

#endif  // ROBOT_MCP_MSG_PLUGINLIB__MESSAGE_PLUGIN_HPP_
