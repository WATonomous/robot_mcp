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

#ifndef ROBOT_MCP_CONFIG_PARSER_HPP_
#define ROBOT_MCP_CONFIG_PARSER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_mcp_server/mcp_config/config_types.hpp"

namespace robot_mcp::config
{

/**
 * @brief Exception thrown when configuration parsing fails
 */
class ConfigParseException : public std::runtime_error
{
public:
  explicit ConfigParseException(const std::string & message)
  : std::runtime_error(message) {}
};

/**
 * @brief Parser for MCP server configuration from ROS2 parameters
 *
 * This class reads ROS2 parameters and constructs a MCPServerConfig object.
 * It handles parameter validation and provides detailed error messages.
 */
class ConfigParser
{
public:
  /**
   * @brief Parse configuration from a ROS2 node's parameters
   *
   * @param node ROS2 lifecycle node to read parameters from
   * @return Parsed configuration
   * @throws ConfigParseException if parsing fails
   */
  static MCPServerConfig parse(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Validate a configuration object
   *
   * Checks for:
   * - Duplicate names
   * - Invalid port numbers
   * - Missing required fields
   * - Invalid resource group references
   *
   * @param config Configuration to validate
   * @throws ConfigParseException if validation fails
   */
  static void validate(const MCPServerConfig & config);

private:
  /**
   * @brief Parse server configuration
   */
  static ServerConfig parseServerConfig(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Parse topic configurations
   */
  static std::vector<TopicConfig> parseTopics(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Parse service configurations
   */
  static std::vector<ServiceConfig> parseServices(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Parse action configurations
   */
  static std::vector<ActionConfig> parseActions(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Parse resource group configurations
   */
  static std::map<std::string, ResourceGroupConfig> parseResourceGroups(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Parse conflict resolution string to enum
   */
  static ConflictResolution parseConflictResolution(const std::string & str);
};

}  // namespace robot_mcp::config

#endif  // ROBOT_MCP_CONFIG_PARSER_HPP_
