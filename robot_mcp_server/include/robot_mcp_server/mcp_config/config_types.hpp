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

#ifndef ROBOT_MCP_CONFIG_TYPES_HPP_
#define ROBOT_MCP_CONFIG_TYPES_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace robot_mcp::config
{

/**
 * @brief Conflict resolution strategy for resource groups
 */
enum class ConflictResolution
{
  ERROR,  ///< Return error if conflict occurs
  CANCEL,  ///< Cancel existing operation and start new one
  QUEUE  ///< Queue new operation until current one completes
};

/**
 * @brief Configuration for HTTP server
 */
struct ServerConfig
{
  std::string host{"0.0.0.0"};  ///< Server host address
  int port{8080};  ///< Server port number
  std::optional<std::string> api_key;  ///< Optional API key for authentication
  int thread_pool_size{10};  ///< HTTP server thread pool size
  int max_connections{100};  ///< Maximum concurrent connections
  int timeout_ms{30000};  ///< Request timeout in milliseconds
  bool enable_cors{true};  ///< Enable CORS headers
  bool enable_https{false};  ///< Enable HTTPS/TLS (requires OpenSSL)
  std::string ssl_cert_path;  ///< Path to SSL certificate file (PEM format)
  std::string ssl_key_path;  ///< Path to SSL private key file (PEM format)

  // Bond configuration for lifecycle manager compatibility
  bool bond_enabled{true};  ///< Enable bond heartbeat with lifecycle manager
  double bond_timeout{4.0};  ///< Bond heartbeat timeout in seconds
  double bond_heartbeat_period{0.1};  ///< Bond heartbeat period in seconds
};

/**
 * @brief Configuration for a topic (publish/subscribe)
 */
struct TopicConfig
{
  std::string name;  ///< Friendly name for MCP
  std::string topic;  ///< ROS2 topic name
  std::string msg_type;  ///< ROS2 message type (e.g., "std_msgs/msg/String")
  std::string plugin;  ///< Plugin class name
  bool subscribe{true};  ///< Subscribe and cache messages
  bool publish{true};  ///< Allow publishing to this topic
  std::vector<std::string> resource_groups;  ///< Resource groups this topic belongs to
};

/**
 * @brief Configuration for a service (call/response)
 */
struct ServiceConfig
{
  std::string name;  ///< Friendly name for MCP
  std::string service;  ///< ROS2 service name
  std::string srv_type;  ///< ROS2 service type
  std::string plugin;  ///< Plugin class name
  int timeout_ms{5000};  ///< Service call timeout
  std::vector<std::string> resource_groups;  ///< Resource groups this service belongs to
};

/**
 * @brief Configuration for an action (goal/feedback/result)
 */
struct ActionConfig
{
  std::string name;  ///< Friendly name for MCP
  std::string action;  ///< ROS2 action name
  std::string action_type;  ///< ROS2 action type
  std::string plugin;  ///< Plugin class name
  int timeout_ms{0};  ///< Action timeout (0 = no timeout)
  bool cancellable{true};  ///< Whether action can be cancelled
  std::vector<std::string> resource_groups;  ///< Resource groups this action belongs to
};

/**
 * @brief Configuration for a resource group
 *
 * Resource groups prevent conflicting operations from running concurrently.
 * For example, multiple navigation commands shouldn't run simultaneously.
 */
struct ResourceGroupConfig
{
  std::string name;  ///< Resource group name
  int max_concurrent{1};  ///< Maximum concurrent operations
  bool interruptible{true};  ///< Whether operations can be cancelled
  ConflictResolution conflict_resolution{ConflictResolution::ERROR};  ///< Default conflict handling
};

/**
 * @brief Complete MCP server configuration
 */
struct MCPServerConfig
{
  ServerConfig server;  ///< HTTP server configuration
  std::vector<TopicConfig> topics;  ///< Topics to expose
  std::vector<ServiceConfig> services;  ///< Services to expose
  std::vector<ActionConfig> actions;  ///< Actions to expose
  std::map<std::string, ResourceGroupConfig> resource_groups;  ///< Resource group definitions
};

}  // namespace robot_mcp::config

#endif  // ROBOT_MCP_CONFIG_TYPES_HPP_
