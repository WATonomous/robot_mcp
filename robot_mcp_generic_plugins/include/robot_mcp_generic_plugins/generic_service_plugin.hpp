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

#ifndef ROBOT_MCP_GENERIC_PLUGINS__GENERIC_SERVICE_PLUGIN_HPP_
#define ROBOT_MCP_GENERIC_PLUGINS__GENERIC_SERVICE_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/generic_client.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>

#include "robot_mcp_msg_pluginlib/message_plugin.hpp"
#include "robot_mcp_generic_plugins/message_conversion.hpp"

namespace robot_mcp_generic_plugins
{

/**
 * @brief Generic plugin for ALL ROS2 service types
 *
 * This plugin uses rclcpp::GenericClient and runtime type introspection
 * to support any ROS2 service type without compile-time knowledge.
 *
 * Features:
 * - Calls services with JSON request, returns JSON response
 * - Automatic JSON ↔ ROS conversion via introspection
 * - Tag dispatch for Request/Response parts
 * - Exposed as MCP tools via tools/call
 */
class GenericServicePlugin : public robot_mcp_msg_pluginlib::MessagePlugin
{
public:
  GenericServicePlugin();
  virtual ~GenericServicePlugin();

  // ============================================================================
  // MessagePlugin interface
  // ============================================================================

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & primitive_name,
    const std::string & description) override;

  std::vector<std::string> getSupportedTypes() const override
  {
    return {};  // Empty = supports ALL types
  }

  std::string getDescription() const override { return description_; }

  std::string getPrimitiveName() const override { return primitive_name_; }

  // ============================================================================
  // Service-specific methods (called by MCPRouter)
  // ============================================================================

  /**
   * @brief Initialize service-specific configuration
   *
   * This is called after initialize() to provide service-specific config.
   *
   * @param service_name ROS2 service name (e.g., "/reset")
   * @param srv_type ROS2 service type (e.g., "std_srvs/srv/Trigger")
   * @param timeout_ms Service call timeout in milliseconds
   */
  void configureService(
    const std::string & service_name,
    const std::string & srv_type,
    int timeout_ms);

  /**
   * @brief Call the service
   *
   * Called by MCPRouter when MCP client uses tools/call.
   * Blocks until response is received or timeout occurs.
   *
   * @param request JSON service request
   * @return JSON service response
   * @throws std::runtime_error on timeout or service failure
   */
  nlohmann::json callService(const nlohmann::json & request);

protected:
  // Node handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Primitive identification
  std::string primitive_name_;
  std::string description_;

  // Service configuration
  std::string service_name_;
  std::string srv_type_;
  int timeout_ms_{5000};

  // Generic ROS2 client (runtime type handling)
  std::shared_ptr<rclcpp::GenericClient> client_;

  // Type support for introspection
  const rosidl_service_type_support_t * type_support_{nullptr};
  rosidl_message_type_support_t request_type_support_{};
  rosidl_message_type_support_t response_type_support_{};
};

}  // namespace robot_mcp_generic_plugins

#endif  // ROBOT_MCP_GENERIC_PLUGINS__GENERIC_SERVICE_PLUGIN_HPP_
