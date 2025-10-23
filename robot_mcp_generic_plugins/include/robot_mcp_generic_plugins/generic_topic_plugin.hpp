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

#ifndef ROBOT_MCP_GENERIC_PLUGINS__GENERIC_TOPIC_PLUGIN_HPP_
#define ROBOT_MCP_GENERIC_PLUGINS__GENERIC_TOPIC_PLUGIN_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>

#include "robot_mcp_msg_pluginlib/message_plugin.hpp"
#include "robot_mcp_generic_plugins/message_conversion.hpp"

namespace robot_mcp_generic_plugins
{

/**
 * @brief Generic plugin for ALL ROS2 topic types
 *
 * This plugin uses rclcpp::GenericSubscription and runtime type introspection
 * to support any ROS2 message type without compile-time knowledge.
 *
 * Features:
 * - Subscribes to topics and caches latest N messages
 * - Publishes messages to topics
 * - Automatic JSON ↔ ROS conversion via introspection
 * - Exposed as MCP resources (for subscribe=true) and/or tools (for publish=true)
 */
class GenericTopicPlugin : public robot_mcp_msg_pluginlib::MessagePlugin
{
public:
  GenericTopicPlugin();
  virtual ~GenericTopicPlugin();

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
  // Topic-specific methods (called by MCPRouter)
  // ============================================================================

  /**
   * @brief Initialize topic-specific configuration
   *
   * This is called after initialize() to provide topic-specific config.
   *
   * @param topic_name ROS2 topic name (e.g., "/battery_state")
   * @param msg_type ROS2 message type (e.g., "sensor_msgs/msg/BatteryState")
   * @param subscribe Whether to subscribe and cache messages
   * @param publish Whether to allow publishing
   */
  void configureTopic(
    const std::string & topic_name,
    const std::string & msg_type,
    bool subscribe,
    bool publish);

  /**
   * @brief Publish a message to the topic
   *
   * Called by MCPRouter when MCP client uses tools/call to publish.
   *
   * @param msg JSON message to publish
   * @throws std::runtime_error if publish is not enabled
   */
  void publishMessage(const nlohmann::json & msg);

  /**
   * @brief Read the latest cached message
   *
   * Called by MCPRouter when MCP client uses resources/read.
   *
   * @return JSON representation of latest message
   * @throws std::runtime_error if subscribe is not enabled or no messages cached
   */
  nlohmann::json readCachedMessage() const;

  /**
   * @brief Check if plugin has cached messages available
   *
   * @return true if at least one message is cached
   */
  bool hasCachedMessages() const { return !message_cache_.empty(); }

protected:
  // Node handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Primitive identification
  std::string primitive_name_;
  std::string description_;

  // Topic configuration
  std::string topic_name_;
  std::string msg_type_;
  bool subscribe_enabled_{false};
  bool publish_enabled_{false};

  // Generic ROS2 clients (runtime type handling)
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  std::shared_ptr<rclcpp::GenericPublisher> publisher_;

  // Message cache (JSON format for fast access)
  static constexpr size_t MAX_CACHE_SIZE = 10;
  mutable std::deque<nlohmann::json> message_cache_;

  // Type support for introspection
  const rosidl_message_type_support_t * type_support_{nullptr};

  /**
   * @brief Callback for generic subscription
   *
   * Converts ROS message to JSON and caches it.
   */
  void subscriptionCallback(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg);
};

}  // namespace robot_mcp_generic_plugins

#endif  // ROBOT_MCP_GENERIC_PLUGINS__GENERIC_TOPIC_PLUGIN_HPP_
