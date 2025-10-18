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

#ifndef ROBOT_MCP_MSG_PLUGINLIB__MESSAGE_PLUGIN_HPP_
#define ROBOT_MCP_MSG_PLUGINLIB__MESSAGE_PLUGIN_HPP_

#include <string>
#include <nlohmann/json.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>

namespace robot_mcp_msg_pluginlib
{

/**
 * @brief Base class for all message plugins
 *
 * Each plugin handles a specific ROS2 message type and provides:
 * - JSON serialization (ROS2 message -> JSON)
 * - JSON deserialization (JSON -> ROS2 message)
 * - Message type information for dynamic type handling
 *
 * Plugins are discovered automatically at runtime using pluginlib.
 */
class MessagePlugin
{
public:
  virtual ~MessagePlugin() = default;

  /**
   * @brief Get the ROS2 message type this plugin handles
   * @return Message type string (e.g., "sensor_msgs/msg/BatteryState")
   */
  virtual std::string getMessageType() const = 0;

  /**
   * @brief Convert ROS2 message to JSON
   * @param msg_ptr Pointer to the ROS2 message (must be correct type)
   * @return JSON representation of the message
   */
  virtual nlohmann::json toJson(const void * msg_ptr) const = 0;

  /**
   * @brief Convert JSON to ROS2 message
   * @param j JSON object to deserialize
   * @param msg_ptr Pointer to the ROS2 message to populate (must be correct type)
   */
  virtual void fromJson(const nlohmann::json & j, void * msg_ptr) const = 0;

  /**
   * @brief Get the rosidl message type support structure
   * @return Pointer to the type support structure for dynamic message handling
   */
  virtual const rosidl_message_type_support_t * getTypeSupport() const = 0;
};

}  // namespace robot_mcp_msg_pluginlib

#endif  // ROBOT_MCP_MSG_PLUGINLIB__MESSAGE_PLUGIN_HPP_
