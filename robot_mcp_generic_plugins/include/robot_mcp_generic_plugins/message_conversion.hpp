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

#ifndef ROBOT_MCP_GENERIC_PLUGINS__MESSAGE_CONVERSION_HPP_
#define ROBOT_MCP_GENERIC_PLUGINS__MESSAGE_CONVERSION_HPP_

#include <string>

#include <nlohmann/json.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace robot_mcp_generic_plugins
{
namespace conversion
{

/**
 * @brief Convert ROS2 message to JSON using runtime introspection
 *
 * This function uses rosidl_typesupport_introspection_cpp to walk the message
 * structure at runtime and convert each field to JSON.
 *
 * @param ros_msg Pointer to ROS2 message
 * @param type_support Type support structure for introspection
 * @return JSON representation of the message
 *
 * TODO: Implement full introspection-based conversion
 */
nlohmann::json rosMessageToJson(
  const void * ros_msg,
  const rosidl_message_type_support_t * type_support);

/**
 * @brief Convert JSON to ROS2 message using runtime introspection
 *
 * This function uses rosidl_typesupport_introspection_cpp to walk the message
 * structure at runtime and populate each field from JSON.
 *
 * @param json JSON object to deserialize
 * @param ros_msg Pointer to ROS2 message to populate
 * @param type_support Type support structure for introspection
 *
 * TODO: Implement full introspection-based conversion
 */
void jsonToRosMessage(
  const nlohmann::json & json,
  void * ros_msg,
  const rosidl_message_type_support_t * type_support);

/**
 * @brief Get message type support from type string
 *
 * Resolves message type support at runtime from a string like
 * "sensor_msgs/msg/BatteryState".
 *
 * @param msg_type Message type string
 * @return Type support structure
 * @throws std::runtime_error if type not found
 *
 * TODO: Implement type support lookup
 */
const rosidl_message_type_support_t * getMessageTypeSupport(
  const std::string & msg_type);

/**
 * @brief Get service type support from type string
 *
 * Resolves service type support at runtime from a string like
 * "std_srvs/srv/Trigger".
 *
 * @param srv_type Service type string
 * @return Type support structure
 * @throws std::runtime_error if type not found
 *
 * TODO: Implement type support lookup
 */
const rosidl_service_type_support_t * getServiceTypeSupport(
  const std::string & srv_type);

/**
 * @brief Get action type support from type string
 *
 * Resolves action type support at runtime from a string like
 * "nav2_msgs/action/NavigateToPose".
 *
 * @param action_type Action type string
 * @return Type support structure
 * @throws std::runtime_error if type not found
 *
 * TODO: Implement type support lookup
 */
const rosidl_action_type_support_t * getActionTypeSupport(
  const std::string & action_type);

}  // namespace conversion
}  // namespace robot_mcp_generic_plugins

#endif  // ROBOT_MCP_GENERIC_PLUGINS__MESSAGE_CONVERSION_HPP_
