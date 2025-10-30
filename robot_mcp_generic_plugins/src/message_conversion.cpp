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

#include "robot_mcp_generic_plugins/message_conversion.hpp"

#include <stdexcept>
#include <cstring>
#include <sstream>
#include <dlfcn.h>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

namespace robot_mcp_generic_plugins
{
namespace conversion
{

namespace
{

// Forward declarations for recursive conversion
nlohmann::json fieldToJson(
  const void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member,
  const rosidl_message_type_support_t * type_support);

void jsonToField(
  const nlohmann::json & json,
  void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member,
  const rosidl_message_type_support_t * type_support);

// Convert primitive field to JSON
nlohmann::json primitiveToJson(
  const void * field_ptr,
  uint8_t type_id)
{
  using namespace rosidl_typesupport_introspection_cpp;

  switch (type_id) {
    case ROS_TYPE_BOOL:
      return *static_cast<const bool *>(field_ptr);
    case ROS_TYPE_BYTE:
    case ROS_TYPE_UINT8:
      return *static_cast<const uint8_t *>(field_ptr);
    case ROS_TYPE_CHAR:
    case ROS_TYPE_INT8:
      return *static_cast<const int8_t *>(field_ptr);
    case ROS_TYPE_UINT16:
      return *static_cast<const uint16_t *>(field_ptr);
    case ROS_TYPE_INT16:
      return *static_cast<const int16_t *>(field_ptr);
    case ROS_TYPE_UINT32:
      return *static_cast<const uint32_t *>(field_ptr);
    case ROS_TYPE_INT32:
      return *static_cast<const int32_t *>(field_ptr);
    case ROS_TYPE_UINT64:
      return *static_cast<const uint64_t *>(field_ptr);
    case ROS_TYPE_INT64:
      return *static_cast<const int64_t *>(field_ptr);
    case ROS_TYPE_FLOAT:
      return *static_cast<const float *>(field_ptr);
    case ROS_TYPE_DOUBLE:
      return *static_cast<const double *>(field_ptr);
    case ROS_TYPE_STRING:
      return *static_cast<const std::string *>(field_ptr);
    default:
      throw std::runtime_error("Unknown primitive type: " + std::to_string(type_id));
  }
}

// Convert JSON to primitive field
void jsonToPrimitive(
  const nlohmann::json & json,
  void * field_ptr,
  uint8_t type_id)
{
  using namespace rosidl_typesupport_introspection_cpp;

  switch (type_id) {
    case ROS_TYPE_BOOL:
      *static_cast<bool *>(field_ptr) = json.get<bool>();
      break;
    case ROS_TYPE_BYTE:
    case ROS_TYPE_UINT8:
      *static_cast<uint8_t *>(field_ptr) = json.get<uint8_t>();
      break;
    case ROS_TYPE_CHAR:
    case ROS_TYPE_INT8:
      *static_cast<int8_t *>(field_ptr) = json.get<int8_t>();
      break;
    case ROS_TYPE_UINT16:
      *static_cast<uint16_t *>(field_ptr) = json.get<uint16_t>();
      break;
    case ROS_TYPE_INT16:
      *static_cast<int16_t *>(field_ptr) = json.get<int16_t>();
      break;
    case ROS_TYPE_UINT32:
      *static_cast<uint32_t *>(field_ptr) = json.get<uint32_t>();
      break;
    case ROS_TYPE_INT32:
      *static_cast<int32_t *>(field_ptr) = json.get<int32_t>();
      break;
    case ROS_TYPE_UINT64:
      *static_cast<uint64_t *>(field_ptr) = json.get<uint64_t>();
      break;
    case ROS_TYPE_INT64:
      *static_cast<int64_t *>(field_ptr) = json.get<int64_t>();
      break;
    case ROS_TYPE_FLOAT:
      *static_cast<float *>(field_ptr) = json.get<float>();
      break;
    case ROS_TYPE_DOUBLE:
      *static_cast<double *>(field_ptr) = json.get<double>();
      break;
    case ROS_TYPE_STRING:
      *static_cast<std::string *>(field_ptr) = json.get<std::string>();
      break;
    default:
      throw std::runtime_error("Unknown primitive type: " + std::to_string(type_id));
  }
}

// Convert array/sequence field to JSON
nlohmann::json arrayToJson(
  const void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member,
  const rosidl_message_type_support_t * type_support)
{
  using namespace rosidl_typesupport_introspection_cpp;

  nlohmann::json array = nlohmann::json::array();

  // Get array/sequence size using the proper introspection function
  size_t array_size = 0;
  if (member.is_array_) {
    if (member.is_upper_bound_) {
      // Dynamic-size bounded sequence - use size function
      array_size = member.size_function(field_ptr);
    } else {
      // Fixed-size array
      array_size = member.array_size_;
    }
  } else {
    // Unbounded sequence - use size function
    array_size = member.size_function(field_ptr);
  }

  // Process each element using get_const_function for proper type-safe access
  for (size_t i = 0; i < array_size; ++i) {
    const void * element_ptr = member.get_const_function(field_ptr, i);

    if (member.type_id_ == ROS_TYPE_MESSAGE) {
      // Nested message array
      const auto * nested_members = static_cast<const MessageMembers *>(member.members_->data);
      nlohmann::json element_json;
      for (size_t j = 0; j < nested_members->member_count_; ++j) {
        const auto & nested_member = nested_members->members_[j];
        const void * nested_field_ptr = static_cast<const uint8_t *>(element_ptr) + nested_member.offset_;
        element_json[nested_member.name_] = fieldToJson(nested_field_ptr, nested_member, type_support);
      }
      array.push_back(element_json);
    } else {
      // Primitive array - element_ptr points directly to the element
      array.push_back(primitiveToJson(element_ptr, member.type_id_));
    }
  }

  return array;
}

// Convert JSON to array/sequence field
void jsonToArray(
  const nlohmann::json & json,
  void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member,
  const rosidl_message_type_support_t * type_support)
{
  using namespace rosidl_typesupport_introspection_cpp;

  if (!json.is_array()) {
    throw std::runtime_error("Expected JSON array for field: " + std::string(member.name_));
  }

  size_t json_size = json.size();

  if (member.is_array_) {
    if (!member.is_upper_bound_) {
      // Fixed-size array - size must match exactly
      if (json_size != member.array_size_) {
        throw std::runtime_error(
          "Array size mismatch for field " + std::string(member.name_) +
          ": expected " + std::to_string(member.array_size_) +
          ", got " + std::to_string(json_size));
      }
    } else {
      // Bounded sequence - resize if needed
      if (json_size > member.array_size_) {
        throw std::runtime_error(
          "Array size exceeds upper bound for field " + std::string(member.name_) +
          ": max " + std::to_string(member.array_size_) +
          ", got " + std::to_string(json_size));
      }
      if (member.resize_function) {
        member.resize_function(field_ptr, json_size);
      }
    }
  } else {
    // Unbounded sequence - resize to JSON array size
    if (member.resize_function) {
      member.resize_function(field_ptr, json_size);
    }
  }

  // Populate elements using get_function for proper type-safe access
  for (size_t i = 0; i < json_size; ++i) {
    void * element_ptr = member.get_function(field_ptr, i);

    if (member.type_id_ == ROS_TYPE_MESSAGE) {
      // Nested message array/sequence
      const auto * nested_members = static_cast<const MessageMembers *>(member.members_->data);
      for (size_t j = 0; j < nested_members->member_count_; ++j) {
        const auto & nested_member = nested_members->members_[j];
        void * nested_field_ptr = static_cast<uint8_t *>(element_ptr) + nested_member.offset_;
        if (json[i].contains(nested_member.name_)) {
          jsonToField(json[i][nested_member.name_], nested_field_ptr, nested_member, type_support);
        }
      }
    } else {
      // Primitive array/sequence - element_ptr points directly to the element
      jsonToPrimitive(json[i], element_ptr, member.type_id_);
    }
  }
}

// Convert nested message field to JSON
nlohmann::json messageToJson(
  const void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  using namespace rosidl_typesupport_introspection_cpp;

  const auto * nested_members = static_cast<const MessageMembers *>(member.members_->data);
  nlohmann::json result;

  for (size_t i = 0; i < nested_members->member_count_; ++i) {
    const auto & nested_member = nested_members->members_[i];
    const void * nested_field_ptr = static_cast<const uint8_t *>(field_ptr) + nested_member.offset_;
    result[nested_member.name_] = fieldToJson(nested_field_ptr, nested_member, nullptr);
  }

  return result;
}

// Convert JSON to nested message field
void jsonToMessage(
  const nlohmann::json & json,
  void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  using namespace rosidl_typesupport_introspection_cpp;

  if (!json.is_object()) {
    throw std::runtime_error("Expected JSON object for field: " + std::string(member.name_));
  }

  const auto * nested_members = static_cast<const MessageMembers *>(member.members_->data);

  for (size_t i = 0; i < nested_members->member_count_; ++i) {
    const auto & nested_member = nested_members->members_[i];
    void * nested_field_ptr = static_cast<uint8_t *>(field_ptr) + nested_member.offset_;

    if (json.contains(nested_member.name_)) {
      jsonToField(json[nested_member.name_], nested_field_ptr, nested_member, nullptr);
    }
  }
}

// Convert any field to JSON (recursive)
nlohmann::json fieldToJson(
  const void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member,
  const rosidl_message_type_support_t * type_support)
{
  using namespace rosidl_typesupport_introspection_cpp;

  if (member.is_array_ || !member.is_upper_bound_) {
    // Array or sequence
    return arrayToJson(field_ptr, member, type_support);
  } else if (member.type_id_ == ROS_TYPE_MESSAGE) {
    // Nested message
    return messageToJson(field_ptr, member);
  } else {
    // Primitive
    return primitiveToJson(field_ptr, member.type_id_);
  }
}

// Convert JSON to any field (recursive)
void jsonToField(
  const nlohmann::json & json,
  void * field_ptr,
  const rosidl_typesupport_introspection_cpp::MessageMember & member,
  const rosidl_message_type_support_t * type_support)
{
  using namespace rosidl_typesupport_introspection_cpp;

  if (member.is_array_ || !member.is_upper_bound_) {
    // Array or sequence
    jsonToArray(json, field_ptr, member, type_support);
  } else if (member.type_id_ == ROS_TYPE_MESSAGE) {
    // Nested message
    jsonToMessage(json, field_ptr, member);
  } else {
    // Primitive
    jsonToPrimitive(json, field_ptr, member.type_id_);
  }
}

}  // anonymous namespace

nlohmann::json rosMessageToJson(
  const void * ros_msg,
  const rosidl_message_type_support_t * type_support)
{
  if (!ros_msg || !type_support) {
    throw std::runtime_error("Null pointer provided to rosMessageToJson");
  }

  // Ensure we have introspection type support
  if (type_support->typesupport_identifier !=
      rosidl_typesupport_introspection_cpp::typesupport_identifier) {
    throw std::runtime_error(
      "Type support must be from rosidl_typesupport_introspection_cpp");
  }

  const auto * members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    type_support->data);

  nlohmann::json result;

  // Walk all message fields
  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const void * field_ptr = static_cast<const uint8_t *>(ros_msg) + member.offset_;

    result[member.name_] = fieldToJson(field_ptr, member, type_support);
  }

  return result;
}

void jsonToRosMessage(
  const nlohmann::json & json,
  void * ros_msg,
  const rosidl_message_type_support_t * type_support)
{
  if (!ros_msg || !type_support) {
    throw std::runtime_error("Null pointer provided to jsonToRosMessage");
  }

  if (!json.is_object()) {
    throw std::runtime_error("JSON must be an object to convert to ROS message");
  }

  // Ensure we have introspection type support
  if (type_support->typesupport_identifier !=
      rosidl_typesupport_introspection_cpp::typesupport_identifier) {
    throw std::runtime_error(
      "Type support must be from rosidl_typesupport_introspection_cpp");
  }

  const auto * members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    type_support->data);

  // Walk all message fields
  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    void * field_ptr = static_cast<uint8_t *>(ros_msg) + member.offset_;

    if (json.contains(member.name_)) {
      jsonToField(json[member.name_], field_ptr, member, type_support);
    }
  }
}

const rosidl_message_type_support_t * getMessageTypeSupport(
  const std::string & msg_type)
{
  // Parse msg_type string: "package_name/msg/MessageName"
  std::istringstream iss(msg_type);
  std::string package, msg_namespace, message_name;

  std::getline(iss, package, '/');
  std::getline(iss, msg_namespace, '/');
  std::getline(iss, message_name);

  if (package.empty() || msg_namespace != "msg" || message_name.empty()) {
    throw std::runtime_error("Invalid message type format: " + msg_type + " (expected: package/msg/MessageName)");
  }

  // Build symbol name for type support lookup
  // Format: rosidl_typesupport_introspection_cpp__get_message_type_support_handle__package_name__msg__MessageName
  std::string symbol_name =
    "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
    package + "__msg__" + message_name;

  // Load type support using dlsym from RTLD_DEFAULT (searches all loaded libraries)
  void * symbol = dlsym(RTLD_DEFAULT, symbol_name.c_str());
  if (!symbol) {
    throw std::runtime_error(
      "Failed to find type support for " + msg_type +
      ": symbol " + symbol_name + " not found. " +
      "Error: " + (dlerror() ? dlerror() : "unknown"));
  }

  // Call the function to get type support
  using TypeSupportFunction = const rosidl_message_type_support_t * (*)();
  auto get_type_support = reinterpret_cast<TypeSupportFunction>(symbol);

  return get_type_support();
}

const rosidl_service_type_support_t * getServiceTypeSupport(
  const std::string & srv_type)
{
  // Parse srv_type string: "package_name/srv/ServiceName"
  std::istringstream iss(srv_type);
  std::string package, srv_namespace, service_name;

  std::getline(iss, package, '/');
  std::getline(iss, srv_namespace, '/');
  std::getline(iss, service_name);

  if (package.empty() || srv_namespace != "srv" || service_name.empty()) {
    throw std::runtime_error("Invalid service type format: " + srv_type + " (expected: package/srv/ServiceName)");
  }

  // Build symbol name for type support lookup
  std::string symbol_name =
    "rosidl_typesupport_introspection_cpp__get_service_type_support_handle__" +
    package + "__srv__" + service_name;

  // Load type support using dlsym from RTLD_DEFAULT
  void * symbol = dlsym(RTLD_DEFAULT, symbol_name.c_str());
  if (!symbol) {
    throw std::runtime_error(
      "Failed to find type support for " + srv_type +
      ": symbol " + symbol_name + " not found. " +
      "Error: " + (dlerror() ? dlerror() : "unknown"));
  }

  // Call the function to get type support
  using TypeSupportFunction = const rosidl_service_type_support_t * (*)();
  auto get_type_support = reinterpret_cast<TypeSupportFunction>(symbol);

  return get_type_support();
}

const rosidl_action_type_support_t * getActionTypeSupport(
  const std::string & action_type)
{
  // Parse action_type string: "package_name/action/ActionName"
  std::istringstream iss(action_type);
  std::string package, action_namespace, action_name;

  std::getline(iss, package, '/');
  std::getline(iss, action_namespace, '/');
  std::getline(iss, action_name);

  if (package.empty() || action_namespace != "action" || action_name.empty()) {
    throw std::runtime_error("Invalid action type format: " + action_type + " (expected: package/action/ActionName)");
  }

  // Build symbol name for type support lookup
  std::string symbol_name =
    "rosidl_typesupport_introspection_cpp__get_action_type_support_handle__" +
    package + "__action__" + action_name;

  // Load type support using dlsym from RTLD_DEFAULT
  void * symbol = dlsym(RTLD_DEFAULT, symbol_name.c_str());
  if (!symbol) {
    throw std::runtime_error(
      "Failed to find type support for " + action_type +
      ": symbol " + symbol_name + " not found. " +
      "Error: " + (dlerror() ? dlerror() : "unknown"));
  }

  // Call the function to get type support
  using TypeSupportFunction = const rosidl_action_type_support_t * (*)();
  auto get_type_support = reinterpret_cast<TypeSupportFunction>(symbol);

  return get_type_support();
}

}  // namespace conversion
}  // namespace robot_mcp_generic_plugins
