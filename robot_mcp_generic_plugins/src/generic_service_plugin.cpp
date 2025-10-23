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

#include "robot_mcp_generic_plugins/generic_service_plugin.hpp"

#include <stdexcept>
#include <chrono>

#include <rclcpp/create_generic_client.hpp>
#include <rclcpp/serialization.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

namespace robot_mcp_generic_plugins
{

GenericServicePlugin::GenericServicePlugin()
{
}

GenericServicePlugin::~GenericServicePlugin()
{
}

void GenericServicePlugin::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & primitive_name,
  const std::string & description)
{
  node_ = node;
  primitive_name_ = primitive_name;
  description_ = description;
}

void GenericServicePlugin::configureService(
  const std::string & service_name,
  const std::string & srv_type,
  int timeout_ms)
{
  service_name_ = service_name;
  srv_type_ = srv_type;
  timeout_ms_ = timeout_ms;

  // Get type support from runtime introspection
  type_support_ = conversion::getServiceTypeSupport(srv_type);

  // Extract request and response message members from service type support
  const auto * service_members = static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    type_support_->data);

  // Create type support structs for request and response
  request_type_support_.typesupport_identifier = rosidl_typesupport_introspection_cpp::typesupport_identifier;
  request_type_support_.data = service_members->request_members_;
  request_type_support_.func = nullptr;

  response_type_support_.typesupport_identifier = rosidl_typesupport_introspection_cpp::typesupport_identifier;
  response_type_support_.data = service_members->response_members_;
  response_type_support_.func = nullptr;

  // Create generic client using the factory function
  client_ = rclcpp::create_generic_client(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_services_interface(),
    service_name_,
    srv_type_,
    rclcpp::ServicesQoS(),
    nullptr  // callback group
  );
}

nlohmann::json GenericServicePlugin::callService(const nlohmann::json & request)
{
  if (!client_) {
    throw std::runtime_error(
      "Service client not initialized for primitive: " + primitive_name_);
  }

  // Wait for service to be available
  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error(
      "Service not available: " + service_name_);
  }

  // Get request/response message members for allocation
  const auto * request_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    request_type_support_.data);
  const auto * response_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    response_type_support_.data);

  // Allocate ROS request message
  std::vector<uint8_t> ros_request_buffer(request_members->size_of_);
  void * ros_request = ros_request_buffer.data();
  request_members->init_function(ros_request, rosidl_runtime_cpp::MessageInitialization::ZERO);

  // Allocate ROS response message
  std::vector<uint8_t> ros_response_buffer(response_members->size_of_);
  void * ros_response = ros_response_buffer.data();
  response_members->init_function(ros_response, rosidl_runtime_cpp::MessageInitialization::ZERO);

  try {
    // Convert JSON to ROS request using introspection
    conversion::jsonToRosMessage(request, ros_request, &request_type_support_);

    // Serialize request
    rclcpp::SerializedMessage serialized_request;
    rclcpp::SerializationBase request_serializer(&request_type_support_);
    request_serializer.serialize_message(ros_request, &serialized_request);

    // Call service (GenericClient expects void* to serialized data)
    auto future = client_->async_send_request(static_cast<void*>(&serialized_request));
    auto status = future.wait_for(std::chrono::milliseconds(timeout_ms_));

    if (status != std::future_status::ready) {
      // Cleanup
      request_members->fini_function(ros_request);
      response_members->fini_function(ros_response);
      throw std::runtime_error("Service call timeout: " + service_name_);
    }

    // Get serialized response (returns std::shared_ptr<void> which is actually a SerializedMessage*)
    auto serialized_response_ptr = future.get();
    auto * serialized_response = static_cast<rclcpp::SerializedMessage*>(serialized_response_ptr.get());

    // Deserialize response
    rclcpp::SerializationBase response_serializer(&response_type_support_);
    response_serializer.deserialize_message(serialized_response, ros_response);

    // Convert ROS response to JSON using introspection
    nlohmann::json json_response = conversion::rosMessageToJson(ros_response, &response_type_support_);

    // Cleanup
    request_members->fini_function(ros_request);
    response_members->fini_function(ros_response);

    return json_response;

  } catch (...) {
    // Cleanup on exception
    request_members->fini_function(ros_request);
    response_members->fini_function(ros_response);
    throw;
  }
}

}  // namespace robot_mcp_generic_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_mcp_generic_plugins::GenericServicePlugin, robot_mcp_msg_pluginlib::MessagePlugin)
