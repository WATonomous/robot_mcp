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

#include "robot_mcp_generic_plugins/generic_topic_plugin.hpp"

#include <stdexcept>

#include <rclcpp/serialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

namespace robot_mcp_generic_plugins
{

GenericTopicPlugin::GenericTopicPlugin()
{
}

GenericTopicPlugin::~GenericTopicPlugin()
{
}

void GenericTopicPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & primitive_name,
  const std::string & description)
{
  node_ = node;
  primitive_name_ = primitive_name;
  description_ = description;
}

void GenericTopicPlugin::configureTopic(
  const std::string & topic_name,
  const std::string & msg_type,
  bool subscribe,
  bool publish)
{
  topic_name_ = topic_name;
  msg_type_ = msg_type;
  subscribe_enabled_ = subscribe;
  publish_enabled_ = publish;

  // Get type support from runtime introspection
  type_support_ = conversion::getMessageTypeSupport(msg_type);

  if (subscribe_enabled_) {
    // Create generic subscription
    auto callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      this->subscriptionCallback(msg);
    };

    subscription_ = node_->create_generic_subscription(
      topic_name_,
      msg_type_,
      rclcpp::QoS(10),
      callback);
  }

  if (publish_enabled_) {
    // Create generic publisher
    publisher_ = node_->create_generic_publisher(
      topic_name_,
      msg_type_,
      rclcpp::QoS(10));
  }
}

void GenericTopicPlugin::publishMessage(const nlohmann::json & msg)
{
  if (!publish_enabled_) {
    throw std::runtime_error(
      "Publishing not enabled for primitive: " + primitive_name_);
  }

  if (!publisher_) {
    throw std::runtime_error(
      "Publisher not initialized for primitive: " + primitive_name_);
  }

  // Get message members for allocation
  const auto * members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    type_support_->data);

  // Allocate ROS message
  std::vector<uint8_t> ros_msg_buffer(members->size_of_);
  void * ros_msg = ros_msg_buffer.data();

  // Initialize the message
  members->init_function(ros_msg, rosidl_runtime_cpp::MessageInitialization::ZERO);

  try {
    // Convert JSON to ROS message using introspection
    conversion::jsonToRosMessage(msg, ros_msg, type_support_);

    // Serialize the message
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::SerializationBase serializer(type_support_);
    serializer.serialize_message(ros_msg, &serialized_msg);

    // Publish
    publisher_->publish(serialized_msg);

    // Cleanup
    members->fini_function(ros_msg);
  } catch (...) {
    // Cleanup on exception
    members->fini_function(ros_msg);
    throw;
  }
}

nlohmann::json GenericTopicPlugin::readCachedMessage() const
{
  if (!subscribe_enabled_) {
    throw std::runtime_error(
      "Subscription not enabled for primitive: " + primitive_name_);
  }

  if (message_cache_.empty()) {
    throw std::runtime_error(
      "No cached messages available for primitive: " + primitive_name_);
  }

  return message_cache_.front();
}

void GenericTopicPlugin::subscriptionCallback(
  std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
{
  // Get message members for allocation
  const auto * members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    type_support_->data);

  // Allocate ROS message
  std::vector<uint8_t> ros_msg_buffer(members->size_of_);
  void * ros_msg = ros_msg_buffer.data();

  // Initialize the message
  members->init_function(ros_msg, rosidl_runtime_cpp::MessageInitialization::ZERO);

  try {
    // Deserialize
    rclcpp::SerializationBase serializer(type_support_);
    serializer.deserialize_message(serialized_msg.get(), ros_msg);

    // Convert to JSON using introspection
    nlohmann::json json_msg = conversion::rosMessageToJson(ros_msg, type_support_);

    // Add to cache
    message_cache_.push_front(json_msg);
    if (message_cache_.size() > MAX_CACHE_SIZE) {
      message_cache_.pop_back();
    }

    // Cleanup
    members->fini_function(ros_msg);
  } catch (const std::exception & e) {
    // Cleanup on exception
    members->fini_function(ros_msg);
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to deserialize/convert message on topic %s: %s",
      topic_name_.c_str(), e.what());
  }
}

}  // namespace robot_mcp_generic_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_mcp_generic_plugins::GenericTopicPlugin, robot_mcp_msg_pluginlib::MessagePlugin)
