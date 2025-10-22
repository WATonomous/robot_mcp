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

#include "robot_mcp_server/mcp_config/config_parser.hpp"

#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace robot_mcp::config
{

MCPServerConfig ConfigParser::parse(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  MCPServerConfig config;

  try {
    config.server = parseServerConfig(node);
    config.resource_groups = parseResourceGroups(node);
    config.topics = parseTopics(node);
    config.services = parseServices(node);
    config.actions = parseActions(node);

    validate(config);
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    throw ConfigParseException("Parameter not declared: " + std::string(e.what()));
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    throw ConfigParseException("Invalid parameter type: " + std::string(e.what()));
  } catch (const ConfigParseException &) {
    throw;
  } catch (const std::exception & e) {
    throw ConfigParseException("Unexpected error during parsing: " + std::string(e.what()));
  }

  return config;
}

ServerConfig ConfigParser::parseServerConfig(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  ServerConfig config;

  // Debug: List all parameters before declaring
  auto params_before = node->list_parameters({}, 0);
  RCLCPP_INFO(node->get_logger(), "Parameters before declaring: %zu", params_before.names.size());
  for (const auto & name : params_before.names) {
    RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
  }

  // Required parameters with defaults
  config.host = node->declare_parameter("server.host", config.host);
  config.port = node->declare_parameter("server.port", config.port);
  config.thread_pool_size = node->declare_parameter("server.thread_pool_size", config.thread_pool_size);
  config.max_connections = node->declare_parameter("server.max_connections", config.max_connections);
  config.timeout_ms = node->declare_parameter("server.timeout_ms", config.timeout_ms);
  config.enable_cors = node->declare_parameter("server.enable_cors", config.enable_cors);

  // SSL/HTTPS configuration
  config.enable_https = node->declare_parameter("server.enable_https", config.enable_https);
  config.ssl_cert_path = node->declare_parameter("server.ssl_cert_path", config.ssl_cert_path);
  config.ssl_key_path = node->declare_parameter("server.ssl_key_path", config.ssl_key_path);

  // Bond configuration for lifecycle manager
  config.bond_enabled = node->declare_parameter("server.bond_enabled", config.bond_enabled);
  config.bond_timeout = node->declare_parameter("server.bond_timeout", config.bond_timeout);
  config.bond_heartbeat_period = node->declare_parameter("server.bond_heartbeat_period", config.bond_heartbeat_period);

  // Optional API key - declare with empty string default
  std::string api_key_str = node->declare_parameter("server.api_key", std::string(""));
  if (!api_key_str.empty()) {
    config.api_key = api_key_str;
  }

  return config;
}

std::vector<TopicConfig> ConfigParser::parseTopics(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  std::vector<TopicConfig> topics;

  // Nav2 pattern: "topics" is a string array of topic names
  // Note: Parameters from YAML files aren't "declared" until we call declare_parameter
  auto topic_names = node->declare_parameter("topics", std::vector<std::string>{});

  if (topic_names.empty()) {
    RCLCPP_INFO(node->get_logger(), "No 'topics' parameter found or empty");
    return topics;  // No topics configured
  }

  RCLCPP_INFO(node->get_logger(), "Found %zu topics in config", topic_names.size());
  for (const auto & name : topic_names) {
    RCLCPP_INFO(node->get_logger(), "  Topic name: %s", name.c_str());
  }

  // Parse each topic by its name
  for (const auto & topic_name : topic_names) {
    std::string prefix = topic_name + ".";

    TopicConfig topic;
    topic.name = topic_name;
    topic.topic = node->declare_parameter(prefix + "topic", "");
    topic.msg_type = node->declare_parameter(prefix + "msg_type", "");
    topic.plugin = node->declare_parameter(prefix + "plugin", "");

    topic.subscribe = node->declare_parameter(prefix + "subscribe", topic.subscribe);
    topic.publish = node->declare_parameter(prefix + "publish", topic.publish);

    if (node->has_parameter(prefix + "resource_groups")) {
      topic.resource_groups = node->declare_parameter(prefix + "resource_groups", std::vector<std::string>{});
    }

    if (topic.topic.empty()) {
      throw ConfigParseException("Topic '" + topic.name + "' missing 'topic'");
    }
    if (topic.msg_type.empty()) {
      throw ConfigParseException("Topic '" + topic.name + "' missing 'msg_type'");
    }
    if (topic.plugin.empty()) {
      throw ConfigParseException("Topic '" + topic.name + "' missing 'plugin'");
    }

    topics.push_back(topic);
  }

  return topics;
}

std::vector<ServiceConfig> ConfigParser::parseServices(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  std::vector<ServiceConfig> services;

  // Nav2 pattern: "services" is a string array of service names
  // Note: Parameters from YAML files aren't "declared" until we call declare_parameter
  auto service_names = node->declare_parameter("services", std::vector<std::string>{});

  if (service_names.empty()) {
    return services;  // No services configured
  }

  // Parse each service by its name
  for (const auto & service_name : service_names) {
    std::string prefix = service_name + ".";

    ServiceConfig service;
    service.name = service_name;
    service.service = node->declare_parameter(prefix + "service", "");
    service.srv_type = node->declare_parameter(prefix + "srv_type", "");
    service.plugin = node->declare_parameter(prefix + "plugin", "");

    service.timeout_ms = node->declare_parameter(prefix + "timeout_ms", service.timeout_ms);

    if (node->has_parameter(prefix + "resource_groups")) {
      service.resource_groups = node->declare_parameter(prefix + "resource_groups", std::vector<std::string>{});
    }

    // Validate required fields
    if (service.service.empty()) {
      throw ConfigParseException("Service '" + service.name + "' missing 'service'");
    }
    if (service.srv_type.empty()) {
      throw ConfigParseException("Service '" + service.name + "' missing 'srv_type'");
    }
    if (service.plugin.empty()) {
      throw ConfigParseException("Service '" + service.name + "' missing 'plugin'");
    }

    services.push_back(service);
  }

  return services;
}

std::vector<ActionConfig> ConfigParser::parseActions(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  std::vector<ActionConfig> actions;

  // Nav2 pattern: "actions" is a string array of action names
  // Note: Parameters from YAML files aren't "declared" until we call declare_parameter
  auto action_names = node->declare_parameter("actions", std::vector<std::string>{});

  if (action_names.empty()) {
    return actions;  // No actions configured
  }

  // Parse each action by its name
  for (const auto & action_name : action_names) {
    std::string prefix = action_name + ".";

    ActionConfig action;
    action.name = action_name;
    action.action = node->declare_parameter(prefix + "action", "");
    action.action_type = node->declare_parameter(prefix + "action_type", "");
    action.plugin = node->declare_parameter(prefix + "plugin", "");

    action.timeout_ms = node->declare_parameter(prefix + "timeout_ms", action.timeout_ms);
    action.cancellable = node->declare_parameter(prefix + "cancellable", action.cancellable);

    if (node->has_parameter(prefix + "resource_groups")) {
      action.resource_groups = node->declare_parameter(prefix + "resource_groups", std::vector<std::string>{});
    }

    // Validate required fields
    if (action.action.empty()) {
      throw ConfigParseException("Action '" + action.name + "' missing 'action'");
    }
    if (action.action_type.empty()) {
      throw ConfigParseException("Action '" + action.name + "' missing 'action_type'");
    }
    if (action.plugin.empty()) {
      throw ConfigParseException("Action '" + action.name + "' missing 'plugin'");
    }

    actions.push_back(action);
  }

  return actions;
}

std::map<std::string, ResourceGroupConfig> ConfigParser::parseResourceGroups(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  std::map<std::string, ResourceGroupConfig> groups;

  // Nav2 pattern: "resource_groups" is a string array of group names
  // Note: Parameters from YAML files aren't "declared" until we call declare_parameter
  auto group_names = node->declare_parameter("resource_groups", std::vector<std::string>{});

  if (group_names.empty()) {
    return groups;  // No resource groups configured
  }

  // Parse each resource group by its name
  for (const auto & group_name : group_names) {
    std::string prefix = group_name + ".";

    ResourceGroupConfig group;
    group.name = group_name;

    group.max_concurrent = node->declare_parameter(prefix + "max_concurrent", group.max_concurrent);
    group.interruptible = node->declare_parameter(prefix + "interruptible", group.interruptible);

    if (node->has_parameter(prefix + "conflict_resolution")) {
      std::string cr_str = node->declare_parameter(prefix + "conflict_resolution", "error");
      group.conflict_resolution = parseConflictResolution(cr_str);
    }

    groups[group.name] = group;
  }

  return groups;
}

ConflictResolution ConfigParser::parseConflictResolution(const std::string & str)
{
  if (str == "error") {
    return ConflictResolution::ERROR;
  } else if (str == "cancel") {
    return ConflictResolution::CANCEL;
  } else if (str == "queue") {
    return ConflictResolution::QUEUE;
  } else {
    throw ConfigParseException(
      "Invalid conflict_resolution value: '" + str + "'. Must be 'error', 'cancel', or 'queue'");
  }
}

void ConfigParser::validate(const MCPServerConfig & config)
{
  // Validate server config
  if (config.server.port < 1 || config.server.port > 65535) {
    throw ConfigParseException(
      "Invalid port number: " + std::to_string(config.server.port) + ". Must be between 1 and 65535");
  }

  if (config.server.max_connections < 1) {
    throw ConfigParseException(
      "Invalid max_connections: " + std::to_string(config.server.max_connections) + ". Must be at least 1");
  }

  // Validate HTTPS configuration
  if (config.server.enable_https) {
    if (config.server.ssl_cert_path.empty()) {
      throw ConfigParseException("HTTPS is enabled but ssl_cert_path is not specified");
    }
    if (config.server.ssl_key_path.empty()) {
      throw ConfigParseException("HTTPS is enabled but ssl_key_path is not specified");
    }
  }

  // Check for duplicate names
  std::set<std::string> names;

  for (const auto & topic : config.topics) {
    if (names.count(topic.name)) {
      throw ConfigParseException("Duplicate name: '" + topic.name + "'");
    }
    names.insert(topic.name);
  }

  for (const auto & service : config.services) {
    if (names.count(service.name)) {
      throw ConfigParseException("Duplicate name: '" + service.name + "'");
    }
    names.insert(service.name);
  }

  for (const auto & action : config.actions) {
    if (names.count(action.name)) {
      throw ConfigParseException("Duplicate name: '" + action.name + "'");
    }
    names.insert(action.name);
  }

  // Validate resource group references
  for (const auto & topic : config.topics) {
    for (const auto & group_name : topic.resource_groups) {
      if (!config.resource_groups.count(group_name)) {
        throw ConfigParseException(
          "Topic '" + topic.name + "' references undefined resource group: '" + group_name + "'");
      }
    }
  }

  for (const auto & service : config.services) {
    for (const auto & group_name : service.resource_groups) {
      if (!config.resource_groups.count(group_name)) {
        throw ConfigParseException(
          "Service '" + service.name + "' references undefined resource group: '" + group_name + "'");
      }
    }
  }

  for (const auto & action : config.actions) {
    for (const auto & group_name : action.resource_groups) {
      if (!config.resource_groups.count(group_name)) {
        throw ConfigParseException(
          "Action '" + action.name + "' references undefined resource group: '" + group_name + "'");
      }
    }
  }
}

}  // namespace robot_mcp::config
