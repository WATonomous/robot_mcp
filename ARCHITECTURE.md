# robot_mcp Architecture & Implementation Plan

**Last Updated:** 2025-10-17

---

## Table of Contents

1. [Overview](#overview)
2. [Repository Structure](#repository-structure)
3. [Core Architecture](#core-architecture)
4. [Component Breakdown](#component-breakdown)
5. [Implementation Plan](#implementation-plan)
6. [Design Decisions](#design-decisions)

---

## Overview

`robot_mcp` is a Model Context Protocol (MCP) server for ROS2 robots that enables AI assistants (like Claude) to control robots through natural language via HTTP.

**Key Features:**
- Configuration-driven ROS2 HTTP server
- Pure C++ implementation using `rclcpp_lifecycle`
- HTTP-based (not stdio) for network accessibility
- Plugin-based message system with autodiscovery
- Inheritance-based plugin design (base plugins implement infrastructure)
- Resource conflict management for concurrency control
- Multi-distro ROS2 support

---

## Repository Structure

```
robot_mcp/
├── robot_mcp_server/                 # Main HTTP MCP server (C++)
|   ├── include
|   │   └── robot_mcp_server
|   │       ├── mcp_http_server       # HTTP server layer, handles mcp as well
|   │       ├── mcp_router            # Orchestrates and routes requests to various plugins
|   │       ├── mcp_factory           # Plugin loader & factory
|   |       ├── mcp_config            # Handles ros2 parameter parsing
|   |       └── robot_mcp_server_node.hpp # Central implementation that brings everything together
│   ├── src/
│   ├── test/                         # Unit & integration tests
│   ├── package.xml
│   └── CMakeLists.txt
│
├── robot_mcp_msg_pluginlib/          # Base plugin interface (CORE - header-only)
│   ├── include/
│   │   └── robot_mcp_msg_pluginlib/
│   │       ├── message_plugin.hpp   # Base class for all message plugins
│   │       └── visibility_control.h # Symbol visibility
│   ├── package.xml
│   └── CMakeLists.txt
│
├── robot_mcp_common_msg_plugins/     # Directory containing all message plugin packages
│   │
│   ├── robot_common_msg_plugins/     # Metapackage (aggregates common plugins)
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── robot_mcp_std_msg_plugins/    # std_msgs plugin implementations
│   │   ├── include/                  # (to be populated in Phase 3)
│   │   ├── src/                      # (to be populated in Phase 3)
│   │   ├── plugins.xml
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── robot_mcp_geometry_msg_plugins/  # geometry_msgs plugin implementations
│   │   ├── include/
│   │   ├── src/
│   │   ├── plugins.xml
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── robot_mcp_sensor_msg_plugins/    # sensor_msgs plugin implementations
│   │   ├── include/
│   │   ├── src/
│   │   ├── plugins.xml
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   └── robot_mcp_nav_msg_plugins/       # nav_msgs plugin implementations
│       ├── include/
│       ├── src/
│       ├── plugins.xml
│       ├── package.xml
│       └── CMakeLists.txt
│
└── robot_mcp_bringup/                # Launch files and example configs
    ├── config/
    │   └── presets/                  # Example configurations (to be populated)
    ├── launch/
    │   └── robot_mcp.launch.yaml     # YAML launch file
    ├── package.xml
    └── CMakeLists.txt
```

---

## Core Architecture

```
┌─────────────────────────────────────────────────────┐
│              HTTP Server (cpp-httplib)               │
│  ┌──────────────┐  ┌───────────────┐               │
│  │ Auth Layer   │→ │ JSON-RPC      │               │
│  │ (API Key)    │  │ Parser        │               │
│  └──────────────┘  └───────────────┘               │
└─────────────────────┬───────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────┐
│            MCP Protocol Handler                      │
│  ┌────────────┐ ┌────────────┐ ┌─────────────────┐ │
│  │ initialize │ │ tools/*    │ │ resources/*     │ │
│  │ ping       │ │ list, call │ │ list, read      │ │
│  └────────────┘ └────────────┘ └─────────────────┘ │
└─────────────────────┬───────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────┐
│              Router & Orchestration                  │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │ Resource Conflict Manager                   │   │
│  │ (Check conflicts before dispatching)        │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │ Operation Tracker                           │   │
│  │ (Track active operations by operation_id)   │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
│  Route request → Dispatch to appropriate plugin     │
└─────────────────────┬───────────────────────────────┘
                      │
                      │ Dispatch based on message type
                      │
          ┌───────────┼───────────┐
          │           │           │
          ▼           ▼           ▼
┌────────────┐ ┌────────────┐ ┌────────────┐
│   Topic    │ │  Service   │ │   Action   │
│  Plugins   │ │  Plugins   │ │  Plugins   │
└──────┬─────┘ └──────┬─────┘ └──────┬─────┘
       │              │              │
       │ Each plugin creates & manages its own ROS2 constructs
       │              │              │
       ▼              ▼              ▼
┌────────────┐ ┌────────────┐ ┌────────────┐
│ Publishers │ │ Service    │ │ Action     │
│ Subscriber │ │ Clients    │ │ Clients    │
│  + Cache   │ │            │ │ + Tracking │
└────────────┘ └────────────┘ └────────────┘

┌──────────────────────────────────────────────────────┐
│         Support Systems (Shared Infrastructure)      │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │ Lifecycle Node (rclcpp_lifecycle)           │   │
│  │ - Provided to plugins during initialize()   │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
│  ┌────────────┐ ┌────────────┐ ┌──────────────────┐│
│  │ Plugin     │ │ Config     │ │ Built-in Tools   ││
│  │ Loader &   │ │ Parser     │ │ (status, list,   ││
│  │ Registry   │ │ (YAML)     │ │  cancel)         ││
│  └────────────┘ └────────────┘ └──────────────────┘│
└──────────────────────────────────────────────────────┘
```

**Key Architectural Principles:**

1. **Plugin-Centric**: Plugins are self-contained and handle all ROS2 interaction for their message type
2. **Thin Router**: Router focuses on orchestration (routing, conflicts, tracking), not message handling
3. **Autodiscovery**: Plugins discovered automatically via pluginlib at startup
4. **Extensibility**: Users create custom plugins without modifying core

### Plugin System Flow

**At Startup (Autodiscovery):**
```
1. Plugin Loader scans all packages with pluginlib manifests (plugins.xml)
2. Discovers available message plugins:
   - robot_mcp_std_msg_plugins::StringPlugin
   - robot_mcp_geometry_msg_plugins::PosePlugin
   - robot_mcp_sensor_msg_plugins::BatteryStatePlugin
   - custom_my_robot_msg_plugins::CustomMessagePlugin
   - etc.
3. Builds registry: "sensor_msgs/msg/BatteryState" → BatteryStatePlugin class
```

**At Configuration (Factory Pattern):**
```
1. Parse YAML config:
   topics:
     - name: "battery"
       topic: "/battery_state"
       msg_type: "sensor_msgs/msg/BatteryState"
2. Factory looks up "sensor_msgs/msg/BatteryState" in plugin registry
3. Factory instantiates BatteryStatePlugin
4. Plugin.initialize(node) is called
5. Plugin.subscribe("/battery_state") creates subscriber and begins caching
```

**At Runtime (Message Handling):**
```
1. ROS2 topic receives sensor_msgs/msg/BatteryState message
2. Plugin's subscriber callback: plugin->toJson(msg) → JSON data
3. Plugin caches JSON data with timestamp
4. When Claude requests resource: router calls plugin->readLatest() → return cached JSON
```

### Data Flow Example

**User Request:** "Navigate to pose X"

```
1. Claude Desktop → HTTP POST /mcp
2. HTTP Server → Auth Middleware → JSON-RPC Parser
3. MCP Protocol Handler → tools/call handler
4. Router checks resource groups for conflicts
5. Router looks up NavigateToPosePlugin from registry
6. Router calls plugin->sendGoal(action_name, json_goal, operation_id)
7. Plugin validates goal via validateGoal() hook
8. Plugin converts JSON → ROS2 action goal message via fromJson()
9. Plugin creates/reuses action client and submits goal
10. Plugin stores goal handle for tracking
11. Router's operation tracker records operation_id → plugin mapping
12. Return operation_id to Claude (or result if shouldBlock() is true)
13. Claude can poll status via built-in tools (router → plugin->getStatus())
14. Claude can cancel via built-in tools (router → plugin->cancel())
```



**Plugin Architecture Philosophy:**

Each message plugin defines **how that message type is handled**, not just how it's converted. This allows:
- Users to extend base plugins with custom validation, preprocessing, QoS settings
- Separation by message type rather than by operation type
- Easier custom implementations without modifying core (just create a new plugin)

**Plugin Base Classes (`robot_mcp_msg_pluginlib/`):**

```cpp
namespace robot_mcp_msg_pluginlib {

// ============================================================================
// Base interface - all plugins must implement
// ============================================================================
class MessagePlugin {
public:
    virtual ~MessagePlugin() = default;

    // Return the ROS2 message type this plugin handles
    virtual std::string getMessageType() const = 0;

    // Convert ROS2 message to JSON
    virtual nlohmann::json toJson(const void* msg_ptr) const = 0;

    // Convert JSON to ROS2 message
    virtual void fromJson(const nlohmann::json& j, void* msg_ptr) const = 0;

    // Get message type info for dynamic type handling
    virtual const rosidl_message_type_support_t* getTypeSupport() const = 0;
};

// ============================================================================
// TopicPlugin - for messages used in pub/sub
// ============================================================================
class TopicPlugin : public MessagePlugin {
public:
    // Initialize plugin with node (called once at startup)
    virtual void initialize(rclcpp::Node::SharedPtr node) {
        node_ = node;
    }

    // Publish a message to a topic
    // Router calls this when MCP client wants to publish
    virtual void publish(const std::string& topic_name, const nlohmann::json& msg) {
        // Plugin creates publisher if needed, validates, converts, and publishes
        // Default implementation provided in base class
    }

    // Subscribe to a topic and cache latest message
    // Router calls this during configuration/startup
    virtual void subscribe(const std::string& topic_name) {
        // Plugin creates subscriber if needed and caches messages
        // Default implementation provided in base class
    }

    // Read the latest cached message
    // Router calls this when MCP client reads a resource
    virtual nlohmann::json readLatest(const std::string& topic_name) const {
        // Return cached message
        // Default implementation provided in base class
    }

    // Get default QoS profile for this message type
    virtual rclcpp::QoS getDefaultQoS() const {
        return rclcpp::QoS(10);  // Default: reliable, volatile, depth 10
    }

    // Validate message before publishing (optional hook)
    virtual void validateForPublish(const nlohmann::json& msg) const {
        // Default: no validation
    }

    // Postprocess message after subscription (optional hook)
    virtual nlohmann::json postprocessAfterSubscribe(
        const nlohmann::json& msg) const {
        return msg;  // Default: no postprocessing
    }

    // Whether to cache this topic's latest message
    virtual bool shouldCache() const {
        return true;  // Default: cache for resources/read
    }

    // How long to cache (seconds), 0 = forever
    virtual double getCacheDuration() const {
        return 0.0;  // Default: cache forever
    }

protected:
    rclcpp::Node::SharedPtr node_;
    // Base class manages publishers, subscribers, and cache
    // Derived classes can override behavior but get default implementation
};

// ============================================================================
// ServicePlugin - for messages used in services (request/response pairs)
// ============================================================================
class ServicePlugin : public MessagePlugin {
public:
    // Initialize plugin with node (called once at startup)
    virtual void initialize(rclcpp::Node::SharedPtr node) {
        node_ = node;
    }

    // Call a service
    // Router calls this when MCP client wants to call a service
    // Returns the response as JSON
    virtual nlohmann::json callService(
        const std::string& service_name,
        const nlohmann::json& request) {
        // Plugin creates service client if needed, validates, converts, and calls
        // Default implementation provided in base class
        // Blocks until response or timeout
    }

    // Get default timeout for service calls (milliseconds)
    virtual int getDefaultTimeout() const {
        return 5000;  // Default: 5 seconds
    }

    // Validate request before sending (optional hook)
    virtual void validateRequest(const nlohmann::json& request) const {
        // Default: no validation
    }

    // Postprocess response before returning to client (optional hook)
    virtual nlohmann::json postprocessResponse(
        const nlohmann::json& response) const {
        return response;  // Default: no postprocessing
    }

    // Whether to retry on failure
    virtual bool shouldRetry() const {
        return false;  // Default: no retry
    }

    // Get message type support for service
    virtual const rosidl_service_type_support_t* getServiceTypeSupport() const = 0;

protected:
    rclcpp::Node::SharedPtr node_;
    // Base class manages service clients
    // Derived classes can override behavior but get default implementation
};

// ============================================================================
// ActionPlugin - for messages used in actions (goal/feedback/result)
// ============================================================================
class ActionPlugin : public MessagePlugin {
public:
    // Initialize plugin with node (called once at startup)
    virtual void initialize(rclcpp::Node::SharedPtr node) {
        node_ = node;
    }

    // Send an action goal
    // Router calls this when MCP client wants to execute an action
    // Returns operation_id if non-blocking, or result if blocking
    virtual nlohmann::json sendGoal(
        const std::string& action_name,
        const nlohmann::json& goal,
        std::string& operation_id) {
        // Plugin creates action client if needed, validates, converts, and sends goal
        // Default implementation provided in base class
        // If shouldBlock() is true, waits for result and returns it
        // If shouldBlock() is false, returns immediately with operation_id
    }

    // Get status of an active action
    // Router calls this when MCP client queries operation status
    virtual nlohmann::json getStatus(const std::string& operation_id) const {
        // Returns current status: pending, active, succeeded, failed, cancelled
        // Includes latest feedback if available
    }

    // Cancel an active action
    // Router calls this when MCP client cancels an operation
    virtual void cancel(const std::string& operation_id) {
        // Sends cancel request to action server
    }

    // Get default timeout for action (milliseconds), 0 = no timeout
    virtual int getDefaultTimeout() const {
        return 0;  // Default: no timeout for actions
    }

    // Validate goal before sending (optional hook)
    virtual void validateGoal(const nlohmann::json& goal) const {
        // Default: no validation
    }

    // Postprocess feedback before returning to client (optional hook)
    virtual nlohmann::json postprocessFeedback(
        const nlohmann::json& feedback) const {
        return feedback;  // Default: no postprocessing
    }

    // Postprocess result before returning to client (optional hook)
    virtual nlohmann::json postprocessResult(
        const nlohmann::json& result) const {
        return result;  // Default: no postprocessing
    }

    // Determine if action should block until complete or return immediately
    // Based on estimated duration or complexity
    virtual bool shouldBlock() const {
        return false;  // Default: return operation_id immediately
    }

    // Whether action can be cancelled
    virtual bool isCancellable() const {
        return true;  // Default: allow cancellation
    }

    // Get message type support for action
    virtual const rosidl_action_type_support_t* getActionTypeSupport() const = 0;

protected:
    rclcpp::Node::SharedPtr node_;
    // Base class manages action clients and tracks active goals
    // Derived classes can override behavior but get default implementation
};

}  // namespace robot_mcp_msg_pluginlib
```

**Key Architectural Insight: Inheritance-Based Design**

The plugin architecture uses the **Template Method pattern** with a well-designed inheritance hierarchy. This achieves:

1. **Base plugins do ALL the heavy lifting**
   - `TopicPlugin` implements complete publisher/subscriber/caching infrastructure
   - `ServicePlugin` implements complete service client management
   - `ActionPlugin` implements complete action client/goal tracking
   - Concrete plugins inherit this for free

2. **Concrete plugins are trivial (90% of cases)**
   - Only implement 3-4 methods: `toJson()`, `fromJson()`, `getTypeSupport()`, `getMessageType()`
   - Pure conversion logic - easy to test, easy to write
   - No ROS2 infrastructure code needed

3. **Progressive enhancement (customize only what you need)**
   - **Level 1**: Inherit defaults (most plugins) - just conversion
   - **Level 2**: Override hooks (QoS, validation) - customize behavior
   - **Level 3**: Override methods (publish, subscribe) - fully custom

4. **No code duplication**
   - Infrastructure implemented once in base classes
   - Tested once, reused everywhere
   - Concrete plugins focus on message-specific logic only

5. **Open/Closed Principle**
   - Extend without modifying core
   - Add new message types without touching infrastructure
   - Users can create custom base classes for their robot

**Implementation Depth Examples:**

```cpp
// ============================================================================
// Level 1: Minimal Plugin (90% of plugins) - TRIVIAL TO WRITE
// ============================================================================
class BatteryStatePlugin : public TopicPlugin {
    // ONLY need these 4 methods - inherits ALL infrastructure:
    //   - Publisher/subscriber management
    //   - Message caching
    //   - Default QoS (reliable, volatile, depth 10)
    //   - Error handling

    std::string getMessageType() const override;
    nlohmann::json toJson(const void* msg_ptr) const override;
    void fromJson(const nlohmann::json& j, void* msg_ptr) const override;
    const rosidl_message_type_support_t* getTypeSupport() const override;
};

// ============================================================================
// Level 2: Custom Hooks (customize behavior, not infrastructure)
// ============================================================================
class TwistPlugin : public TopicPlugin {
    // Conversion methods (required)
    std::string getMessageType() const override;
    nlohmann::json toJson(const void* msg_ptr) const override;
    void fromJson(const nlohmann::json& j, void* msg_ptr) const override;
    const rosidl_message_type_support_t* getTypeSupport() const override;

    // CUSTOMIZE: QoS for velocity commands
    rclcpp::QoS getDefaultQoS() const override {
        return rclcpp::QoS(1).best_effort();  // Low latency
    }

    // CUSTOMIZE: Validate velocity limits
    void validateForPublish(const nlohmann::json& msg) const override {
        double linear_x = msg["linear"]["x"];
        if (std::abs(linear_x) > 1.0) {
            throw std::runtime_error("Velocity exceeds limit");
        }
    }

    // CUSTOMIZE: Don't cache velocity commands
    bool shouldCache() const override { return false; }
};

// ============================================================================
// Level 3: Full Override (rare - only when you need complete control)
// ============================================================================
class SpecialImagePlugin : public TopicPlugin {
    // Conversion methods (required)
    std::string getMessageType() const override;
    nlohmann::json toJson(const void* msg_ptr) const override;
    void fromJson(const nlohmann::json& j, void* msg_ptr) const override;
    const rosidl_message_type_support_t* getTypeSupport() const override;

    // FULL OVERRIDE: Custom publish with compression
    void publish(const std::string& topic_name, const nlohmann::json& msg) override {
        // Custom implementation: compress image before publishing
        auto compressed = compressImage(msg);
        // ... custom publisher logic
    }

    // FULL OVERRIDE: Custom subscribe with decompression
    void subscribe(const std::string& topic_name) override {
        // Custom implementation: decompress on receive
        // ... custom subscriber logic
    }
};
```

**Why This Design Wins:**

| Aspect | Alternative (Manager Services) | Our Design (Base Plugin) |
|--------|-------------------------------|-------------------------|
| **Concrete plugin complexity** | Medium (needs manager APIs) | Trivial (3-4 methods) |
| **Infrastructure code** | Centralized managers | Base classes (reusable) |
| **Extensibility** | Add to manager | Inherit & override |
| **Testing** | Mock manager services | Mock node only |
| **Code duplication** | None (good) | None (also good) |
| **Message-specific behavior** | Hard to customize | Easy (override hooks) |
| **Open/Closed Principle** | Medium | Excellent |

The inheritance-based approach gives us the best of both worlds: shared infrastructure without sacrificing flexibility.

**Plugin Implementation Examples:**

```cpp
// ============================================================================
// Example 1: Simple topic plugin with default behavior
// ============================================================================
// In robot_mcp_sensor_msg_plugins/plugins/battery_state_plugin.hpp
class BatteryStatePlugin : public robot_mcp_msg_pluginlib::TopicPlugin {
public:
    std::string getMessageType() const override {
        return "sensor_msgs/msg/BatteryState";
    }

    nlohmann::json toJson(const void* msg_ptr) const override {
        auto msg = static_cast<const sensor_msgs::msg::BatteryState*>(msg_ptr);
        return {
            {"voltage", msg->voltage},
            {"percentage", msg->percentage},
            {"current", msg->current},
            {"power_supply_status", msg->power_supply_status}
        };
    }

    void fromJson(const nlohmann::json& j, void* msg_ptr) const override {
        auto msg = static_cast<sensor_msgs::msg::BatteryState*>(msg_ptr);
        msg->voltage = j.value("voltage", 0.0);
        msg->percentage = j.value("percentage", 0.0);
        msg->current = j.value("current", 0.0);
        msg->power_supply_status = j.value("power_supply_status", 0);
    }

    const rosidl_message_type_support_t* getTypeSupport() const override {
        return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState);
    }

    // Use default QoS, caching, etc. from TopicPlugin base class
};

// ============================================================================
// Example 2: Topic plugin with custom QoS and validation
// ============================================================================
// In robot_mcp_geometry_msg_plugins/plugins/twist_plugin.hpp
class TwistPlugin : public robot_mcp_msg_pluginlib::TopicPlugin {
public:
    std::string getMessageType() const override {
        return "geometry_msgs/msg/Twist";
    }

    nlohmann::json toJson(const void* msg_ptr) const override {
        auto msg = static_cast<const geometry_msgs::msg::Twist*>(msg_ptr);
        return {
            {"linear", {
                {"x", msg->linear.x},
                {"y", msg->linear.y},
                {"z", msg->linear.z}
            }},
            {"angular", {
                {"x", msg->angular.x},
                {"y", msg->angular.y},
                {"z", msg->angular.z}
            }}
        };
    }

    void fromJson(const nlohmann::json& j, void* msg_ptr) const override {
        auto msg = static_cast<geometry_msgs::msg::Twist*>(msg_ptr);
        msg->linear.x = j["linear"]["x"];
        msg->linear.y = j["linear"]["y"];
        msg->linear.z = j["linear"]["z"];
        msg->angular.x = j["angular"]["x"];
        msg->angular.y = j["angular"]["y"];
        msg->angular.z = j["angular"]["z"];
    }

    const rosidl_message_type_support_t* getTypeSupport() const override {
        return ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    }

    // Custom QoS for velocity commands - best effort, no caching
    rclcpp::QoS getDefaultQoS() const override {
        return rclcpp::QoS(1).best_effort();
    }

    // Validate velocity limits before publishing
    void validateForPublish(const nlohmann::json& msg) const override {
        double linear_x = msg["linear"]["x"];
        double angular_z = msg["angular"]["z"];

        if (std::abs(linear_x) > 1.0) {
            throw std::runtime_error("Linear velocity exceeds limit (1.0 m/s)");
        }
        if (std::abs(angular_z) > 2.0) {
            throw std::runtime_error("Angular velocity exceeds limit (2.0 rad/s)");
        }
    }

    // Don't cache velocity commands
    bool shouldCache() const override {
        return false;
    }
};

// ============================================================================
// Example 3: Action plugin with custom behavior
// ============================================================================
// In robot_mcp_nav_msg_plugins/plugins/navigate_to_pose_plugin.hpp
class NavigateToPosePlugin : public robot_mcp_msg_pluginlib::ActionPlugin {
public:
    std::string getMessageType() const override {
        return "nav2_msgs/action/NavigateToPose";
    }

    // ... toJson/fromJson implementations ...

    const rosidl_action_type_support_t* getActionTypeSupport() const override {
        return ROSIDL_GET_ACTION_TYPE_SUPPORT(nav2_msgs, NavigateToPose);
    }

    // Validate pose is reachable
    void validateGoal(const nlohmann::json& goal) const override {
        if (!goal.contains("pose")) {
            throw std::runtime_error("Goal must contain 'pose'");
        }
        // Additional validation...
    }

    // Add estimated distance to feedback
    nlohmann::json postprocessFeedback(
        const nlohmann::json& feedback) const override {
        auto processed = feedback;
        // Could add distance remaining, ETA, etc.
        return processed;
    }

    // Navigation takes a while - don't block
    bool shouldBlock() const override {
        return false;  // Return operation_id immediately
    }
};

// Export plugins (pluginlib)
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_mcp_sensor_msg_plugins::BatteryStatePlugin,
                       robot_mcp_msg_pluginlib::MessagePlugin)
PLUGINLIB_EXPORT_CLASS(robot_mcp_geometry_msg_plugins::TwistPlugin,
                       robot_mcp_msg_pluginlib::MessagePlugin)
PLUGINLIB_EXPORT_CLASS(robot_mcp_nav_msg_plugins::NavigateToPosePlugin,
                       robot_mcp_msg_pluginlib::MessagePlugin)
```

**Plugin Manifest (plugins.xml):**
```xml
<library path="robot_mcp_sensor_msg_plugins">
  <class type="robot_mcp_sensor_msg_plugins::BatteryStatePlugin"
         base_class_type="robot_mcp_msg_pluginlib::MessagePlugin">
    <description>Plugin for sensor_msgs/msg/BatteryState</description>
  </class>
  <!-- More plugins... -->
</library>
```

## Implementation Plan

### 1. Skeleton & Configuration
- Create `robot_mcp_server_node.hpp` with lifecycle node skeleton
- Implement configuration parsing (`config_types.hpp`, `config_parser.cpp`)
- Define YAML structure for topics, services, actions, resource groups
- Load and validate configuration on lifecycle configure transition

### 2. HTTP Server Layer
- Implement HTTP server wrapper around cpp-httplib (`http_server.cpp`)
- Add JSON-RPC 2.0 parsing and error handling (`json_rpc_handler.cpp`)
- Implement optional API key authentication (`auth_middleware.cpp`)
- Start/stop HTTP server on lifecycle activate/deactivate

### 3. MCP Router Implementation
- Implement MCP protocol handlers (`mcp_server.cpp`, `initialize_handler.cpp`, `tools_handler.cpp`, `resources_handler.cpp`)
- Create router to dispatch MCP requests to plugins (`router.cpp`)
- Add operation tracking for actions (`operation_tracker.cpp`)
- Implement resource conflict checking (`resource_management/`)

### 4. Plugin System (pluginlib & factory)
- Define base plugin interfaces in `robot_mcp_msg_pluginlib` (`message_plugin.hpp`)
- Implement `TopicPlugin`, `ServicePlugin`, `ActionPlugin` base classes with full infrastructure
- Create plugin loader, factory, and registry (`plugin_system/`)
- Base classes handle all ROS2 interaction (publishers, subscribers, clients, caching)

### 5. Basic Message Plugins
- Implement concrete plugins inheriting from base classes:
  - **std_msgs**: String, Int32, Float64, Bool
  - **geometry_msgs**: Pose, PoseStamped, Twist, Transform
  - **sensor_msgs**: BatteryState, LaserScan, Image
  - **nav_msgs**: Odometry, Path
- Each plugin implements 4 methods: `getMessageType()`, `toJson()`, `fromJson()`, `getTypeSupport()`
- Create `plugins.xml` manifests for autodiscovery

**Note:** Plugins are trivial (20-50 lines) because base classes provide all infrastructure. Focus effort on base classes (step 4), then concrete plugins are easy.

---

## Design Decisions

### Multi-Distro ROS2 Support

**Decision:** Target all ROS2 distributions (Humble, Iron, Jazzy, etc.)

**Implementation:**
- Use standard `rclcpp` APIs available in all distros
- Avoid distro-specific features
- Test on Humble (LTS) and Jazzy (latest LTS)
- Document any known compatibility issues

### Authentication Default

**Decision:** Auth disabled by default in example configs

**Rationale:**
- Easier development and testing experience
- Most users will be in trusted networks initially
- Document security setup clearly for production
- Easy to enable via config when needed

### Server-Sent Events (SSE)

**Decision:** Defer SSE to Phase 13+ (post-MVP)

**Rationale:**
- Polling-based status checking is sufficient for MVP
- MCP client support for SSE is still maturing
- Adds ~2-3 weeks of complexity
- Can be added later without breaking changes

**Future Implementation:**
- Add SSE endpoint `/mcp/events`
- Stream action feedback in real-time
- Push topic updates at configured rate
- Notification on operation completion

### Message Type System: Inheritance-Based Plugin Architecture

**Decision:** Plugin-based architecture using pluginlib with autodiscovery AND inheritance-based design using Template Method pattern

**Key Innovation:** Base plugins (TopicPlugin, ServicePlugin, ActionPlugin) implement ALL infrastructure. Concrete plugins are trivial (3-4 methods).

**Advantages:**
- **Runtime autodiscovery** - pluginlib scans all packages automatically
- **Modular** - users install only needed plugin packages
- **Extensible** - anyone can create custom plugins without modifying core
- **No code duplication** - infrastructure implemented once in base classes
- **Easy to write** - 90% of plugins are ~20-50 lines (just conversion logic)
- **Easy to test** - pure conversion functions, no ROS2 mocking needed for unit tests
- **Progressive enhancement** - customize only what you need (3 levels of depth)
- **Follows ROS2 patterns** - similar to nav2, costmap_2d plugin systems

**Architecture Layers:**
1. **Base interface** (`MessagePlugin`): Minimal interface all plugins implement
2. **Base plugins** (`TopicPlugin`, `ServicePlugin`, `ActionPlugin`): Implement ROS2 infrastructure
   - Publisher/subscriber management
   - Service client management
   - Action client and goal tracking
   - Caching, QoS, error handling
3. **Concrete plugins**: Inherit from base plugins, implement only conversion
   - Example: `BatteryStatePlugin` extends `TopicPlugin`
   - Only implements: `toJson()`, `fromJson()`, `getTypeSupport()`, `getMessageType()`
4. **Custom plugins**: Users can extend at any level for custom behavior

**Why This Wins:**

Compared to alternative architecture with separate manager services:
- **Concrete plugins**: 3-4 methods vs medium complexity
- **Infrastructure**: Reusable base classes vs centralized managers
- **Testing**: Mock node only vs mock manager APIs
- **Message-specific behavior**: Easy to customize (override hooks) vs hard to customize
- **Extensibility**: Inherit & override vs modify managers
- **Code duplication**: None vs none (both good, but inheritance more flexible)

**Implementation Example:**
```cpp
// Base Plugin (in robot_mcp_msg_pluginlib - ONE IMPLEMENTATION)
class TopicPlugin : public MessagePlugin {
public:
    virtual void initialize(rclcpp::Node::SharedPtr node);
    virtual void publish(const std::string& topic, const nlohmann::json& msg);
    virtual void subscribe(const std::string& topic);
    virtual nlohmann::json readLatest(const std::string& topic) const;
    virtual rclcpp::QoS getDefaultQoS() const { return rclcpp::QoS(10); }
    // ... hooks for validation, caching, etc.
protected:
    std::map<std::string, rclcpp::Publisher> publishers_;
    std::map<std::string, rclcpp::Subscription> subscribers_;
    std::map<std::string, nlohmann::json> cache_;
};

// Concrete Plugin (in robot_mcp_sensor_msg_plugins - TRIVIAL)
class BatteryStatePlugin : public TopicPlugin {
public:
    std::string getMessageType() const override {
        return "sensor_msgs/msg/BatteryState";
    }

    nlohmann::json toJson(const void* msg_ptr) const override {
        auto msg = static_cast<const sensor_msgs::msg::BatteryState*>(msg_ptr);
        return {{"voltage", msg->voltage}, {"percentage", msg->percentage}};
    }

    void fromJson(const nlohmann::json& j, void* msg_ptr) const override {
        auto msg = static_cast<sensor_msgs::msg::BatteryState*>(msg_ptr);
        msg->voltage = j["voltage"];
        msg->percentage = j["percentage"];
    }

    const rosidl_message_type_support_t* getTypeSupport() const override {
        return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState);
    }

    // Inherits ALL infrastructure: publishers, subscribers, caching, QoS, etc.
};

PLUGINLIB_EXPORT_CLASS(robot_mcp_sensor_msg_plugins::BatteryStatePlugin,
                       robot_mcp_msg_pluginlib::MessagePlugin)
```

**Progressive Enhancement (3 Levels):**
- **Level 1** (90% of plugins): Just conversion - inherit all infrastructure
- **Level 2** (10% of plugins): Override hooks for custom QoS, validation, caching
- **Level 3** (rare): Override methods for complete custom behavior

**User Benefits:**
- Install only needed plugin packages (e.g., just sensor_msgs if that's all you need)
- Create custom plugins in separate packages without modifying robot_mcp
- No name conflicts with ROS2 `*_msgs` packages (those define .msg files)

### Resource Conflict Management

**Decision:** Explicit resource group system with configurable resolution

**Rationale:**
- AI assistants may send parallel conflicting requests
- Need deterministic behavior, not race conditions
- Configuration allows robot-specific policies
- Three strategies handle different use cases:
  - `error`: Safe default, user handles retry
  - `cancel`: Convenient for interruptible actions
  - `queue`: For sequential operations

### HTTP vs stdio Transport

**Decision:** HTTP-only (no stdio support)

**Advantages:**
- Network accessible (control robot remotely)
- Multiple concurrent clients
- Standard debugging tools (curl, Postman)
- Better for production deployment
- No SSH tunneling needed

**Tradeoff:**
- Cannot use with local-only MCP clients
- Requires network setup
- Decided acceptable for robotic use case

---

## Open Questions / Future Enhancements

### Phase 13+: Advanced Features

1. **Server-Sent Events (SSE)**
   - Real-time action feedback streaming
   - Topic update push notifications
   - Reduces polling overhead

2. **Web Dashboard**
   - Monitor active operations
   - View robot state
   - Test tools manually
   - Debug interface issues

3. **Advanced Retry Logic**
   - Automatic retry for transient failures
   - Exponential backoff
   - Circuit breaker pattern

4. **Performance Optimizations**
   - Zero-copy message handling
   - Connection pooling
   - Response caching
   - Compression support

5. **Security Enhancements**
   - TLS/HTTPS support
   - OAuth2 authentication
   - Role-based access control
   - Audit logging

6. **Observability**
   - Prometheus metrics export
   - OpenTelemetry tracing
   - Structured logging to stdout

---

## Success Metrics

**MVP (Phases 1-8):**
- Can control TurtleBot3 via Claude Desktop
- Navigate, read sensors, publish velocity commands
- Basic status tracking works
- Configuration via YAML

**Production Ready (Phases 1-12):**
- 80%+ test coverage
- Documentation complete
- Debian package available
- Docker image published
- Sub-10ms HTTP latency
- Handles 1000+ req/s

**Community Adoption:**
- 100+ GitHub stars
- Used in 5+ different robot platforms
- Active community contributions
- Plugins for custom message types

---

## Contributing

See `CONTRIBUTING.md` for development guidelines, coding standards, and how to submit PRs.

## License

Apache 2.0 - See `LICENSE` file for details.
