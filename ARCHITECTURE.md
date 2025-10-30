# robot_mcp Architecture & Implementation Plan

**Last Updated:** 2025-10-23

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Repository Structure](#repository-structure)
4. [Core Architecture](#core-architecture)
5. [Component Breakdown](#component-breakdown)
6. [Implementation Plan](#implementation-plan)
7. [Design Decisions](#design-decisions)

---

## Overview

`robot_mcp` is a Model Context Protocol (MCP) server for ROS2 robots that enables AI assistants (like Claude) to control robots through natural language via HTTP.

**Key Features:**
- Configuration-driven ROS2 HTTP server
- Pure C++ implementation using `rclcpp_lifecycle`
- HTTP-based (not stdio) for network accessibility
- **Generic plugin system** - 3 plugins support ALL message types
- Runtime type introspection for automatic JSON conversion
- Resource conflict management for concurrency control
- Multi-distro ROS2 support

---

## Terminology

**Primitive**: A configured MCP-ROS2 connection (logical concept)
- Defined in YAML config with name, ROS2 resource, type, and plugin
- Each primitive becomes one MCP tool or resource
- Example: `{name: "battery", topic: "/battery_state", msg_type: "sensor_msgs/msg/BatteryState", plugin: "BasicTopicPlugin"}`

**Plugin**: Backend implementation class (C++ code)
- Example: `BasicTopicPlugin`, `BasicServicePlugin`, `BasicActionPlugin`
- One plugin class can power many primitives
- Loaded via pluginlib from .so files
- Generic plugins support ALL message types via runtime introspection

**Plugin Instance**: Runtime object (one per primitive)
- Each primitive needs its own plugin instance for state (subscriptions, cache, etc.)
- Example: 3 topics using BasicTopicPlugin → 3 separate plugin instances
- Managed by `MCPPrimitiveRegistry`

---

## Repository Structure

```
robot_mcp/
├── robot_mcp_server/                 # Main HTTP MCP server (C++)
|   ├── include/robot_mcp_server/
|   │   ├── mcp_http_server/          # HTTP server layer
|   │   │   ├── http_server.hpp       # HTTPServer class
|   │   │   ├── auth_middleware.hpp   # API key authentication
|   │   │   └── json_rpc_handler.hpp  # JSON-RPC 2.0 parsing
|   │   ├── mcp_handler/              # MCP protocol handler
|   │   │   └── mcp_handler.hpp       # MCPHandler functor (initialize, ping, tools/*, resources/*)
|   │   ├── mcp_router/               # Request routing
|   │   │   ├── mcp_router.hpp        # MCPRouter class
|   │   │   └── operation_tracker.hpp # Operation ID tracking for actions
|   │   ├── mcp_primitive/            # Plugin infrastructure
|   │   │   ├── mcp_primitive_factory.hpp   # Creates registry from config
|   │   │   ├── mcp_primitive_registry.hpp  # Owns plugin instances
|   │   │   └── mcp_primitive_loader.hpp    # Pluginlib wrapper
|   │   ├── mcp_config/               # Configuration
|   │   │   ├── config_types.hpp      # Struct definitions
|   │   │   └── config_parser.hpp     # ROS2 parameter parsing
|   │   └── robot_mcp_server_node.hpp # Main lifecycle node
│   ├── src/
│   ├── test/                         # Catch2 unit tests + launch tests
│   ├── package.xml
│   └── CMakeLists.txt
│
├── robot_mcp_msg_pluginlib/          # Base plugin interface (CORE)
│   ├── include/robot_mcp_msg_pluginlib/
│   │   ├── message_plugin.hpp        # Base interface for all plugins
│   │   ├── basic_topic_plugin.hpp    # Generic topic plugin (all msg types)
│   │   ├── basic_service_plugin.hpp  # Generic service plugin (all srv types)
│   │   ├── basic_action_plugin.hpp   # Generic action plugin (all action types)
│   │   ├── message_conversion.hpp    # JSON ↔ ROS conversion utilities
│   │   └── visibility_control.h
│   ├── src/                          # Implementation of generic plugins
│   ├── plugins.xml                   # Pluginlib manifest for 3 basic plugins
│   ├── package.xml
│   └── CMakeLists.txt
│
└── robot_mcp_bringup/                # Launch files and example configs
    ├── config/
    │   ├── minimal.yaml              # Minimal example config
    │   └── presets/                  # Example configurations
    ├── launch/
    │   └── robot_mcp.launch.yaml     # YAML launch file
    ├── package.xml
    └── CMakeLists.txt
```

**Key Structural Notes:**
- **3 Generic Plugins**: `BasicTopicPlugin`, `BasicServicePlugin`, `BasicActionPlugin` in `robot_mcp_msg_pluginlib`
- **No type-specific plugin packages**: Generic plugins use runtime introspection to support all message types
- **Custom plugins**: Users can create custom plugin packages if they need specialized behavior

---

## Core Architecture

### Component Flow

```
┌─────────────────────────────────────────────────────────────┐
│                     HTTPServer (cpp-httplib)                 │
│  Handles HTTP requests/responses (httplib::Request/Response) │
│  ┌────────────────┐    ┌─────────────────┐                 │
│  │ AuthMiddleware │ →  │ JSONRPCHandler  │                 │
│  │ (API Key)      │    │ (HTTP → JSON)   │                 │
│  └────────────────┘    └─────────────────┘                 │
└────────────────────────────┬────────────────────────────────┘
                             │ nlohmann::json
                             │
┌────────────────────────────▼────────────────────────────────┐
│                     MCPHandler (Functor)                     │
│  Single class handling all 6 MCP methods:                   │
│  - initialize: Return server info                           │
│  - ping: Health check                                       │
│  - tools/list: List available tools (topics/services/actions)│
│  - tools/call: Execute tool (delegate to router)            │
│  - resources/list: List available resources (topics)        │
│  - resources/read: Read resource (delegate to router)       │
│                                                              │
│  asFunction() → std::function<json(json)> for HTTPServer    │
└────────────────────────────┬────────────────────────────────┘
                             │ Route to appropriate plugin
                             │
┌────────────────────────────▼────────────────────────────────┐
│                         MCPRouter                            │
│  Routes requests to correct plugin instance                 │
│  ┌────────────────────────────────────────────────┐        │
│  │ OperationTracker (owned by router)             │        │
│  │ - Tracks active action operation IDs           │        │
│  │ - Maps operation_id → plugin instance          │        │
│  └────────────────────────────────────────────────┘        │
│                                                              │
│  Non-owning reference to MCPPrimitiveRegistry               │
└────────────────────────────┬────────────────────────────────┘
                             │ Lookup plugin by primitive name
                             │
┌────────────────────────────▼────────────────────────────────┐
│                   MCPPrimitiveRegistry                       │
│  Owns all plugin instances (unique_ptr<MessagePlugin>)      │
│  Maps: primitive name → plugin instance                     │
│                                                              │
│  Primitive 1: "/chatter"     → BasicTopicPlugin instance #1 │
│  Primitive 2: "/battery"     → BasicTopicPlugin instance #2 │
│  Primitive 3: "/my_trigger"  → BasicServicePlugin inst #1   │
│  Primitive 4: "/navigate"    → BasicActionPlugin inst #1    │
└────────────────────────────┬────────────────────────────────┘
                             │ Call methods on plugin instances
          ┌──────────────────┼──────────────────┐
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ BasicTopicPlugin│ │BasicServicePlugin│ │BasicActionPlugin│
│  (Generic)      │ │  (Generic)       │ │  (Generic)      │
│                 │ │                  │ │                 │
│ - Uses Generic  │ │ - Uses Generic   │ │ - Uses Generic  │
│   Subscription  │ │   Client         │ │   Action Client │
│ - Runtime type  │ │ - Runtime type   │ │ - Runtime type  │
│   introspection │ │   introspection  │ │   introspection │
│ - JSON ↔ ROS    │ │ - JSON ↔ ROS     │ │ - JSON ↔ ROS    │
│   conversion    │ │   conversion     │ │   conversion    │
│ - Caching       │ │                  │ │ - Op tracking   │
└────────┬────────┘ └────────┬─────────┘ └────────┬────────┘
         │                   │                     │
         ▼                   ▼                     ▼
┌─────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│ ROS2 Topics     │ │ ROS2 Services    │ │ ROS2 Actions     │
│ (pub/sub)       │ │ (client)         │ │ (action client)  │
└─────────────────┘ └──────────────────┘ └──────────────────┘
```

### MCPServerNode Ownership

```cpp
class MCPServerNode : public rclcpp_lifecycle::LifecycleNode {
  std::unique_ptr<HTTPServer> http_server_;
  std::unique_ptr<MCPHandler> mcp_handler_;
  std::unique_ptr<MCPRouter> mcp_router_;
  std::unique_ptr<MCPPrimitiveRegistry> mcp_primitive_registry_;

  // MCPRouter internally owns:
  //   - OperationTracker operation_tracker_

  // MCPPrimitiveRegistry internally owns:
  //   - std::map<std::string, std::unique_ptr<MessagePlugin>> plugins_
};
```

**Key Architectural Principles:**

1. **Generic Plugins**: 3 plugins support ALL message types via runtime introspection
2. **Thin Router**: Router only routes, doesn't handle ROS2 operations
3. **Functor Pattern**: MCPHandler is a functor with `asFunction()` for clean HTTP integration
4. **Clear Ownership**: Each component owns its dependencies, non-owning references for loose coupling
5. **Data Conversion at Plugin Boundary**: JSON ↔ ROS conversion happens entirely within plugins

### Plugin System Flow

**At Startup (Plugin Loading via Pluginlib):**
```
1. MCPPrimitiveFactory uses pluginlib to discover available plugins
2. Scans packages with pluginlib manifests (plugins.xml):
   - robot_mcp_msg_pluginlib::BasicTopicPlugin
   - robot_mcp_msg_pluginlib::BasicServicePlugin
   - robot_mcp_msg_pluginlib::BasicActionPlugin
   - [optional] custom_plugins::MyCustomPlugin
3. Factory.createRegistry(config) instantiates plugin instances per primitive
```

**At Configuration (Factory Pattern):**
```
1. Parse YAML config:
   topics:
     - name: "battery"
       topic: "/battery_state"
       msg_type: "sensor_msgs/msg/BatteryState"
       plugin: "BasicTopicPlugin"
       description: "Robot battery status and charge level"

2. For each primitive in config:
   a. Factory loads plugin class via pluginlib
   b. Validates plugin supports msg_type (getSupportedTypes())
   c. Instantiates plugin with primitive config
   d. Calls plugin->initialize(node, primitive_config)
   e. Plugin creates ROS2 constructs (subscribers, clients, etc.)

3. Factory returns MCPPrimitiveRegistry with all plugin instances

4. MCPRouter created with reference to registry
5. MCPHandler created with reference to router
```

**At Runtime (Request Handling):**
```
Example: Claude reads battery status

1. HTTP POST /mcp → {"method": "resources/read", "params": {"uri": "battery"}}
2. HTTPServer → AuthMiddleware → JSONRPCHandler (HTTP → JSON)
3. MCPHandler receives JSON-RPC request
4. MCPHandler routes to appropriate method handler (resources/read)
5. MCPHandler calls MCPRouter.readResource("battery")
6. MCPRouter looks up "battery" in MCPPrimitiveRegistry
7. MCPRouter calls plugin->readCachedMessage()
8. BasicTopicPlugin returns cached JSON message
9. Response flows back: Plugin → Router → Handler → HTTP
```

### Data Flow Example

**User Request:** "Navigate to pose X"

```
1. Claude Desktop → HTTP POST /mcp
   {"method": "tools/call", "params": {"name": "navigate", "arguments": {"x": 1.0, "y": 2.0, ...}}}

2. HTTPServer → AuthMiddleware (validates API key) → JSONRPCHandler (parses JSON-RPC)

3. MCPHandler receives request, routes to tools/call handler

4. MCPHandler calls MCPRouter.callTool("navigate", arguments)

5. MCPRouter:
   a. Looks up "navigate" primitive in MCPPrimitiveRegistry
   b. Checks resource group conflicts (Phase 5 - deferred for now)
   c. Generates operation_id for tracking

6. MCPRouter calls plugin->sendGoal(arguments, operation_id)

7. BasicActionPlugin:
   a. Validates goal structure (JSON schema check)
   b. Converts JSON → ROS2 action goal using runtime introspection
   c. Creates/reuses action client for "/navigate_to_pose"
   d. Sends goal to action server
   e. Stores goal handle with operation_id

8. MCPRouter's OperationTracker records: operation_id → plugin instance

9. Response returns to Claude: {"operation_id": "uuid-1234", "status": "pending"}

10. Claude later queries status: tools/call {"name": "get_status", "arguments": {"operation_id": "uuid-1234"}}
    Router → OperationTracker → Plugin → Returns status + feedback

11. Claude can cancel: tools/call {"name": "cancel", "arguments": {"operation_id": "uuid-1234"}}
    Router → OperationTracker → Plugin → Cancels action
```

**Key Data Flow Principles:**

1. **JSON ↔ ROS conversion happens entirely in plugins** - Router never sees ROS messages
2. **Router is thin** - Only routing, lookup, and operation tracking
3. **Plugins are stateful** - Each instance owns ROS2 clients and cached data
4. **Type information flows through config** - msg_type specified in YAML, used for introspection

**Plugin Base Classes (`robot_mcp_msg_pluginlib/`):**

Our plugin system uses **3 generic plugins** that support ALL message types via runtime introspection.

```cpp
namespace robot_mcp_msg_pluginlib {

// ============================================================================
// Tag Dispatch for Service/Action Message Parts
// ============================================================================
namespace message_part {
  struct Request {};    // For service requests and action goals
  struct Response {};   // For service responses and action results
  struct Feedback {};   // For action feedback only
}

// ============================================================================
// Base interface - all plugins must implement
// ============================================================================
class MessagePlugin {
public:
    virtual ~MessagePlugin() = default;

    // Initialize plugin with node and primitive config
    virtual void initialize(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        const config::PrimitiveConfig& config) = 0;

    // Returns empty vector = supports ALL types (generic)
    // Returns non-empty = only supports specific types (custom plugins)
    virtual std::vector<std::string> getSupportedTypes() const = 0;

    // Get description for MCP tools/resources list
    virtual std::string getDescription() const = 0;
};

// ============================================================================
// BasicTopicPlugin - Generic plugin for ALL topic types
// ============================================================================
class BasicTopicPlugin : public MessagePlugin {
public:
    // Initialization
    void initialize(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        const config::TopicConfig& config) override;

    // Generic support - handles ALL message types
    std::vector<std::string> getSupportedTypes() const override {
        return {};  // Empty = ALL types supported
    }

    std::string getDescription() const override {
        return description_;  // From config
    }

    // Publish a message (tools/call for topics with publish=true)
    virtual void publishMessage(const nlohmann::json& msg);

    // Read cached message (resources/read)
    virtual nlohmann::json readCachedMessage() const;

protected:
    // Uses rclcpp::GenericSubscription for runtime type handling
    std::shared_ptr<rclcpp::GenericSubscription> subscription_;

    // Uses rclcpp::GenericPublisher for runtime type handling
    std::shared_ptr<rclcpp::GenericPublisher> publisher_;

    // Cached latest message(s) as JSON
    std::deque<nlohmann::json> message_cache_;

    // Message type info for introspection
    std::string msg_type_;
    const rosidl_message_type_support_t* type_support_;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string description_;
};

// ============================================================================
// BasicServicePlugin - Generic plugin for ALL service types
// ============================================================================
class BasicServicePlugin : public MessagePlugin {
public:
    // Initialization
    void initialize(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        const config::ServiceConfig& config) override;

    // Generic support - handles ALL service types
    std::vector<std::string> getSupportedTypes() const override {
        return {};  // Empty = ALL types supported
    }

    std::string getDescription() const override {
        return description_;
    }

    // Call service (tools/call)
    virtual nlohmann::json callService(const nlohmann::json& request);

    // Tag dispatch for request/response conversion
    virtual nlohmann::json toJson(const void* msg_ptr, message_part::Request) const;
    virtual nlohmann::json toJson(const void* msg_ptr, message_part::Response) const;
    virtual void fromJson(const nlohmann::json& j, void* msg_ptr, message_part::Request) const;
    virtual void fromJson(const nlohmann::json& j, void* msg_ptr, message_part::Response) const;

protected:
    // Uses rclcpp::GenericClient for runtime type handling
    std::shared_ptr<rclcpp::GenericClient> client_;

    // Service type info for introspection
    std::string srv_type_;
    const rosidl_service_type_support_t* type_support_;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string description_;
    int timeout_ms_;
};

// ============================================================================
// BasicActionPlugin - Generic plugin for ALL action types
// ============================================================================
class BasicActionPlugin : public MessagePlugin {
public:
    // Initialization
    void initialize(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        const config::ActionConfig& config) override;

    // Generic support - handles ALL action types
    std::vector<std::string> getSupportedTypes() const override {
        return {};  // Empty = ALL types supported
    }

    std::string getDescription() const override {
        return description_;
    }

    // Send goal (tools/call)
    virtual nlohmann::json sendGoal(const nlohmann::json& goal, std::string& operation_id);

    // Get status (tools/call with operation_id)
    virtual nlohmann::json getStatus(const std::string& operation_id) const;

    // Cancel action (tools/call)
    virtual void cancel(const std::string& operation_id);

    // Tag dispatch for goal/result/feedback conversion
    virtual nlohmann::json toJson(const void* msg_ptr, message_part::Request) const;   // Goal
    virtual nlohmann::json toJson(const void* msg_ptr, message_part::Response) const;  // Result
    virtual nlohmann::json toJson(const void* msg_ptr, message_part::Feedback) const;  // Feedback
    virtual void fromJson(const nlohmann::json& j, void* msg_ptr, message_part::Request) const;  // Goal

protected:
    // Uses rclcpp_action::Client with generic types (Phase 4)
    // For now: std::shared_ptr<void> for type-erased action client
    std::shared_ptr<void> action_client_;

    // Track active goals by operation_id
    std::map<std::string, std::shared_ptr<void>> active_goals_;

    // Action type info for introspection
    std::string action_type_;
    const rosidl_action_type_support_t* type_support_;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string description_;
    bool cancellable_;
};

}  // namespace robot_mcp_msg_pluginlib
```

**Key Architectural Insight: Generic Plugins with Runtime Introspection**

The plugin architecture uses **3 generic plugins** that support ALL message types via runtime introspection:

1. **Generic by Default**: `BasicTopicPlugin`, `BasicServicePlugin`, `BasicActionPlugin`
   - Support ANY ROS2 message/service/action type
   - Use `rclcpp::GenericSubscription`, `rclcpp::GenericClient`, etc.
   - JSON ↔ ROS conversion via `rosidl_typesupport_introspection_cpp`

2. **Runtime Type Handling**: No compile-time knowledge of message types needed
   - Type information comes from config (`msg_type: "sensor_msgs/msg/BatteryState"`)
   - Plugins use type support introspection to walk message structure
   - Automatic JSON conversion for all field types (primitives, arrays, nested)

3. **Tag Dispatch for Message Parts**: Services and actions have multiple message parts
   - `toJson(void*, message_part::Request)` for service requests/action goals
   - `toJson(void*, message_part::Response)` for service responses/action results
   - `toJson(void*, message_part::Feedback)` for action feedback
   - Compile-time overload resolution with runtime polymorphism

4. **Extensibility**: Users can create custom plugins for specialized behavior
   - Custom plugins declare `getSupportedTypes()` to limit scope
   - Example: `ImagePlugin` might only support `sensor_msgs/msg/Image`
   - Factory validates type compatibility at startup

5. **Configuration-Driven**: Each primitive specifies its plugin
   - Primitives using same plugin type → separate instances
   - Each instance owns its own ROS2 clients and state
   - Description field in config used for MCP tools/resources list

**Example Configuration:**

```yaml
topics:
  - name: "battery"
    topic: "/battery_state"
    msg_type: "sensor_msgs/msg/BatteryState"
    plugin: "BasicTopicPlugin"
    description: "Robot battery status including voltage, current, and charge percentage"
    subscribe: true
    publish: false

  - name: "cmd_vel"
    topic: "/cmd_vel"
    msg_type: "geometry_msgs/msg/Twist"
    plugin: "BasicTopicPlugin"
    description: "Velocity commands for robot base motion (linear and angular)"
    subscribe: false
    publish: true

services:
  - name: "reset"
    service: "/reset"
    srv_type: "std_srvs/srv/Trigger"
    plugin: "BasicServicePlugin"
    description: "Reset robot state to initial configuration"
    timeout_ms: 3000

actions:
  - name: "navigate"
    action: "/navigate_to_pose"
    action_type: "nav2_msgs/action/NavigateToPose"
    plugin: "BasicActionPlugin"
    description: "Navigate robot to a target pose in the map frame"
    timeout_ms: 60000
    cancellable: true
```

**Plugin Manifest (plugins.xml):**
```xml
<library path="librobot_mcp_msg_pluginlib">
  <class type="robot_mcp_msg_pluginlib::BasicTopicPlugin"
         base_class_type="robot_mcp_msg_pluginlib::MessagePlugin">
    <description>Generic plugin for all ROS2 topic types</description>
  </class>

  <class type="robot_mcp_msg_pluginlib::BasicServicePlugin"
         base_class_type="robot_mcp_msg_pluginlib::MessagePlugin">
    <description>Generic plugin for all ROS2 service types</description>
  </class>

  <class type="robot_mcp_msg_pluginlib::BasicActionPlugin"
         base_class_type="robot_mcp_msg_pluginlib::MessagePlugin">
    <description>Generic plugin for all ROS2 action types</description>
  </class>
</library>
```

**Runtime Introspection Approach:**

```cpp
// Simplified example of JSON ↔ ROS conversion using introspection
nlohmann::json BasicTopicPlugin::messageToJson(const void* ros_msg) {
    auto members = rosidl_typesupport_introspection_cpp::get_message_members(type_support_);
    nlohmann::json result;

    for (size_t i = 0; i < members->member_count_; ++i) {
        const auto& member = members->members_[i];
        const void* field_ptr = static_cast<const uint8_t*>(ros_msg) + member.offset_;

        switch (member.type_id_) {
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
                result[member.name_] = *static_cast<const float*>(field_ptr);
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
                result[member.name_] = *static_cast<const std::string*>(field_ptr);
                break;
            // ... handle all types, arrays, nested messages recursively
        }
    }
    return result;
}
```

## Implementation Plan (Phase 3+4)

### 1. Plugin Base Classes ✅ (Phase 1-2 Complete)
- **Already Implemented**: `config_types.hpp`, `config_parser.hpp`, HTTPServer, AuthMiddleware, JSONRPCHandler
- **Phase 3+4**: Create `robot_mcp_msg_pluginlib` package
  - `message_plugin.hpp`: Base interface with `getSupportedTypes()`, `getDescription()`
  - `basic_topic_plugin.hpp`: Generic topic plugin using `rclcpp::GenericSubscription`
  - `basic_service_plugin.hpp`: Generic service plugin with tag dispatch
  - `basic_action_plugin.hpp`: Generic action plugin with operation tracking
  - `message_conversion.hpp`: Introspection-based JSON ↔ ROS utilities

### 2. Plugin Infrastructure
- **MCPPrimitiveLoader**: Wraps pluginlib for loading plugin classes
- **MCPPrimitiveFactory**: Creates plugin instances and returns registry
  - Validates `getSupportedTypes()` compatibility at startup
  - Instantiates plugins with primitive config (name, description, etc.)
- **MCPPrimitiveRegistry**: Owns plugin instances via `std::map<string, unique_ptr<MessagePlugin>>`
  - Provides lookup by primitive name

### 3. MCP Handler
- **MCPHandler**: Single functor class handling all 6 MCP methods
  - `initialize()`: Return server info
  - `ping()`: Health check
  - `tools/list`: List primitives with descriptions
  - `tools/call`: Dispatch to router
  - `resources/list`: List topics with subscribe=true
  - `resources/read`: Dispatch to router
  - `asFunction()`: Returns `std::function<json(json)>` for HTTPServer

### 4. Router & Operation Tracker
- **MCPRouter**: Routes requests to correct plugin in registry
  - Non-owning reference to `MCPPrimitiveRegistry`
  - Internally owns `OperationTracker`
  - Methods: `callTool()`, `readResource()`, `getOperationStatus()`, `cancelOperation()`
- **OperationTracker**: Tracks active action operations
  - Maps `operation_id` → plugin instance reference
  - Used for status queries and cancellation

### 5. Integration in MCPServerNode
- **on_configure()**:
  - Create `MCPPrimitiveFactory`
  - `mcp_primitive_registry_ = factory.createRegistry(shared_from_this(), config_)`
  - `mcp_router_ = make_unique<MCPRouter>(*mcp_primitive_registry_)`
  - `mcp_handler_ = make_unique<MCPHandler>(config_, *mcp_router_)`
- **on_activate()**:
  - `http_server_->start(config_.server, mcp_handler_->asFunction())`
- **Remove**: `handleMCPRequest()` method (MCPHandler owns data path)

### 6. Testing Strategy
- **Catch2 Unit Tests**:
  - Test each plugin class with mock message types
  - Test introspection-based JSON conversion
  - Test MCPRouter routing logic
  - Test OperationTracker
  - Test MCPHandler method dispatch
- **Launch Tests**:
  - End-to-end test with real ROS2 topics/services/actions
  - Test all 6 MCP methods
  - Test operation tracking for long-running actions

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

### Message Type System: Generic Plugins with Runtime Introspection

**Decision:** 3 generic plugins support ALL message types via runtime introspection using `rosidl_typesupport_introspection_cpp`

**Key Innovation:** No type-specific plugins needed. Plugins use runtime type information from config to handle any ROS2 message/service/action.

**Advantages:**
- **Zero plugin proliferation** - 3 plugins handle all message types
- **No maintenance burden** - Don't need to create/maintain hundreds of type-specific plugins
- **Automatic support for new types** - Custom messages work immediately without new plugins
- **Configuration-driven** - Type information comes from YAML, not compilation
- **Extensible** - Users can still create custom plugins for specialized behavior
- **Follows ROS2 patterns** - Uses same introspection APIs as ros2bag, ros2 topic echo, etc.

**Why Generic Over Type-Specific:**

| Aspect | Type-Specific Plugins | Generic Plugins |
|--------|----------------------|-----------------|
| **Plugin count** | Hundreds (one per type) | 3 (topic/service/action) |
| **Maintenance** | High (update for each type) | Low (one implementation) |
| **New message types** | Requires new plugin | Works immediately |
| **Testing** | Test each plugin | Test once |
| **Compile dependencies** | All message packages | None (runtime only) |
| **Custom behavior** | Easy (per-type) | Possible (custom plugins) |

**Implementation Approach:**

1. **Generic ROS2 Clients:**
   - `rclcpp::GenericSubscription` for topics
   - `rclcpp::GenericClient` for services
   - Generic action client (Phase 4 work needed)

2. **Runtime Introspection:**
   - Use `rosidl_typesupport_introspection_cpp::get_message_members()`
   - Walk message structure recursively
   - Convert each field based on `type_id_` (float, string, nested message, array, etc.)

3. **Tag Dispatch:**
   - Services have Request/Response parts
   - Actions have Goal/Result/Feedback parts
   - Use empty tag structs for compile-time overload resolution
   - Virtual functions provide runtime polymorphism for router

4. **Type Compatibility Validation:**
   - Plugins declare `getSupportedTypes()`: empty = ALL, non-empty = specific types
   - Factory validates compatibility at startup
   - Allows custom plugins to restrict scope

**Configuration Example:**
```yaml
# All use the same BasicTopicPlugin class
topics:
  - name: "battery"
    topic: "/battery_state"
    msg_type: "sensor_msgs/msg/BatteryState"
    plugin: "BasicTopicPlugin"
    description: "Robot battery status"

  - name: "odom"
    topic: "/odom"
    msg_type: "nav_msgs/msg/Odometry"
    plugin: "BasicTopicPlugin"
    description: "Robot odometry"

  - name: "custom_msg"
    topic: "/my_custom_topic"
    msg_type: "my_pkg/msg/MyCustomMessage"
    plugin: "BasicTopicPlugin"  # Works without creating new plugin!
    description: "Custom message type"
```

**When to Create Custom Plugins:**
- **Specialized validation**: Image size limits, velocity bounds, etc.
- **Performance optimization**: Zero-copy for large messages
- **Complex transformations**: Coordinate frame conversions, compression
- **Domain-specific logic**: Robot-specific safety checks

**Custom Plugin Example:**
```cpp
class ImagePlugin : public BasicTopicPlugin {
    std::vector<std::string> getSupportedTypes() const override {
        return {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"};
    }

    // Override to add compression before sending over network
    nlohmann::json readCachedMessage() const override {
        auto full_json = BasicTopicPlugin::readCachedMessage();
        return compressImageData(full_json);  // Custom compression
    }
};
```

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
