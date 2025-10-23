# Phase 3+4 Implementation Status

**Date:** 2025-10-23
**Status:** In Progress - Building Plugin Infrastructure
**Next:** Switch to Jazzy for better generic client support

---

## What We've Completed

### 1. ✅ Config Updates
- **File:** `robot_mcp_server/include/robot_mcp_server/mcp_config/config_types.hpp`
- **Changes:** Added `description` field to `TopicConfig`, `ServiceConfig`, `ActionConfig`
- **Purpose:** Descriptions will flow through to MCP tools/resources list for Claude to reason about

### 2. ✅ Base Plugin Interface
- **File:** `robot_mcp_msg_pluginlib/include/robot_mcp_msg_pluginlib/message_plugin.hpp`
- **Changes:** Completely rewritten for generic plugin design
- **Key Methods:**
  - `initialize(node, primitive_name, description)` - Setup with lifecycle node
  - `getSupportedTypes()` - Returns empty vector for generic plugins, non-empty for custom
  - `getDescription()` - Returns description from config
  - `getPrimitiveName()` - Returns primitive name
- **Tag Dispatch:** Added `message_part::Request`, `message_part::Response`, `message_part::Feedback` for service/action parts

### 3. ✅ New Package: robot_mcp_generic_plugins
Created complete package structure:

```
robot_mcp_generic_plugins/
├── CMakeLists.txt          ✅ Modern CMake with target_link_libraries
├── package.xml             ✅ Dependencies on robot_mcp_msg_pluginlib + introspection
├── plugins.xml             ✅ Pluginlib manifest for 3 generic plugins
├── include/robot_mcp_generic_plugins/
│   ├── basic_topic_plugin.hpp          ✅ Generic topic plugin (all msg types)
│   ├── basic_service_plugin.hpp        ✅ Generic service plugin (stub - needs work)
│   ├── basic_action_plugin.hpp         ✅ Generic action plugin (stub - needs work)
│   └── message_conversion.hpp          ✅ Introspection utilities (stub)
└── src/
    ├── basic_topic_plugin.cpp          ✅ Stub implementation
    ├── basic_service_plugin.cpp        ✅ Stub implementation
    ├── basic_action_plugin.cpp         ✅ Stub implementation with UUID generation
    └── message_conversion.cpp          ✅ Stub implementation
```

### 4. ✅ Plugin Implementations (Stubs)

**BasicTopicPlugin:**
- Uses `rclcpp::GenericSubscription` and `rclcpp::GenericPublisher` ✅
- Methods: `configureTopic()`, `publishMessage()`, `readCachedMessage()`
- Caching: Latest 10 messages in JSON format
- Status: Compiles but needs introspection conversion implementation

**BasicServicePlugin:**
- Placeholder for generic service client (doesn't exist in Humble)
- Methods: `configureService()`, `callService()`
- Status: ⚠️ Needs GenericClient (may exist in Jazzy)

**BasicActionPlugin:**
- Placeholder for generic action client
- Methods: `configureAction()`, `sendGoal()`, `getStatus()`, `cancel()`
- UUID generation for operation IDs ✅
- Status: ⚠️ Needs generic action client

**message_conversion.hpp/cpp:**
- Stub functions for introspection-based conversion:
  - `rosMessageToJson()` - TODO
  - `jsonToRosMessage()` - TODO
  - `getMessageTypeSupport()` - TODO
  - `getServiceTypeSupport()` - TODO
  - `getActionTypeSupport()` - TODO

---

## Current Build Status

**Last Build Attempt:** Failed on Humble
**Issue:** `rclcpp/generic_client.hpp` does not exist in Humble
**Packages Built:**
- ✅ `robot_mcp_msg_pluginlib` - Rebuilt successfully with new interface
- ❌ `robot_mcp_generic_plugins` - Failed due to missing GenericClient

**Warnings to Fix:**
- Unused parameter warnings (use `(void)param` to suppress)
- Parentheses suggestion in UUID generation

---

## What's Next (After Jazzy Switch)

### Immediate Tasks

1. **Fix Compilation on Jazzy**
   - Check if `GenericClient` exists in Jazzy
   - If not, implement manual service client creation using type support
   - Fix all compilation warnings

2. **Implement Introspection Conversion** (Critical Path)
   - File: `robot_mcp_generic_plugins/src/message_conversion.cpp`
   - Implement `rosMessageToJson()` using `rosidl_typesupport_introspection_cpp`
   - Implement `jsonToRosMessage()`
   - Implement `getMessageTypeSupport()`, `getServiceTypeSupport()`, `getActionTypeSupport()`
   - Walk message members recursively
   - Handle all field types: primitives, strings, arrays, nested messages
   - Reference: `rosbag2_cpp` or `ros2 topic echo` for examples

3. **Complete Plugin Implementations**
   - Update `basic_topic_plugin.cpp` to use conversion utilities
   - Update `basic_service_plugin.cpp` once GenericClient is available
   - Update `basic_action_plugin.cpp` once generic action client is available

4. **Build and Test Plugins Package**
   ```bash
   colcon build --packages-select robot_mcp_generic_plugins
   ```

### Phase 3+4 Remaining Work

5. **Create MCPPrimitiveFactory**
   - File: `robot_mcp_server/include/robot_mcp_server/mcp_primitive/mcp_primitive_factory.hpp`
   - Uses `pluginlib::ClassLoader<MessagePlugin>` directly (no wrapper)
   - Validates `getSupportedTypes()` compatibility at startup
   - Returns `MCPPrimitiveRegistry` populated with plugin instances

6. **Create MCPPrimitiveRegistry**
   - File: `robot_mcp_server/include/robot_mcp_server/mcp_primitive/mcp_primitive_registry.hpp`
   - Owns plugin instances via `std::map<std::string, std::unique_ptr<MessagePlugin>>`
   - Provides lookup by primitive name
   - Methods: `getPlugin(name)`, `getAllPrimitives()`, `hasPrimitive(name)`

7. **Create OperationTracker**
   - File: `robot_mcp_server/include/robot_mcp_server/mcp_router/operation_tracker.hpp`
   - Maps `operation_id` → plugin instance reference
   - Used for action status queries and cancellation
   - Thread-safe for concurrent access

8. **Create MCPRouter**
   - File: `robot_mcp_server/include/robot_mcp_server/mcp_router/mcp_router.hpp`
   - Routes requests to correct plugin in registry
   - Non-owning reference to `MCPPrimitiveRegistry`
   - Internally owns `OperationTracker`
   - Methods:
     - `callTool(primitive_name, args)` - Dispatch to plugin
     - `readResource(primitive_name)` - Read cached topic data
     - `getOperationStatus(operation_id)` - Query action status
     - `cancelOperation(operation_id)` - Cancel action

9. **Create MCPHandler**
   - File: `robot_mcp_server/include/robot_mcp_server/mcp_handler/mcp_handler.hpp`
   - Single functor class handling all 6 MCP methods:
     - `initialize()` - Return server info
     - `ping()` - Health check
     - `tools/list` - List all primitives with descriptions
     - `tools/call` - Dispatch to router
     - `resources/list` - List topics with subscribe=true
     - `resources/read` - Dispatch to router
   - `asFunction()` returns `std::function<json(json)>` for HTTPServer
   - Non-owning reference to `MCPRouter`

10. **Update MCPServerNode Integration**
    - File: `robot_mcp_server/src/robot_mcp_server_node.cpp`
    - **on_configure():**
      ```cpp
      MCPPrimitiveFactory factory;
      mcp_primitive_registry_ = factory.createRegistry(shared_from_this(), config_);
      mcp_router_ = make_unique<MCPRouter>(*mcp_primitive_registry_);
      mcp_handler_ = make_unique<MCPHandler>(config_, *mcp_router_);
      ```
    - **on_activate():**
      ```cpp
      http_server_->start(config_.server, mcp_handler_->asFunction());
      ```
    - **REMOVE:** `handleMCPRequest()` method (MCPHandler owns data path)

11. **Write End-to-End Launch Test**
    - Test all 6 MCP methods with real ROS2 topics/services/actions
    - Test operation tracking for long-running actions
    - Add to `robot_mcp_test` package

12. **Verify All Tests Pass**
    ```bash
    colcon test
    pre-commit run --all-files
    ```

---

## Architecture Decisions Summary

### Generic Plugin Design
- **3 plugins total:** BasicTopicPlugin, BasicServicePlugin, BasicActionPlugin
- **Runtime introspection:** No compile-time knowledge of message types
- **Configuration-driven:** Type info comes from YAML config
- **Extensibility:** Users can create custom plugins for specialized behavior

### Ownership Structure
```
MCPServerNode owns:
├── http_server_ (HTTPServer)
├── mcp_handler_ (MCPHandler - functor)
├── mcp_router_ (MCPRouter - owns OperationTracker)
└── mcp_primitive_registry_ (MCPPrimitiveRegistry - owns all plugin instances)
```

### Factory Pattern
- Factory creates registry from config
- Factory validates plugin-type compatibility via `getSupportedTypes()`
- Factory is transient (local variable in on_configure)
- Registry is owned by MCPServerNode

### Data Flow
```
HTTP Request → HTTPServer → MCPHandler → MCPRouter → MCPPrimitiveRegistry → Plugin
                                                                               ↓
JSON ← HTTP Response ← MCPHandler ← MCPRouter ← JSON ← Plugin ← ROS2 Message
```

---

## Known Issues & Limitations

1. **Humble ROS2:** No GenericClient for services/actions
   - **Solution:** Switch to Jazzy or implement manual client creation

2. **Introspection Not Implemented:** Core conversion logic TODO
   - **Impact:** Plugins are stubs, can't actually convert messages
   - **Priority:** Critical path for Phase 3+4

3. **Generic Action Client:** May not exist even in Jazzy
   - **Fallback:** Manual action client creation using type support
   - **Complexity:** High - action client setup is non-trivial

4. **Type Support Lookup:** Need runtime lookup mechanism
   - **Challenge:** Parsing type strings like "sensor_msgs/msg/BatteryState"
   - **Approach:** Use rosidl_typesupport APIs or dlopen() for .so files

---

## Testing Strategy

### Unit Tests (Catch2 in robot_mcp_test)
- Test introspection conversion with known message types
- Test plugin initialization and configuration
- Test MCPRouter routing logic
- Test OperationTracker
- Test MCPHandler method dispatch

### Launch Tests (launch_testing)
- End-to-end test with real ROS2 topics/services/actions
- Test all 6 MCP methods
- Test operation tracking
- Test error handling

---

## References

- **ARCHITECTURE.md:** Complete design doc (updated 2025-10-23)
- **Introspection Examples:**
  - `ros2 topic echo` implementation
  - `rosbag2_cpp` serialization
  - `rosidl_typesupport_introspection_cpp` examples
- **Generic Subscription:** `/opt/ros/jazzy/include/rclcpp/rclcpp/generic_subscription.hpp`

---

## Command to Resume

After switching to Jazzy:

```bash
# Source Jazzy workspace
source /opt/ros/jazzy/setup.bash

# Rebuild all packages
colcon build --packages-select robot_mcp_msg_pluginlib robot_mcp_generic_plugins

# Check if GenericClient exists
find /opt/ros/jazzy/include -name "*generic*"

# Continue with introspection implementation
# Edit: robot_mcp_generic_plugins/src/message_conversion.cpp
```

---

## Questions to Resolve

1. **Jazzy GenericClient:** Does it exist? What's the header?
2. **Type Support Lookup:** Best approach for runtime type lookup?
3. **Action Client:** Generic or manual creation?
4. **Resource Groups:** Defer to Phase 5 or stub in Phase 3+4?

---

**Next Session:** Start by checking Jazzy's generic client support, then implement introspection conversion.
