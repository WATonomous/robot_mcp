## Project Overview
Building `robot_mcp` - a Model Context Protocol (MCP) server for ROS2 robots that enables AI assistants (like Claude) to control robots through natural language via HTTP.

---

## Core Architecture

### What It Is
- **Configuration-driven** ROS2 HTTP server that exposes actions, services, and topics as MCP tools/resources
- **Pure C++** implementation using `rclcpp_lifecycle`
- **HTTP-based** (not stdio) for network accessibility and multi-client support
- User configures YAML file specifying which ROS2 interfaces to expose - server handles everything else

### Key Components
```
User's Computer (Claude) ←→ HTTP/JSON-RPC ←→ robot_mcp Server (on robot) ←→ ROS2 Stack
```

---

## Technical Decisions

### Dependencies
**Non-ROS (only 2 required):**
- `cpp-httplib` (header-only HTTP server)
- `nlohmann/json` (header-only JSON parsing)

### Transport Choice: HTTP (not stdio)
**Why HTTP over stdio:**
- Network accessible (control robot from anywhere)
- Multiple concurrent clients
- Standard debugging tools (curl, Postman)
- Better for production deployment
- No SSH tunneling needed

### Message Type System: Macro-Based Registration
Using macros for registering ROS2 message types to reduce boilerplate:

```cpp
REGISTER_MESSAGE_TYPE(sensor_msgs::msg::BatteryState, "sensor_msgs/msg/BatteryState")
IMPLEMENT_MESSAGE_JSON(sensor_msgs::msg::BatteryState) {
    return {
        {"voltage", msg.voltage},
        {"percentage", msg.percentage},
        // ...
    };
}
```

**Benefits:**
- 3-5 lines per message type
- Auto-registration at startup
- Type-safe (compile-time)
- Extensible (users can add custom types)

**Custom Message Registration**
- users should have the option to register their own messages
- this should be some sort of plugin architecture which lets robot_mcp autodiscover available registered message types (both the defaults we provide, and registered ones the user makes)

---

## Configuration System

### Full config with smart defaults
Users specify only what they want to expose; everything else uses sensible defaults.

**Example minimal config:**
```yaml
mcp_http_server:
  ros__parameters:
    actions:
      - name: "navigate_to_pose"
        action_type: "nav2_msgs/action/NavigateToPose"
    
    topics:
      - name: "battery_state"
        topic: "/battery_state"
        msg_type: "sensor_msgs/msg/BatteryState"
```

**Full config includes:**
- Server settings (host, port, threads, timeouts)
- Authentication (API key, basic, none)
- Resource groups (conflict management)
- Actions, services, topics (what to expose)
- Built-in tools (system info, active operations)
- Logging and monitoring
- Advanced options (rate limiting, retry logic)

---

## Resource Conflict Management

**Problem:** Claude can send conflicting parallel requests (e.g., two navigation goals simultaneously)

**Solution:** Resource group system
```yaml
resource_groups:
  base_motion:
    max_concurrent: 1
    interruptible: true
    priority: 10

actions:
  - name: "navigate_to_pose"
    resource_groups: ["base_motion"]
    conflict_resolution: "error"  # or "cancel", "queue"
```

Server enforces mutual exclusion - only one action per resource group at a time.

---

## MCP Protocol Implementation

### Request Types Claude Can Make:
1. `initialize` - Handshake
2. `tools/list` - What actions are available?
3. `tools/call` - Execute an action
4. `resources/list` - What data can I read?
5. `resources/read` - Read topic data
6. `ping` - Keep-alive

### How Data Flows Back to Claude:

**Services:** Synchronous (wait for response)

**Short actions (<5s):** Block and return result

**Long actions (>5s):** Return immediately + provide status tools
- `navigate_to_pose` returns "Started"
- Claude can call `get_navigation_status` to check progress
- Claude can call `cancel_navigation` to stop

**Topics:** Exposed as resources, Claude reads when needed, can also publish a message once or at a set rate

**Optional:** Server-Sent Events (SSE) for push updates

---

## Key Design Principles

1. **Configuration over code** - Users configure YAML, not C++
2. **Safe by default** - Explicit about what's exposed, conflict detection
3. **Reusable** - Same binary works for any robot with different config
4. **Production-ready** - Lifecycle node, auth, monitoring, error handling
5. **Performance** - Native C++, zero-copy when possible, thread-safe
6. **Standards-compliant** - Follows ROS2 and MCP specifications

---

## Why C++ over Python FastMCP

**Performance:**
- 10-20x lower latency (0.5ms vs 5ms overhead)
- 10x+ better throughput (50k+ req/s vs 1-2k req/s)
- No GIL - true parallel request handling
- 10x lower memory (10MB vs 100MB)

**Production Benefits:**
- Native ROS2 lifecycle integration
- Single binary deployment (no Python runtime)
- Compile-time type safety
- Better for embedded systems

**Tradeoff:** More initial development time, but better for production robotics.

---

## Package Naming

**Repository:** `robot_mcp`
contains a series of packages that all work with each other to do stuff.

Follows ROS2 conventions (like `robot_localization`, `robot_state_publisher`).

---

## Usage Examples

```bash
# Install dependencies (through rosdep)
sudo apt-get install nlohmann-json3-dev
curl -o /tmp/httplib.h https://raw.githubusercontent.com/yhirose/cpp-httplib/master/httplib.h
sudo mv /tmp/httplib.h /usr/local/include/

# Build
colcon build --packages-select robot_mcp

# Run with config
ros2 launch robot_mcp robot_mcp.launch.py config:=/path/to/config.yaml

# Run with preset
ros2 launch robot_mcp robot_mcp.launch.py preset:=turtlebot3

# Check lifecycle state
ros2 lifecycle get /robot_mcp_server
```

**Claude Desktop config:**
```json
{
  "mcpServers": {
    "my-robot": {
      "url": "http://localhost:8080/mcp"
    }
  }
}
```

---

## Open Questions / TODOs

2. How to handle action feedback streaming? worth some time to think about (and I need to know how MCP works)
3. Should we include common message types (sensor_msgs, geometry_msgs, etc.) or make users register them? (Ship with 20-30 common types)
4. Authentication default: enabled or disabled? (Disabled for dev, document how to enable)
5. Should we build Docker image for easy deployment? (Yes, it can and should be VERY light, and easily integratable into other dockerfiles)

---

## Testing Strategy

1. **Unit tests** - Configuration parsing, conflict detection, message conversion
2. **Integration tests** - Full MCP protocol flow with mock ROS2 nodes
3. **End-to-end tests** - Test with actual Claude Desktop + simulated robot (Gazebo)
4. **Performance benchmarks** - Measure latency, throughput, memory under load

---

## Documentation Needed

1. **User Guide** - How to configure and use
2. **Configuration Reference** - All config options explained
3. **Architecture Document** - System design and flow
4. **Developing Guide** - How to add custom message types
5. **Examples** - TurtleBot3, Nav2, custom robot setups
