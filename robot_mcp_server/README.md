# robot_mcp_server

MCP (Model Context Protocol) server for ROS2 robots - enables AI assistants to control robots via HTTP.

## Quick Start

```bash
# Install dependencies and build
rosdep install --from-paths . --ignore-src -y
colcon build --packages-select robot_mcp_server

# Run (use robot_mcp_bringup for production)
ros2 run robot_mcp_server robot_mcp_server_node --ros-args --params-file config.yaml

# Test
curl -X POST http://localhost:8080/mcp/v1 \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc": "2.0", "method": "test", "id": 1}'
```

## Configuration

See `robot_mcp_bringup/config/minimal.yaml` for example configuration. Configuration follows Nav2 pattern:

```yaml
mcp_http_server:
  ros__parameters:
    server:
      host: "0.0.0.0"
      port: 8080
      api_key: ""  # Optional
      enable_https: false  # Requires ssl_cert_path and ssl_key_path

    topics: ["topic_name"]  # List of topic names to expose
    topic_name:
      topic: "/ros/topic"
      msg_type: "package/msg/Type"
      plugin: "plugin_class_name"
```

## Architecture

- **Config System**: Parses ROS2 parameters
- **HTTP Server**: HTTP/HTTPS with JSON-RPC 2.0
- **Router** (Phase 3): Dispatch MCP requests
- **Plugin System** (Phase 4): Dynamic message/service/action handlers

Lifecycle node compatible with `nav2_lifecycle_manager`. See `DEVELOPING.md` for class reference.
