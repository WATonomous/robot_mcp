# robot_mcp_bringup

Launch files and configurations for `robot_mcp_server` with lifecycle management.

## Usage

```bash
# Launch with default config
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml

# Custom config
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml params_file:=/path/to/config.yaml

# Without autostart (manual lifecycle control)
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml autostart:=false
```

## Configuration

See `config/minimal.yaml` for example. Configuration structure:

```yaml
mcp_http_server:
  ros__parameters:
    server:
      host: "0.0.0.0"
      port: 8080
      api_key: ""  # Optional, recommended for production
      enable_https: false  # Requires cert/key paths

    topics: ["topic_name"]
    topic_name:
      topic: "/ros/topic"
      msg_type: "package/msg/Type"
      plugin: "plugin_class_name"
```

## Testing

```bash
# Check status
ros2 lifecycle get /mcp_http_server

# Test endpoint
curl -X POST http://localhost:8080/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc": "2.0", "method": "test", "id": 1}'
```

## Launch Arguments

- `params_file`: Config YAML path (default: `config/minimal.yaml`)
- `autostart`: Auto configure/activate (default: `true`)
- `namespace`: Node namespace (default: `""`)

Uses `nav2_lifecycle_manager` for lifecycle management with bond monitoring.
