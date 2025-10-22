# robot_mcp_bringup Development Guide

## Launch File

`launch/robot_mcp.launch.yaml`: YAML launch format for declarative configuration

- Launches `robot_mcp_server_node` with parameters from file
- Launches `nav2_lifecycle_manager` to manage node lifecycle with bond monitoring
- Arguments: `params_file`, `autostart`, `namespace`

## Configuration

Uses Nav2 nested parameter pattern (see `config/minimal.yaml` for example):

```yaml
mcp_http_server:
  ros__parameters:
    topics: ["name1", "name2"]  # List of names
    name1:                       # Nested params per name
      field: value
```

ConfigParser in `robot_mcp_server` validates configuration at runtime.

## Testing

```bash
# Test launch
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml
ros2 lifecycle get /mcp_http_server  # Should show "active"

# Test without autostart
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml autostart:=false
ros2 lifecycle set /mcp_http_server configure
ros2 lifecycle set /mcp_http_server activate

# Multi-robot (use namespaces and different ports in config)
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml \
  namespace:=/robot1 params_file:=/path/to/robot1.yaml
```
