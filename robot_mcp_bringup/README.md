# robot_mcp_bringup

Launch files and configurations for bringing up the robot_mcp_server with lifecycle management.

## Overview

This package provides launch files and configuration templates for deploying `robot_mcp_server` on your robot. It includes integration with `nav2_lifecycle_manager` for robust lifecycle management.

## Quick Start

### Launch with default configuration

```bash
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml
```

### Launch with custom configuration

```bash
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml \
  params_file:=/path/to/your/config.yaml
```

### Launch without autostart

```bash
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml autostart:=false

# Manually trigger lifecycle transitions
ros2 lifecycle set /mcp_http_server configure
ros2 lifecycle set /mcp_http_server activate
```

## Configuration Files

### `config/minimal.yaml`

Minimal working configuration with:
- HTTP server on localhost:8080
- No authentication (development only)
- Example topic/service/action configuration structure
- Bond support enabled for lifecycle management

### Creating Your Own Config

```yaml
mcp_http_server:
  ros__parameters:
    # Server configuration
    server:
      host: "0.0.0.0"
      port: 8080
      api_key: "your-secret-key"  # Recommended for production
      enable_https: false  # Set true for production with certs

      # Lifecycle manager integration
      bond_enabled: true
      bond_timeout: 4.0
      bond_heartbeat_period: 0.1

    # Define topics to expose
    topics: ["robot_pose", "robot_status"]

    robot_pose:
      topic: "/robot/pose"
      msg_type: "geometry_msgs/msg/PoseStamped"
      plugin: "robot_mcp_msg_pluginlib::PoseStampedPlugin"
      subscribe: true
      publish: false

    robot_status:
      topic: "/robot/status"
      msg_type: "std_msgs/msg/String"
      plugin: "robot_mcp_msg_pluginlib::StringPlugin"
      subscribe: true
      publish: false

    # Define services to expose
    services: ["get_map"]

    get_map:
      service: "/map_server/get_map"
      srv_type: "nav_msgs/srv/GetMap"
      plugin: "robot_mcp_srv_pluginlib::GetMapPlugin"
      timeout_ms: 5000

    # Define actions to expose (future)
    actions: []

    # Resource groups for conflict resolution (future)
    resource_groups: []
```

## Launch Arguments

- **params_file**: Path to configuration YAML file
  - Default: `config/minimal.yaml` from this package

- **autostart**: Automatically configure and activate the node
  - Default: `true`
  - Set to `false` for manual lifecycle control

- **namespace**: Top-level namespace for all nodes
  - Default: `""` (no namespace)
  - Useful for multi-robot setups

## Testing Your Deployment

### Check node status

```bash
# View lifecycle state
ros2 lifecycle get /mcp_http_server

# View node info
ros2 node info /mcp_http_server
```

### Test HTTP endpoint

```bash
# Basic connectivity test
curl http://localhost:8080/mcp/v1

# MCP protocol test
curl -X POST http://localhost:8080/mcp/v1 \
  -H "Content-Type: application/json" \
  -d '{
    "jsonrpc": "2.0",
    "method": "initialize",
    "params": {"protocolVersion": "1.0"},
    "id": 1
  }'
```

### With API key authentication

```bash
curl -X POST http://localhost:8080/mcp/v1 \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-secret-key" \
  -d '{"jsonrpc": "2.0", "method": "initialize", "id": 1}'
```

## Lifecycle Manager Integration

The launch file includes `nav2_lifecycle_manager` to manage the MCP server's lifecycle:

- **Automatic recovery**: Restarts node if it crashes
- **Bond monitoring**: Detects node health via heartbeat
- **Coordinated startup**: Ensures proper initialization order

### Manual lifecycle control

```bash
# Configure
ros2 lifecycle set /mcp_http_server configure

# Activate
ros2 lifecycle set /mcp_http_server activate

# Deactivate
ros2 lifecycle set /mcp_http_server deactivate

# Cleanup
ros2 lifecycle set /mcp_http_server cleanup
```

## Security Considerations

### Development (Local Network)

```yaml
server:
  host: "127.0.0.1"  # Localhost only
  port: 8080
  enable_https: false
  api_key: ""  # Optional
```

### Production (Internet-Exposed)

```yaml
server:
  host: "0.0.0.0"
  port: 8443
  enable_https: true
  ssl_cert_path: "/path/to/cert.pem"
  ssl_key_path: "/path/to/key.pem"
  api_key: "strong-random-key"  # Required!
```

**Note:** Self-signed certificates won't work with MCP clients. Use Let's Encrypt or similar for valid certificates.

## Troubleshooting

### Node fails to configure

Check logs: `ros2 topic echo /rosout`

Common issues:
- Invalid parameter format in config file
- Missing required parameters
- Invalid port number or already in use

### HTTP server not accessible

- Check firewall rules
- Verify correct host/port in config
- Check if port is already in use: `netstat -tulpn | grep 8080`

### Lifecycle manager errors

- Ensure bond support is enabled in config
- Check bondcpp is installed: `ros2 pkg list | grep bondcpp`
- Verify lifecycle manager can communicate with node

See `DEVELOPING.md` for advanced configuration and customization.
