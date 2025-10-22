# robot_mcp_bringup Development Guide

Technical details for launch files and configuration patterns in `robot_mcp_bringup`.

## Package Structure

```
robot_mcp_bringup/
├── launch/
│   └── robot_mcp.launch.yaml    # Main launch file
├── config/
│   └── minimal.yaml              # Example configuration
├── README.md                     # User guide
└── DEVELOPING.md                 # This file
```

## Launch File Pattern

### YAML Launch Format

Using ROS2 YAML launch format for declarative configuration:

```yaml
launch:
  # Define arguments
  - arg:
      name: params_file
      default: "$(find-pkg-share robot_mcp_bringup)/config/minimal.yaml"
      description: "Path to configuration file"

  # Launch nodes
  - node:
      pkg: robot_mcp_server
      exec: robot_mcp_server_node
      name: mcp_http_server
      namespace: $(var namespace)
      output: screen
      param:
        - from: $(var params_file)
```

**Benefits of YAML launch:**
- Declarative, easy to read
- No Python imports needed
- Direct parameter file loading
- Composable via included launch files

### Lifecycle Manager Integration

Pattern for managed lifecycle nodes:

```yaml
# 1. Launch the lifecycle node
- node:
    pkg: robot_mcp_server
    exec: robot_mcp_server_node
    name: mcp_http_server
    # ... node config

# 2. Launch lifecycle manager
- node:
    pkg: nav2_lifecycle_manager
    exec: lifecycle_manager
    name: robot_mcp_lifecycle_manager
    param:
      - name: autostart
        value: $(var autostart)
      - name: node_names
        value: ['mcp_http_server']  # List of nodes to manage
```

**Lifecycle manager features:**
- Automatic node startup/shutdown
- Bond-based health monitoring
- Coordinated multi-node lifecycle
- Recovery from failures

## Configuration Pattern

### ROS2 Parameter Structure

Parameters use nested namespace pattern from Nav2:

```yaml
node_name:
  ros__parameters:
    # Simple parameters
    server:
      host: "0.0.0.0"
      port: 8080

    # List-based resources
    topics: ["name1", "name2"]  # List of resource names

    # Each resource gets its own namespace
    name1:
      field1: value1
      field2: value2
```

**Why this pattern?**
- Clear organization
- Easy to add/remove resources
- No array-of-objects (ROS2 limitation)
- Validated at runtime by ConfigParser

### Adding New Configuration

1. **Add to config file:**
```yaml
topics: ["my_new_topic"]  # Add name to list

my_new_topic:
  topic: "/my/topic"
  msg_type: "std_msgs/msg/String"
  plugin: "robot_mcp_msg_pluginlib::StringPlugin"
```

2. **Parser automatically loads it** - no code changes needed!

3. **Add plugin** (Phase 4) to handle the message type

### Configuration Validation

Validation happens in `robot_mcp_server::config::ConfigParser`:
- Port range (1-65535)
- Required fields present
- No duplicate names
- Valid resource group references
- HTTPS requires cert/key paths

**Errors appear in logs** during the `configure` transition.

## Testing Launch Files

### Basic launch test

```bash
# Launch and check output
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml

# In another terminal, verify node started
ros2 node list | grep mcp_http_server
ros2 lifecycle get /mcp_http_server  # Should show "active"
```

### Test with custom config

```bash
# Create test config
cat > /tmp/test_config.yaml << EOF
mcp_http_server:
  ros__parameters:
    server:
      port: 9999  # Different port
EOF

# Launch with custom config
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml \
  params_file:=/tmp/test_config.yaml

# Verify it used custom port
curl http://localhost:9999/mcp/v1
```

### Test lifecycle transitions

```bash
# Launch without autostart
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml autostart:=false

# Manually test transitions
ros2 lifecycle set /mcp_http_server configure  # Should succeed
ros2 lifecycle set /mcp_http_server activate   # Should succeed
ros2 lifecycle get /mcp_http_server             # Should show "active"

# Test HTTP endpoint works
curl http://localhost:8080/mcp/v1

# Deactivate and verify
ros2 lifecycle set /mcp_http_server deactivate
curl http://localhost:8080/mcp/v1  # Should fail or timeout
```

## Advanced Patterns

### Multi-Robot Setup

Use namespaces to run multiple instances:

```bash
# Robot 1
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml \
  namespace:=/robot1 \
  params_file:=/path/to/robot1_config.yaml

# Robot 2
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml \
  namespace:=/robot2 \
  params_file:=/path/to/robot2_config.yaml
```

**Note:** Each needs unique port in config!

### Composable Nodes (Future)

For performance, consider composition:

```yaml
# Load into container instead of separate process
- node:
    pkg: rclcpp_components
    exec: component_container
    name: mcp_container

- load_composable_node:
    target: mcp_container
    plugin: robot_mcp::MCPServerNode
    name: mcp_http_server
```

**Benefits:**
- Lower memory usage
- Reduced IPC overhead
- Better performance

### Custom Lifecycle Manager

For integration with existing lifecycle managers:

```yaml
# Don't launch lifecycle manager, just the node
- node:
    pkg: robot_mcp_server
    exec: robot_mcp_server_node
    name: mcp_http_server
    # ... config

# Your existing lifecycle manager should include:
# node_names: [..., 'mcp_http_server']
```

## Configuration Templates

### Development Template

```yaml
# Fast iteration, no security
mcp_http_server:
  ros__parameters:
    server:
      host: "127.0.0.1"  # Localhost only
      port: 8080
      enable_https: false
      api_key: ""  # No auth

    topics: ["status"]
    status:
      topic: "/robot/status"
      msg_type: "std_msgs/msg/String"
      plugin: "robot_mcp_msg_pluginlib::StringPlugin"
      subscribe: true
```

### Production Template

```yaml
# Secure, internet-exposed
mcp_http_server:
  ros__parameters:
    server:
      host: "0.0.0.0"
      port: 8443
      enable_https: true
      ssl_cert_path: "/etc/letsencrypt/live/robot.example.com/fullchain.pem"
      ssl_key_path: "/etc/letsencrypt/live/robot.example.com/privkey.pem"
      api_key: "${API_KEY}"  # From environment

    # Full topic/service/action configuration
    # ... (based on your robot's capabilities)
```

### Multi-Robot Template

```yaml
# robot1_config.yaml
mcp_http_server:
  ros__parameters:
    server:
      port: 8081  # Unique per robot
      api_key: "${ROBOT1_API_KEY}"
    # ...

# robot2_config.yaml
mcp_http_server:
  ros__parameters:
    server:
      port: 8082  # Different port
      api_key: "${ROBOT2_API_KEY}"
    # ...
```

## Debugging

### Launch file issues

```bash
# Verbose launch output
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml --debug

# Check parameter loading
ros2 param list /mcp_http_server
ros2 param get /mcp_http_server server.port
```

### Configuration parsing errors

Check logs for ConfigParseException:
```bash
ros2 topic echo /rosout | grep mcp_http_server
```

Common errors:
- Missing required parameter
- Invalid parameter type (string vs int)
- Duplicate resource names
- Invalid resource group reference

### Lifecycle manager issues

```bash
# Check lifecycle manager status
ros2 node info /robot_mcp_lifecycle_manager

# Check bond connection
ros2 topic list | grep bond
ros2 topic echo /bond  # Should see heartbeat messages
```

## Contributing Launch Files

When adding new launch files:

1. Use YAML format for declarative launches
2. Provide launch arguments for common customizations
3. Include example configuration files
4. Document launch arguments in comments
5. Test with and without lifecycle manager
6. Test with different configurations

Keep it simple - complex launch logic belongs in Python launch files.
