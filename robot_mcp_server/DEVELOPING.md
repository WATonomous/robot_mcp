# robot_mcp_server Development Guide

## Classes

See header files for detailed API documentation.

### Main
- **`MCPServerNode`**: Lifecycle node, orchestrates config/HTTP/bond

### Config (`mcp_config/`)
- **`ConfigParser`**: Parse ROS2 params → structs (Nav2 pattern)
- **`config_types.hpp`**: Plain C++ config structs (ServerConfig, TopicConfig, etc.)
- **`ConfigParseException`**: Config error exception

### HTTP (`mcp_http_server/`)
- **`HTTPServer`**: cpp-httplib wrapper, HTTP/HTTPS switching
- **`JSONRPCHandler`**: JSON-RPC 2.0 protocol, static utility
- **`AuthMiddleware`**: API key auth, static utility

### Plugin System (Phase 4 - planned)
- **`MessagePlugin`**, **`ServicePlugin`**, **`ActionPlugin`**: Base classes for pluginlib

## Request Flow

```
HTTP POST → AuthMiddleware → JSONRPCHandler → MCPServerNode.handleMCPRequest()
→ (Phase 3: Router → Plugins) → JSON-RPC Response
```

## Testing

```bash
# Run tests
colcon test --packages-select robot_mcp_server --event-handlers console_direct+

# Manual test
ros2 launch robot_mcp_bringup robot_mcp.launch.yaml
curl -X POST http://localhost:8080/mcp -H "Content-Type: application/json" \
  -d '{"jsonrpc": "2.0", "method": "test", "id": 1}'
```

**Test files:**
- `test/test_lifecycle.cpp`: Unit tests
- `test_mcp_config/*.py`: Launch tests with config files

## Development Status

- Phase 1: Config ✅
- Phase 2: HTTP ✅
- Phase 3: Router (in progress)
- Phase 4: Plugins (planned)
