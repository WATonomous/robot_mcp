# Phase 2 Implementation Catchup

**Date:** 2025-10-21
**Phase:** 2 - HTTP Server Layer
**Status:** Implementation complete, needs testing in devcontainer

---

## What Was Completed

### Phase 2: HTTP Server Layer Implementation

We successfully implemented the HTTP server layer for robot_mcp. This provides the foundation for accepting MCP (Model Context Protocol) requests over HTTP.

**Components Created:**

1. **AuthMiddleware** - Validates API key from Authorization header (Bearer token)
2. **JSONRPCHandler** - Parses and formats JSON-RPC 2.0 requests/responses
3. **HTTPServer** - Main HTTP server using cpp-httplib, integrates auth and JSON-RPC

**Integration:**
- HTTPServer integrated into MCPServerNode lifecycle
- Starts on `activate()`, stops on `deactivate()`
- Placeholder request handler returns Phase 2 completion message

---

## Files Modified

### New Files (6 total):
```
robot_mcp_server/
├── include/robot_mcp_server/mcp_http_server/
│   ├── auth_middleware.hpp       [NEW]
│   ├── json_rpc_handler.hpp      [NEW]
│   └── http_server.hpp           [NEW]
│
└── src/mcp_http_server/
    ├── auth_middleware.cpp       [NEW]
    ├── json_rpc_handler.cpp      [NEW]
    └── http_server.cpp           [NEW]
```

### Modified Files (5 total):
1. `robot_mcp_server/include/robot_mcp_server/mcp_config/config_types.hpp`
   - Added `thread_pool_size` to ServerConfig (default: 10)

2. `robot_mcp_server/src/mcp_config/config_parser.cpp`
   - Parse `thread_pool_size` parameter

3. `robot_mcp_server/include/robot_mcp_server/robot_mcp_server_node.hpp`
   - Added `http_server_` member
   - Added `handleMCPRequest()` method declaration

4. `robot_mcp_server/src/robot_mcp_server_node.cpp`
   - Create HTTPServer in `on_activate()`
   - Stop in `on_deactivate()`
   - Cleanup in `on_cleanup()` and `on_shutdown()`
   - Implemented `handleMCPRequest()` placeholder

5. `robot_mcp_server/CMakeLists.txt`
   - Added new source files to build

---

## Architecture Summary

### Class Ownership:
```
MCPServerNode (lifecycle node)
  └── owns → HTTPServer (unique_ptr)
      ├── owns → AuthMiddleware (unique_ptr)
      └── owns → JSONRPCHandler (unique_ptr)
```

### Request Flow:
```
1. HTTP POST /mcp
2. AuthMiddleware validates API key
3. JSONRPCHandler parses request
4. MCPServerNode::handleMCPRequest() called (placeholder)
5. JSONRPCHandler formats response
6. HTTP response sent
```

### Threading Model:
- **HTTP Thread Pool:** cpp-httplib creates fixed pool (configurable, default: 10)
- **ROS2 Executor:** Separate thread pool managed by rclcpp
- **No threads created per request** - both pools are fixed size

---

## Next Steps (DO THIS IN DEVCONTAINER)

### 1. Build the Package
```bash
cd /workspaces/robot_mcp  # or wherever your workspace is
colcon build --packages-select robot_mcp_server
source install/setup.bash
```

### 2. Expected Build Output
Should compile successfully with no errors. Watch for:
- All 6 new files compiling
- No missing headers (cpp-httplib, nlohmann/json)
- Linking successful

### 3. Run the Node
```bash
ros2 run robot_mcp_server robot_mcp_server_node
```

The node will start in UNCONFIGURED state (lifecycle).

### 4. Trigger Lifecycle Transitions

In a separate terminal:
```bash
# Configure (load config)
ros2 lifecycle set /mcp_http_server configure

# Activate (start HTTP server)
ros2 lifecycle set /mcp_http_server activate
```

Expected output:
```
[INFO] [robot_mcp.mcp_http_server]: Configuring MCP server...
[INFO] [robot_mcp.mcp_http_server]: Configuration loaded successfully
[INFO] [robot_mcp.mcp_http_server]: Activating MCP server...
[INFO] [robot_mcp.mcp_http_server]: HTTP server started on 0.0.0.0:8080
[WARN] [robot_mcp.mcp_http_server]: Authentication: DISABLED
[INFO] [robot_mcp.mcp_http_server]: MCP server activated successfully on 0.0.0.0:8080
```

### 5. Test HTTP Endpoint

**Test 1: Basic JSON-RPC request**
```bash
curl -X POST http://localhost:8080/mcp \
  -H "Content-Type: application/json" \
  -d '{
    "jsonrpc": "2.0",
    "id": 1,
    "method": "initialize",
    "params": {}
  }'
```

**Expected Response:**
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "message": "MCP server Phase 2 complete - HTTP layer working",
    "method": "initialize",
    "note": "Actual MCP protocol implementation coming in Phase 3",
    "server_info": {
      "name": "robot_mcp_server",
      "version": "0.1.0",
      "phase": 2
    }
  }
}
```

**Test 2: Invalid JSON (should return parse error)**
```bash
curl -X POST http://localhost:8080/mcp \
  -H "Content-Type: application/json" \
  -d 'not valid json'
```

**Expected Response:**
```json
{
  "jsonrpc": "2.0",
  "id": null,
  "error": {
    "code": -32700,
    "message": "Parse error: Invalid JSON",
    "data": "..."
  }
}
```

**Test 3: Invalid JSON-RPC structure**
```bash
curl -X POST http://localhost:8080/mcp \
  -H "Content-Type: application/json" \
  -d '{"foo": "bar"}'
```

**Expected Response:**
```json
{
  "jsonrpc": "2.0",
  "id": null,
  "error": {
    "code": -32600,
    "message": "Invalid Request: Missing required JSON-RPC fields"
  }
}
```

**Test 4: CORS preflight**
```bash
curl -X OPTIONS http://localhost:8080/mcp \
  -H "Access-Control-Request-Method: POST" \
  -H "Origin: http://example.com" \
  -v
```

Should return 204 No Content with CORS headers.

---

## Known Issues / Things to Watch

1. **Port 8080 in use:** If server fails to start, port might be occupied
   - Check: `sudo lsof -i :8080`
   - Change port in config if needed

2. **cpp-httplib not found:** Make sure system dependency is installed
   - Should be handled by package.xml dependency

3. **Lifecycle state:** Node must be configured AND activated before HTTP server runs

---

## What Comes Next (Phase 3)

After Phase 2 testing is complete:

**Phase 3: MCP Protocol Router**
- Implement MCPRouter class
- Handle MCP methods: initialize, tools/list, tools/call, resources/list, resources/read
- Replace placeholder `handleMCPRequest()` with actual router
- Built-in tools: status, list, cancel

**Phase 4: Plugin System**
- Implement PluginFactory and PluginLoader
- Create base plugin classes (TopicPlugin, ServicePlugin, ActionPlugin)
- Autodiscovery via pluginlib

**Phase 5: Basic Message Plugins**
- Implement concrete plugins for common message types
- std_msgs, geometry_msgs, sensor_msgs, nav_msgs

---

## Important Context from Discussion

### Threading Model (Clarified):
- **Fixed thread pools**, not thread-per-request
- HTTP threads handle requests synchronously (blocking)
- ROS2 executor threads handle callbacks
- For long-running actions: return operation_id immediately, poll for status

### Design Principles:
- **Not a God Object:** HTTPServer only handles HTTP layer
- Delegates auth to AuthMiddleware
- Delegates JSON-RPC to JSONRPCHandler
- Delegates MCP protocol to Router (Phase 3)
- Each class has ONE responsibility (SRP)

### Resource Management:
- Thread pool prevents unbounded resource usage
- `max_connections` limits queue depth
- `timeout_ms` prevents threads blocking forever
- Future: rate limiting, queue limits per resource group

---

## Testing Checklist

- [ ] Build succeeds with no errors
- [ ] Node starts and reaches UNCONFIGURED state
- [ ] Configure transition succeeds, loads config
- [ ] Activate transition succeeds, starts HTTP server
- [ ] Can send valid JSON-RPC request, get Phase 2 response
- [ ] Invalid JSON returns parse error (-32700)
- [ ] Invalid JSON-RPC structure returns invalid request (-32600)
- [ ] CORS headers present (if enabled)
- [ ] Deactivate stops server cleanly
- [ ] Cleanup releases resources

---

## Quick Reference Commands

```bash
# Build
colcon build --packages-select robot_mcp_server

# Source
source install/setup.bash

# Run node
ros2 run robot_mcp_server robot_mcp_server_node

# Lifecycle transitions (separate terminal)
ros2 lifecycle set /mcp_http_server configure
ros2 lifecycle set /mcp_http_server activate

# Test endpoint
curl -X POST http://localhost:8080/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":1,"method":"test","params":{}}'

# Check lifecycle state
ros2 lifecycle get /mcp_http_server

# Deactivate
ros2 lifecycle set /mcp_http_server deactivate

# Cleanup
ros2 lifecycle set /mcp_http_server cleanup
```

---

## Files to Review (If Debugging)

1. `robot_mcp_server/src/mcp_http_server/http_server.cpp:handleMCPEndpoint()` - Main request handler
2. `robot_mcp_server/src/robot_mcp_server_node.cpp:handleMCPRequest()` - Placeholder response
3. `robot_mcp_server/src/mcp_http_server/auth_middleware.cpp:validate()` - Auth logic
4. `robot_mcp_server/src/mcp_http_server/json_rpc_handler.cpp` - JSON-RPC parsing

---

## Success Criteria for Phase 2

✅ HTTP server starts on lifecycle activate
✅ Accepts POST requests to /mcp endpoint
✅ Parses JSON-RPC 2.0 correctly
✅ Returns proper error codes for invalid requests
✅ Authentication middleware works (test with api_key config)
✅ CORS headers present
✅ Server stops cleanly on deactivate
✅ No memory leaks (run with valgrind if paranoid)

**When all tests pass → Phase 2 COMPLETE! Ready for Phase 3 (MCP Router)** 🎉
