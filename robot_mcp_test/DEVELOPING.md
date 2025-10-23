# Developing robot_mcp_test

## Building

```bash
colcon build --packages-select robot_mcp_test
```

## Running Tests

```bash
colcon test --packages-select robot_mcp_test
colcon test-result --verbose
```

## Adding New Features

### Test Fixtures
This includes code that follows the definition of a fixture as per catch2.

1. Add header to `include/test_fixtures/`
2. Add implementation to `src/` if needed
3. Include in `include/robot_mcp_test/robot_mcp_test.hpp`

### Test Nodes
This includes code that acts as helper ros nodes for testing. They should be inherently event-driven nodes (do not introduce arbitrary sleeps).

1. Add header to `include/test_nodes/`
2. Include in main header
3. Keep nodes header-only when possible

## CMake Function

The `add_robot_mcp_test()` function automatically:
- Links Catch2::Catch2WithMain
- Links ROS dependencies (rclcpp, rclcpp_lifecycle)
- Configures ament test registration
- Sets up JUnit XML output

The `add_robot_mcp_launch_test()` function automatically:
- Links up test utility files within `test/launch_tests` directory of a project
- Run the ROS `add_launch_test` function
 