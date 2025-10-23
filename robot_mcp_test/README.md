# robot_mcp_test

A ROS 2 testing framework built on Catch2 that provides test fixtures and utilities for testing ROS nodes, services, and communication patterns.

## Features

- **TestExecutorFixture**: Manages ROS executor lifecycle for tests
- **Test Nodes**: Pre-built nodes for testing publishers, subscribers, services, and clients
- **CMake Integration**: Simple `add_robot_mcp_test()` function for creating tests
- **Catch2 Integration**: Built on Catch2 for modern C++ testing

## Quick Start

```cpp
#include <catch2/catch.hpp>
#include <robot_mcp_test/robot_mcp_test.hpp>

TEST_CASE_METHOD(robot_mcp::test::TestExecutorFixture, "My ROS Test", "[ros]") {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  add_node(node);
  start_spinning();

  // Your test code here as sections
  SECTION("My test") {
    REQUIRE(node->get_name() == "test_node");
  }
}
```

## Package.xml Usage

```xml
<test_depend>robot_mcp_test</test_depend>
```

## CMake Usage

### Catch2 Unit Tests

```cmake
find_package(robot_mcp_test REQUIRED)

add_robot_mcp_test(my_test
  test/my_test.cpp
  LIBRARIES
    my_package::my_library
    std_msgs::std_msgs
)
```

### Launch Tests

```cmake
find_package(robot_mcp_test REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

add_robot_mcp_launch_test(test/launch_tests/test_example.py)
```

The `add_robot_mcp_launch_test()` macro automatically:
- Wraps `add_launch_test()` from launch_testing_ament_cmake
- Adds `test/launch_tests` to `PYTHONPATH` for importing shared utilities
- Allows test files to import from `test_utils.py` for common test helpers

**Example test_utils.py pattern:**

```python
# test/launch_tests/test_utils.py
import time
import requests

def wait_for_server(base_url, max_attempts=50):
    """Poll server until ready."""
    for attempt in range(max_attempts):
        try:
            requests.post(f"{base_url}/health", timeout=0.5)
            return
        except (requests.exceptions.ConnectionError, requests.exceptions.Timeout):
            if attempt == max_attempts - 1:
                raise RuntimeError("Server did not start")
            time.sleep(0.1)

# test/launch_tests/test_my_server.py
from test_utils import wait_for_server

class TestMyServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wait_for_server("http://localhost:8080")
```
