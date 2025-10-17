# robot_mcp

**Model Context Protocol server for ROS2 robots - Control your robot with natural language through AI assistants like Claude.**

## Features

- **Out-of-the-box solution** for linking LLMs to ROS2 messages, actions, and services

- **Expansive macro architecture** to allow for custom message handling with MCP

- **HTTP-based MCP server** to allow for remote connections to multiple robots with authentication

- **Comprehensive configuration settings** to allow for fine-grained behavior control

- **Resource group management** to handle possible concurrency conflicts

- **Native C++ implementation** to maximize performance

- **rclcpp_lifecycle management** to allow for fine-grained control over robot_mcp's lifespan

## Quick Start

```bash
# Install debian package
sudo apt install ros-<$ROS_DISTRO>-robot-mcp

# source ROS2 and Run
source /opt/ros/<$ROS_DISTRO>/setup.bash
ros2 launch robot_mcp robot_mcp.launch.py
```

**Configure Claude Desktop** to connect at `http://localhost:8080/mcp`

## Status

**Under active development** - Documentation and examples coming soon.
