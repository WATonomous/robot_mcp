#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Copyright 2025 WATonomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest
import pytest
import launch
import launch.actions
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
import launch_ros.event_handlers
import launch_testing.actions
import launch_testing.markers
import launch_testing.asserts
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch MCP server node with test configuration."""

    # Get the test config directory
    pkg_share = get_package_share_directory("robot_mcp_server")
    config_dir = os.path.join(pkg_share, "test", "launch_tests")
    config_file = os.path.join(config_dir, "complete_config.yaml")

    mcp_server_node = launch_ros.actions.LifecycleNode(
        package="robot_mcp_server",
        executable="robot_mcp_server_node",
        name="mcp_http_server",
        namespace="",
        parameters=[config_file],
        output="both",
        emulate_tty=True,
    )

    # Emit event when node reaches 'inactive' state (after configure)
    register_event_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=mcp_server_node,
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(msg="Node configured successfully"),
                launch_testing.actions.ReadyToTest(),
            ],
        )
    )

    # Configure the node after launch
    configure_node = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(mcp_server_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return launch.LaunchDescription(
        [
            mcp_server_node,
            register_event_handler,
            launch.actions.TimerAction(period=1.0, actions=[configure_node]),
        ]
    )


class TestConfigParser(unittest.TestCase):
    """Active tests while the node is running."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for parameter queries."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2."""
        rclpy.shutdown()

    def test_node_configures(self, proc_output):
        """Verify the node configures successfully."""
        # Wait for configuration success message
        proc_output.assertWaitFor("Configuration loaded successfully", timeout=10.0)

    def test_server_config_loaded(self):
        """Verify server configuration was loaded from YAML via ROS2 parameter API."""
        node = Node("test_param_client")

        try:
            # Create service client to query parameters
            cli = node.create_client(GetParameters, "/mcp_http_server/get_parameters")

            # Wait for service to be available
            self.assertTrue(
                cli.wait_for_service(timeout_sec=5.0), "Parameter service not available"
            )

            # Query server parameters
            request = GetParameters.Request()
            request.names = ["server.host", "server.port"]

            future = cli.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            self.assertTrue(future.done(), "Parameter query timed out")
            response = future.result()

            # Verify server config from complete_config.yaml
            self.assertEqual(response.values[0].string_value, "127.0.0.1")
            self.assertEqual(response.values[1].integer_value, 9090)

        finally:
            node.destroy_node()

    def test_topics_list_loaded(self):
        """Verify topics list was loaded from YAML via ROS2 parameter API."""
        node = Node("test_topics_client")

        try:
            # Create service client to query parameters
            cli = node.create_client(GetParameters, "/mcp_http_server/get_parameters")

            # Wait for service to be available
            self.assertTrue(
                cli.wait_for_service(timeout_sec=5.0), "Parameter service not available"
            )

            # Query topics parameter
            request = GetParameters.Request()
            request.names = ["topics"]

            future = cli.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            self.assertTrue(future.done(), "Parameter query timed out")
            response = future.result()

            # Verify topics array from complete_config.yaml
            topics_array = response.values[0].string_array_value
            self.assertEqual(len(topics_array), 2)
            self.assertIn("battery", topics_array)
            self.assertIn("cmd_vel", topics_array)

        finally:
            node.destroy_node()

    def test_topic_configs_loaded(self):
        """Verify individual topic configurations were loaded from YAML."""
        node = Node("test_topic_config_client")

        try:
            # Create service client to query parameters
            cli = node.create_client(GetParameters, "/mcp_http_server/get_parameters")

            # Wait for service to be available
            self.assertTrue(
                cli.wait_for_service(timeout_sec=5.0), "Parameter service not available"
            )

            # Query battery topic config
            request = GetParameters.Request()
            request.names = [
                "battery.topic",
                "battery.msg_type",
                "battery.plugin",
                "battery.subscribe",
            ]

            future = cli.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            self.assertTrue(future.done(), "Parameter query timed out")
            response = future.result()

            # Verify battery topic config from complete_config.yaml
            self.assertEqual(response.values[0].string_value, "/battery_state")
            self.assertEqual(
                response.values[1].string_value, "sensor_msgs/msg/BatteryState"
            )
            self.assertEqual(
                response.values[2].string_value,
                "robot_mcp_sensor_msg_plugins::BatteryStatePlugin",
            )
            self.assertTrue(response.values[3].bool_value)

        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestConfigParserShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_codes(self, proc_info):
        """Check that all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
