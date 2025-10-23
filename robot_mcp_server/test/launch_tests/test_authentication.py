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

"""Integration tests for MCP server authentication."""

import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
import requests


@pytest.mark.launch_test
def generate_test_description():
    """Launch the MCP server with authentication enabled."""
    # Test API key
    test_api_key = "test_secret_key_12345"

    mcp_server_node = launch_ros.actions.LifecycleNode(
        package="robot_mcp_server",
        executable="robot_mcp_server_node",
        name="mcp_http_server",
        namespace="",
        parameters=[
            {
                "server.host": "127.0.0.2",
                "server.port": 18081,
                "server.api_key": test_api_key,
                "server.enable_https": False,
            }
        ],
        output="screen",
    )

    # Use nav2_lifecycle_manager to automatically activate the node
    lifecycle_manager = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="test_lifecycle_manager",
        parameters=[
            {"autostart": True, "node_names": ["mcp_http_server"], "bond_timeout": 4.0}
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            mcp_server_node,
            lifecycle_manager,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestAuthentication(unittest.TestCase):
    """Active tests for authentication."""

    BASE_URL = "http://127.0.0.2:18081"
    VALID_API_KEY = "test_secret_key_12345"

    @classmethod
    def setUpClass(cls):
        """Wait for server to be ready."""
        time.sleep(5)

    def test_valid_api_key(self):
        """Test request with valid API key succeeds."""
        payload = {"jsonrpc": "2.0", "method": "test", "id": 1}
        headers = {"Authorization": f"Bearer {self.VALID_API_KEY}"}

        response = requests.post(f"{self.BASE_URL}/mcp", json=payload, headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "result" in data or "message" in data

    def test_invalid_api_key(self):
        """Test request with invalid API key is rejected."""
        payload = {"jsonrpc": "2.0", "method": "test", "id": 1}
        headers = {"Authorization": "Bearer wrong_key"}

        response = requests.post(f"{self.BASE_URL}/mcp", json=payload, headers=headers)

        assert response.status_code == 401
        data = response.json()
        assert "error" in data

    def test_missing_authorization_header(self):
        """Test request without Authorization header is rejected."""
        payload = {"jsonrpc": "2.0", "method": "test", "id": 1}

        response = requests.post(f"{self.BASE_URL}/mcp", json=payload)

        assert response.status_code == 401
        data = response.json()
        assert "error" in data

    def test_malformed_authorization_header(self):
        """Test request with malformed Authorization header is rejected."""
        payload = {"jsonrpc": "2.0", "method": "test", "id": 1}
        headers = {"Authorization": "InvalidFormat"}

        response = requests.post(f"{self.BASE_URL}/mcp", json=payload, headers=headers)

        assert response.status_code == 401
        data = response.json()
        assert "error" in data

    def test_authorization_header_without_bearer(self):
        """Test request with Authorization header but no Bearer prefix is rejected."""
        payload = {"jsonrpc": "2.0", "method": "test", "id": 1}
        headers = {"Authorization": self.VALID_API_KEY}

        response = requests.post(f"{self.BASE_URL}/mcp", json=payload, headers=headers)

        assert response.status_code == 401
        data = response.json()
        assert "error" in data


@launch_testing.post_shutdown_test()
class TestAuthenticationShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_codes(self, proc_info):
        """Check that all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
