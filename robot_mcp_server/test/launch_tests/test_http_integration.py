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

"""Integration test for HTTP server and MCP endpoint."""

import unittest

import pytest
import requests
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import LifecycleNode, Node
from launch_testing.actions import ReadyToTest
from test_utils import wait_for_server


def generate_test_description():
  """Launch the MCP server with lifecycle manager."""

  # Launch the lifecycle node with test configuration
  mcp_server_node = LifecycleNode(
    package="robot_mcp_server",
    executable="robot_mcp_server_node",
    name="mcp_http_server",
    namespace="",
    output="screen",
    parameters=[
      {
        "server.host": "127.0.0.1",
        "server.port": 18080,  # Use different port for testing
        "server.api_key": "",  # No auth for testing
        "server.enable_https": False,
      }
    ],
  )

  # Launch lifecycle manager to auto-start the node
  lifecycle_manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="test_lifecycle_manager",
    output="screen",
    parameters=[{"autostart": True, "node_names": ["mcp_http_server"]}],
  )

  return LaunchDescription(
    [
      mcp_server_node,
      lifecycle_manager,
      TimerAction(
        period=3.0,  # Wait for lifecycle manager to activate node
        actions=[ReadyToTest()],
      ),
    ]
  )


class TestHTTPIntegration(unittest.TestCase):
  """Test HTTP server integration."""

  BASE_URL = "http://127.0.0.1:18080"

  @classmethod
  def setUpClass(cls):
    """Wait for server to be ready."""
    wait_for_server(cls.BASE_URL)

  def test_server_responds_to_valid_jsonrpc_request(self):
    """Test server responds with valid JSON-RPC response."""

    # Send valid JSON-RPC request
    payload = {
      "jsonrpc": "2.0",
      "method": "test",
      "params": {"key": "value"},
      "id": 1,
    }

    response = requests.post(
      f"{self.BASE_URL}/mcp",
      json=payload,
      headers={"Content-Type": "application/json"},
      timeout=5.0,
    )

    # Check HTTP status
    assert response.status_code == 200, f"Expected 200, got {response.status_code}"

    # Parse JSON response
    data = response.json()

    # Validate JSON-RPC response structure
    assert "jsonrpc" in data, "Response missing 'jsonrpc' field"
    assert data["jsonrpc"] == "2.0", "Invalid JSON-RPC version"
    assert "id" in data, "Response missing 'id' field"
    assert data["id"] == 1, "Response id doesn't match request"
    assert "result" in data, "Response missing 'result' field"

    # Check Phase 2 placeholder response
    assert "message" in data["result"]
    assert "MCP server Phase 2" in data["result"]["message"]

  def test_server_rejects_invalid_json(self):
    """Test server returns error for malformed JSON."""
    # Send invalid JSON
    response = requests.post(
      f"{self.BASE_URL}/mcp",
      data="{not valid json}",
      headers={"Content-Type": "application/json"},
      timeout=5.0,
    )

    # Should return 400 with JSON-RPC error
    assert response.status_code == 400

    data = response.json()
    assert "error" in data
    assert data["error"]["code"] == -32700  # Parse error

  def test_server_rejects_invalid_jsonrpc_structure(self):
    """Test server returns error for invalid JSON-RPC structure."""
    # Send JSON missing required fields
    payload = {
      "method": "test",
      "id": 1,
      # Missing 'jsonrpc' field
    }

    response = requests.post(
      f"{self.BASE_URL}/mcp",
      json=payload,
      headers={"Content-Type": "application/json"},
      timeout=5.0,
    )

    # Should return 400 with JSON-RPC error
    assert response.status_code == 400

    data = response.json()
    assert "error" in data
    assert data["error"]["code"] == -32600  # Invalid request

  def test_server_handles_cors_preflight(self):
    """Test server handles CORS preflight OPTIONS request."""
    # Send OPTIONS request
    response = requests.options(
      f"{self.BASE_URL}/mcp",
      headers={
        "Access-Control-Request-Method": "POST",
        "Access-Control-Request-Headers": "Content-Type",
      },
      timeout=5.0,
    )

    # Should return 204 No Content for preflight
    assert response.status_code == 204

    # Check CORS headers present
    assert "Access-Control-Allow-Origin" in response.headers
    assert "Access-Control-Allow-Methods" in response.headers

  def test_server_handles_multiple_requests(self):
    """Test server can handle multiple concurrent requests."""
    # Send multiple requests with different IDs
    responses = []
    for i in range(5):
      payload = {"jsonrpc": "2.0", "method": "test", "id": i}

      response = requests.post(
        f"{self.BASE_URL}/mcp",
        json=payload,
        headers={"Content-Type": "application/json"},
        timeout=5.0,
      )
      responses.append(response)

    # All should succeed
    for i, response in enumerate(responses):
      assert response.status_code == 200
      data = response.json()
      assert data["id"] == i  # ID should match request

  def test_server_responds_with_null_id(self):
    """Test server handles requests with null id."""
    payload = {"jsonrpc": "2.0", "method": "test", "id": None}

    response = requests.post(
      f"{self.BASE_URL}/mcp",
      json=payload,
      headers={"Content-Type": "application/json"},
      timeout=5.0,
    )

    assert response.status_code == 200
    data = response.json()
    assert data["id"] is None


@pytest.mark.launch_test
def test_http_integration_launch():
  """Launch test marker."""
  pass
