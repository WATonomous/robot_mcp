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

"""Integration tests for MCP server HTTPS/SSL support."""

import os
import shutil
import subprocess
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
import requests

from test_utils import wait_for_server


def generate_test_certificates():
    """Generate self-signed certificates for testing."""
    cert_dir = "/tmp/robot_mcp_test_certs"
    os.makedirs(cert_dir, exist_ok=True)

    cert_path = os.path.join(cert_dir, "server.crt")
    key_path = os.path.join(cert_dir, "server.key")

    # Generate private key
    subprocess.run(
        ["openssl", "genrsa", "-out", key_path, "2048"],
        check=True,
        capture_output=True,
    )

    # Generate self-signed certificate (valid for 365 days)
    subprocess.run(
        [
            "openssl",
            "req",
            "-new",
            "-x509",
            "-key",
            key_path,
            "-out",
            cert_path,
            "-days",
            "365",
            "-subj",
            "/C=US/ST=Test/L=Test/O=Test/CN=localhost",
        ],
        check=True,
        capture_output=True,
    )

    return cert_path, key_path


@pytest.mark.launch_test
def generate_test_description():
    """Launch the MCP server with HTTPS enabled."""
    # Generate test certificates
    cert_path, key_path = generate_test_certificates()

    mcp_server_node = launch_ros.actions.LifecycleNode(
        package="robot_mcp_server",
        executable="robot_mcp_server_node",
        name="mcp_http_server",
        namespace="",
        parameters=[
            {
                "server.host": "127.0.0.1",
                "server.port": 18443,
                "server.enable_https": True,
                "server.ssl_cert_path": cert_path,
                "server.ssl_key_path": key_path,
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


class TestHTTPS(unittest.TestCase):
    """Active tests for HTTPS/SSL."""

    BASE_URL = "https://127.0.0.1:18443"

    @classmethod
    def setUpClass(cls):
        """Wait for server to be ready."""
        wait_for_server(cls.BASE_URL, verify=False)

    @classmethod
    def tearDownClass(cls):
        """Clean up test certificates."""
        cert_dir = "/tmp/robot_mcp_test_certs"
        if os.path.exists(cert_dir):
            shutil.rmtree(cert_dir)

    def test_https_connection(self):
        """Test HTTPS server accepts connections."""
        payload = {"jsonrpc": "2.0", "method": "test", "id": 1}

        # Use verify=False for self-signed certificate
        response = requests.post(
            f"{self.BASE_URL}/mcp", json=payload, verify=False, timeout=5
        )

        assert response.status_code == 200
        data = response.json()
        assert "result" in data or "message" in data

    def test_https_with_valid_json_rpc(self):
        """Test HTTPS server handles JSON-RPC requests correctly."""
        payload = {
            "jsonrpc": "2.0",
            "method": "initialize",
            "params": {"clientInfo": {"name": "test", "version": "1.0"}},
            "id": 1,
        }

        response = requests.post(
            f"{self.BASE_URL}/mcp", json=payload, verify=False, timeout=5
        )

        assert response.status_code == 200
        data = response.json()
        assert "jsonrpc" in data
        assert data["jsonrpc"] == "2.0"

    def test_https_cors_headers(self):
        """Test HTTPS server returns CORS headers."""
        response = requests.options(f"{self.BASE_URL}/mcp", verify=False, timeout=5)

        assert response.status_code == 204
        assert "Access-Control-Allow-Origin" in response.headers

    def test_https_rejects_invalid_json(self):
        """Test HTTPS server rejects invalid JSON."""
        response = requests.post(
            f"{self.BASE_URL}/mcp",
            data="{invalid json}",
            headers={"Content-Type": "application/json"},
            verify=False,
            timeout=5,
        )

        assert response.status_code == 400
        data = response.json()
        assert "error" in data


@launch_testing.post_shutdown_test()
class TestHTTPSShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_codes(self, proc_info):
        """Check that all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
