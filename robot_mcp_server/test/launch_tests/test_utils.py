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

"""Shared utilities for launch tests."""

import time

import requests


def wait_for_server(
  base_url,
  max_attempts=50,
  timeout=0.5,
  verify=True,
  headers=None,
):
  """Poll server until ready with exponential backoff.

  Args:
    base_url: Base URL of the server (e.g., "http://127.0.0.1:8080")
    max_attempts: Maximum number of connection attempts
    timeout: Timeout in seconds for each request
    verify: Whether to verify SSL certificates (False for self-signed certs)
    headers: Optional headers to send with the request (e.g., for auth)

  Raises:
    RuntimeError: If server does not respond after max_attempts

  """
  for attempt in range(max_attempts):
    try:
      requests.post(
        f"{base_url}/mcp",
        json={"jsonrpc": "2.0", "method": "ping", "id": 0},
        timeout=timeout,
        verify=verify,
        headers=headers or {},
      )
      # Server is responding, we're ready
      return
    except (requests.exceptions.ConnectionError, requests.exceptions.Timeout):
      if attempt == max_attempts - 1:
        raise RuntimeError(f"Server at {base_url} did not start after {max_attempts * 0.1}s")
      time.sleep(0.1)
