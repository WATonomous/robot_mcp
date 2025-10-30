// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Main header for robot_mcp_test library
// Include this to access all robot_mcp_test functionality including Catch2

// Cross-compatible Catch2 include for Ubuntu 22.04 (Catch2 v2) and Ubuntu 24.04 (Catch2 v3)
#ifdef CATCH2_V3
#include <catch2/catch_test_macros.hpp>
#else
#include <catch2/catch.hpp>
#endif

// Test fixtures
#include "test_fixtures/test_executor_fixture.hpp"

// Test nodes
#include "test_nodes/client_test_node.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/service_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"
