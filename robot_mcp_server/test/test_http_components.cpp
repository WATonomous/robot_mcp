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

#include <string>

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

#include "robot_mcp_server/mcp_http_server/json_rpc_handler.hpp"

using robot_mcp::http::JSONRPCHandler;

// Note: Auth middleware tests are covered by integration tests
// since they require real httplib::Request objects

TEST_CASE("JSONRPCHandler - Parse valid JSON", "[json-rpc]")
{
  SECTION("Parse simple JSON object")
  {
    std::string json_str = R"({"jsonrpc": "2.0", "method": "test", "id": 1})";
    auto parsed = JSONRPCHandler::parse(json_str);
    REQUIRE(parsed["jsonrpc"] == "2.0");
    REQUIRE(parsed["method"] == "test");
    REQUIRE(parsed["id"] == 1);
  }

  SECTION("Parse JSON with params")
  {
    std::string json_str = R"({
      "jsonrpc": "2.0",
      "method": "test",
      "params": {"foo": "bar"},
      "id": 1
    })";
    auto parsed = JSONRPCHandler::parse(json_str);
    REQUIRE(parsed["params"]["foo"] == "bar");
  }
}

TEST_CASE("JSONRPCHandler - Parse invalid JSON", "[json-rpc]")
{
  SECTION("Should throw on malformed JSON")
  {
    std::string json_str = "{not valid json}";
    REQUIRE_THROWS_AS(JSONRPCHandler::parse(json_str), nlohmann::json::parse_error);
  }

  SECTION("Should throw on empty string")
  {
    REQUIRE_THROWS_AS(JSONRPCHandler::parse(""), nlohmann::json::parse_error);
  }
}

TEST_CASE("JSONRPCHandler - Validate request structure", "[json-rpc]")
{
  SECTION("Valid JSON-RPC 2.0 request")
  {
    nlohmann::json req = {{"jsonrpc", "2.0"}, {"method", "test"}, {"id", 1}};
    REQUIRE(JSONRPCHandler::validate(req) == true);
  }

  SECTION("Valid request with params")
  {
    nlohmann::json req = {{"jsonrpc", "2.0"}, {"method", "test"}, {"params", {{"key", "value"}}}, {"id", 1}};
    REQUIRE(JSONRPCHandler::validate(req) == true);
  }

  SECTION("Invalid - missing jsonrpc field")
  {
    nlohmann::json req = {{"method", "test"}, {"id", 1}};
    REQUIRE(JSONRPCHandler::validate(req) == false);
  }

  SECTION("Invalid - missing method field")
  {
    nlohmann::json req = {{"jsonrpc", "2.0"}, {"id", 1}};
    REQUIRE(JSONRPCHandler::validate(req) == false);
  }

  SECTION("Invalid - wrong jsonrpc version")
  {
    nlohmann::json req = {{"jsonrpc", "1.0"}, {"method", "test"}, {"id", 1}};
    REQUIRE(JSONRPCHandler::validate(req) == false);
  }

  SECTION("Valid - id can be null")
  {
    nlohmann::json req = {{"jsonrpc", "2.0"}, {"method", "test"}, {"id", nullptr}};
    REQUIRE(JSONRPCHandler::validate(req) == true);
  }
}

TEST_CASE("JSONRPCHandler - Format success response", "[json-rpc]")
{
  SECTION("Format result with numeric id")
  {
    nlohmann::json result = {{"data", "test"}};
    auto response = JSONRPCHandler::formatSuccess(1, result);

    REQUIRE(response["jsonrpc"] == "2.0");
    REQUIRE(response["id"] == 1);
    REQUIRE(response["result"]["data"] == "test");
    REQUIRE(!response.contains("error"));
  }

  SECTION("Format result with string id")
  {
    nlohmann::json result = {{"status", "ok"}};
    auto response = JSONRPCHandler::formatSuccess("req-123", result);

    REQUIRE(response["id"] == "req-123");
    REQUIRE(response["result"]["status"] == "ok");
  }
}

TEST_CASE("JSONRPCHandler - Format error response", "[json-rpc]")
{
  SECTION("Format parse error")
  {
    auto response = JSONRPCHandler::formatError(nullptr, JSONRPCHandler::PARSE_ERROR, "Parse error", "Invalid JSON");

    REQUIRE(response["jsonrpc"] == "2.0");
    REQUIRE(response["id"] == nullptr);
    REQUIRE(response["error"]["code"] == JSONRPCHandler::PARSE_ERROR);
    REQUIRE(response["error"]["message"] == "Parse error");
    REQUIRE(response["error"]["data"] == "Invalid JSON");
  }

  SECTION("Format method not found error")
  {
    auto response = JSONRPCHandler::formatError(1, JSONRPCHandler::METHOD_NOT_FOUND, "Method not found");

    REQUIRE(response["id"] == 1);
    REQUIRE(response["error"]["code"] == JSONRPCHandler::METHOD_NOT_FOUND);
    REQUIRE(!response["error"].contains("data"));
  }

  SECTION("Error response should not contain result field")
  {
    auto response = JSONRPCHandler::formatError(1, JSONRPCHandler::INVALID_REQUEST, "Invalid request");

    REQUIRE(!response.contains("result"));
    REQUIRE(response.contains("error"));
  }
}

TEST_CASE("JSONRPCHandler - Error codes", "[json-rpc]")
{
  SECTION("Standard JSON-RPC 2.0 error codes")
  {
    REQUIRE(JSONRPCHandler::PARSE_ERROR == -32700);
    REQUIRE(JSONRPCHandler::INVALID_REQUEST == -32600);
    REQUIRE(JSONRPCHandler::METHOD_NOT_FOUND == -32601);
    REQUIRE(JSONRPCHandler::INVALID_PARAMS == -32602);
    REQUIRE(JSONRPCHandler::INTERNAL_ERROR == -32603);
  }
}
