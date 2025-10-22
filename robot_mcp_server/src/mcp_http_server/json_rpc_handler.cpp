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

#include "robot_mcp_server/mcp_http_server/json_rpc_handler.hpp"

namespace robot_mcp::http
{

nlohmann::json JSONRPCHandler::parse(const std::string & body)
{
  // This may throw nlohmann::json::parse_error
  return nlohmann::json::parse(body);
}

bool JSONRPCHandler::validate(const nlohmann::json & request)
{
  // Check required fields per JSON-RPC 2.0 spec
  if (!request.contains("jsonrpc") || !request["jsonrpc"].is_string()) {
    return false;
  }

  if (request["jsonrpc"] != "2.0") {
    return false;
  }

  if (!request.contains("method") || !request["method"].is_string()) {
    return false;
  }

  // id is required for requests (but not for notifications)
  // We'll require it for our use case
  if (!request.contains("id")) {
    return false;
  }

  // params is optional, but if present must be object or array
  if (request.contains("params")) {
    if (!request["params"].is_object() && !request["params"].is_array()) {
      return false;
    }
  }

  return true;
}

nlohmann::json JSONRPCHandler::formatSuccess(
  const nlohmann::json & id,
  const nlohmann::json & result)
{
  return {
    {"jsonrpc", "2.0"},
    {"id", id},
    {"result", result}
  };
}

nlohmann::json JSONRPCHandler::formatError(
  const nlohmann::json & id,
  int code,
  const std::string & message,
  const nlohmann::json & data)
{
  nlohmann::json error = {
    {"code", code},
    {"message", message}
  };

  if (!data.is_null()) {
    error["data"] = data;
  }

  return {
    {"jsonrpc", "2.0"},
    {"id", id},
    {"error", error}
  };
}

}  // namespace robot_mcp::http
