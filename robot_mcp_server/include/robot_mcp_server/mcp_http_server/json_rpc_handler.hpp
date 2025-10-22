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

#ifndef ROBOT_MCP_HTTP_SERVER__JSON_RPC_HANDLER_HPP_
#define ROBOT_MCP_HTTP_SERVER__JSON_RPC_HANDLER_HPP_

#include <nlohmann/json.hpp>
#include <string>

namespace robot_mcp::http
{

/**
 * @brief Handler for JSON-RPC 2.0 protocol
 *
 * Provides utilities for parsing JSON-RPC requests and formatting responses.
 * Implements JSON-RPC 2.0 specification: https://www.jsonrpc.org/specification
 */
class JSONRPCHandler
{
public:
  /**
   * @brief Parse JSON-RPC 2.0 request from string
   *
   * @param body Request body as string
   * @return Parsed JSON object
   * @throws nlohmann::json::parse_error if parsing fails
   */
  static nlohmann::json parse(const std::string & body);

  /**
   * @brief Validate JSON-RPC request structure
   *
   * Checks for required fields: jsonrpc, method, id
   *
   * @param request Parsed JSON object
   * @return true if valid JSON-RPC request, false otherwise
   */
  static bool validate(const nlohmann::json & request);

  /**
   * @brief Format successful JSON-RPC response
   *
   * @param id Request ID (must match request)
   * @param result Result data
   * @return JSON-RPC success response
   */
  static nlohmann::json formatSuccess(
    const nlohmann::json & id,
    const nlohmann::json & result);

  /**
   * @brief Format JSON-RPC error response
   *
   * @param id Request ID (null if request couldn't be parsed)
   * @param code Error code
   * @param message Error message
   * @param data Optional additional error data
   * @return JSON-RPC error response
   */
  static nlohmann::json formatError(
    const nlohmann::json & id,
    int code,
    const std::string & message,
    const nlohmann::json & data = nullptr);

  /**
   * @brief Standard JSON-RPC error codes
   */
  enum ErrorCode
  {
    PARSE_ERROR = -32700,       ///< Invalid JSON
    INVALID_REQUEST = -32600,   ///< Invalid JSON-RPC structure
    METHOD_NOT_FOUND = -32601,  ///< Method doesn't exist
    INVALID_PARAMS = -32602,    ///< Invalid method parameters
    INTERNAL_ERROR = -32603     ///< Internal JSON-RPC error
  };
};

}  // namespace robot_mcp::http

#endif  // ROBOT_MCP_HTTP_SERVER__JSON_RPC_HANDLER_HPP_
