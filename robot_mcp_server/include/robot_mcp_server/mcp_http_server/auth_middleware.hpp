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

#ifndef ROBOT_MCP_HTTP_SERVER__AUTH_MIDDLEWARE_HPP_
#define ROBOT_MCP_HTTP_SERVER__AUTH_MIDDLEWARE_HPP_

#include <optional>
#include <string>

// Forward declare httplib types to avoid including the header here
namespace httplib
{
struct Request;
}

namespace robot_mcp::http
{

/**
 * @brief Authentication middleware for HTTP requests
 *
 * Validates API key from request headers. Auth is optional - if no API key
 * is configured, all requests are allowed.
 */
class AuthMiddleware
{
public:
  /**
   * @brief Constructor
   * @param api_key Optional API key to validate against
   */
  explicit AuthMiddleware(const std::optional<std::string> & api_key);

  /**
   * @brief Validate request authentication
   *
   * Checks the Authorization header for valid API key.
   * Format: "Authorization: Bearer <api_key>"
   *
   * @param req HTTP request
   * @return true if authenticated (or auth disabled), false otherwise
   */
  bool validate(const httplib::Request & req) const;

  /**
   * @brief Check if authentication is enabled
   * @return true if API key is configured, false otherwise
   */
  bool isEnabled() const
  {
    return api_key_.has_value();
  }

private:
  std::optional<std::string> api_key_;
};

}  // namespace robot_mcp::http

#endif  // ROBOT_MCP_HTTP_SERVER__AUTH_MIDDLEWARE_HPP_
