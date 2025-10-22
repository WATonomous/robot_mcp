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

#include "robot_mcp_server/mcp_http_server/auth_middleware.hpp"

#include <httplib.h>

#include <string>

namespace robot_mcp::http
{

AuthMiddleware::AuthMiddleware(const std::optional<std::string> & api_key)
: api_key_(api_key)
{}

bool AuthMiddleware::validate(const httplib::Request & req) const
{
  // If auth is disabled, allow all requests
  if (!api_key_.has_value()) {
    return true;
  }

  // Check for Authorization header
  if (!req.has_header("Authorization")) {
    return false;
  }

  std::string auth_header = req.get_header_value("Authorization");

  // Expected format: "Bearer <api_key>"
  const std::string bearer_prefix = "Bearer ";
  if (auth_header.size() < bearer_prefix.size() || auth_header.substr(0, bearer_prefix.size()) != bearer_prefix) {
    return false;
  }

  // Extract token
  std::string token = auth_header.substr(bearer_prefix.size());

  // Compare with configured API key
  return token == api_key_.value();
}

}  // namespace robot_mcp::http
