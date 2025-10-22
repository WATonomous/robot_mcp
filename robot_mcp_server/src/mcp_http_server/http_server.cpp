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

#include "robot_mcp_server/mcp_http_server/http_server.hpp"

#include <httplib.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "robot_mcp_server/mcp_http_server/auth_middleware.hpp"
#include "robot_mcp_server/mcp_http_server/json_rpc_handler.hpp"

namespace robot_mcp::http
{

HTTPServer::HTTPServer(rclcpp::Logger logger)
: logger_(logger)
{
  RCLCPP_DEBUG(logger_, "HTTPServer created");
}

HTTPServer::~HTTPServer()
{
  if (running_.load()) {
    RCLCPP_WARN(logger_, "HTTPServer destroyed while running - stopping");
    stop();
  }
  RCLCPP_DEBUG(logger_, "HTTPServer destroyed");
}

void HTTPServer::start(const config::ServerConfig & config, RequestHandler handler)
{
  if (running_.load()) {
    throw std::runtime_error("HTTPServer already running");
  }

  if (!handler) {
    throw std::runtime_error("Request handler cannot be null");
  }

  // Store config and handler
  host_ = config.host;
  port_ = config.port;
  enable_cors_ = config.enable_cors;
  request_handler_ = handler;

  // Create components
  server_ = std::make_unique<httplib::Server>();
  auth_ = std::make_unique<AuthMiddleware>(config.api_key);
  json_rpc_ = std::make_unique<JSONRPCHandler>();

  // Configure server
  server_->new_task_queue = [config] {
    return new httplib::ThreadPool(config.thread_pool_size);
  };

  // Register POST /mcp endpoint
  server_->Post("/mcp", [this](const httplib::Request & req, httplib::Response & res) {
    this->handleMCPEndpoint(req, res);
  });

  // Start server in background thread
  running_.store(true);
  server_thread_ = std::thread(&HTTPServer::serverThreadFunc, this);

  RCLCPP_INFO(logger_, "HTTP server started on %s:%d", host_.c_str(), port_);
  if (auth_->isEnabled()) {
    RCLCPP_INFO(logger_, "Authentication: ENABLED");
  } else {
    RCLCPP_WARN(logger_, "Authentication: DISABLED (not recommended for production)");
  }
}

void HTTPServer::stop()
{
  if (!running_.load()) {
    RCLCPP_DEBUG(logger_, "HTTPServer already stopped");
    return;
  }

  RCLCPP_INFO(logger_, "Stopping HTTP server...");

  // Signal server to stop
  running_.store(false);
  if (server_) {
    server_->stop();
  }

  // Wait for server thread to finish
  if (server_thread_.joinable()) {
    server_thread_.join();
  }

  // Clean up components
  server_.reset();
  auth_.reset();
  json_rpc_.reset();
  request_handler_ = nullptr;

  RCLCPP_INFO(logger_, "HTTP server stopped");
}

void HTTPServer::handleMCPEndpoint(const httplib::Request & req, httplib::Response & res)
{
  // Set CORS headers if enabled
  if (enable_cors_) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization");
  }

  // Handle preflight OPTIONS request
  if (req.method == "OPTIONS") {
    res.status = 204;
    return;
  }

  // 1. Authenticate request
  if (!auth_->validate(req)) {
    RCLCPP_WARN(logger_, "Authentication failed for request from %s", req.remote_addr.c_str());
    res.status = 401;
    res.set_content(
      R"({"error": "Unauthorized - valid API key required"})",
      "application/json");
    return;
  }

  // 2. Parse JSON-RPC request
  nlohmann::json json_request;
  try {
    json_request = JSONRPCHandler::parse(req.body);
  } catch (const nlohmann::json::parse_error & e) {
    RCLCPP_WARN(logger_, "JSON parse error: %s", e.what());
    auto error_response = JSONRPCHandler::formatError(
      nullptr,
      JSONRPCHandler::PARSE_ERROR,
      "Parse error: Invalid JSON",
      e.what());
    res.status = 400;
    res.set_content(error_response.dump(), "application/json");
    return;
  }

  // 3. Validate JSON-RPC structure
  if (!JSONRPCHandler::validate(json_request)) {
    RCLCPP_WARN(logger_, "Invalid JSON-RPC request structure");
    auto error_response = JSONRPCHandler::formatError(
      json_request.contains("id") ? json_request["id"] : nullptr,
      JSONRPCHandler::INVALID_REQUEST,
      "Invalid Request: Missing required JSON-RPC fields");
    res.status = 400;
    res.set_content(error_response.dump(), "application/json");
    return;
  }

  // Extract request ID for response
  nlohmann::json request_id = json_request["id"];

  // 4. Call request handler
  nlohmann::json response;
  try {
    nlohmann::json result = request_handler_(json_request);
    response = JSONRPCHandler::formatSuccess(request_id, result);
    res.status = 200;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Request handler error: %s", e.what());
    response = JSONRPCHandler::formatError(
      request_id,
      JSONRPCHandler::INTERNAL_ERROR,
      "Internal error",
      e.what());
    res.status = 500;
  }

  // 5. Send response
  res.set_content(response.dump(), "application/json");

  RCLCPP_DEBUG(
    logger_,
    "Handled request: method=%s, id=%s",
    json_request["method"].get<std::string>().c_str(),
    request_id.dump().c_str());
}

void HTTPServer::serverThreadFunc()
{
  RCLCPP_DEBUG(logger_, "Server thread starting on %s:%d", host_.c_str(), port_);

  // This blocks until server is stopped
  bool success = server_->listen(host_.c_str(), port_);

  if (!success && running_.load()) {
    RCLCPP_ERROR(
      logger_,
      "Server failed to listen on %s:%d - port may be in use",
      host_.c_str(), port_);
  }

  RCLCPP_DEBUG(logger_, "Server thread exiting");
}

}  // namespace robot_mcp::http
