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

#ifndef ROBOT_MCP_HTTP_SERVER__HTTP_SERVER_HPP_
#define ROBOT_MCP_HTTP_SERVER__HTTP_SERVER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_mcp_server/mcp_config/config_types.hpp"

// Forward declarations to avoid including httplib in header
namespace httplib {
class Server;
struct Request;
struct Response;
}  // namespace httplib

namespace robot_mcp::http
{

// Forward declarations
class AuthMiddleware;
class JSONRPCHandler;

/**
 * @brief HTTP server for MCP protocol
 *
 * Wraps cpp-httplib to provide lifecycle management and integration with
 * ROS2. Handles authentication, JSON-RPC parsing, and request routing.
 */
class HTTPServer
{
public:
  /**
   * @brief Callback type for handling MCP requests
   *
   * Takes a JSON-RPC request and returns a JSON-RPC result.
   * The callback should not throw - errors should be returned as JSON-RPC error responses.
   */
  using RequestHandler = std::function<nlohmann::json(const nlohmann::json &)>;

  /**
   * @brief Constructor
   * @param logger ROS logger for diagnostics
   */
  explicit HTTPServer(rclcpp::Logger logger);

  /**
   * @brief Destructor - ensures server is stopped
   */
  ~HTTPServer();

  // Delete copy/move constructors
  HTTPServer(const HTTPServer &) = delete;
  HTTPServer & operator=(const HTTPServer &) = delete;
  HTTPServer(HTTPServer &&) = delete;
  HTTPServer & operator=(HTTPServer &&) = delete;

  /**
   * @brief Start HTTP server
   *
   * Spawns background thread to run server. Non-blocking.
   *
   * @param config Server configuration
   * @param handler Callback for handling MCP requests
   * @throws std::runtime_error if server fails to start
   */
  void start(const config::ServerConfig & config, RequestHandler handler);

  /**
   * @brief Stop HTTP server
   *
   * Stops server and joins background thread. Blocks until server is stopped.
   */
  void stop();

  /**
   * @brief Check if server is running
   * @return true if server is running, false otherwise
   */
  bool isRunning() const { return running_.load(); }

private:
  /**
   * @brief Handle POST /mcp endpoint
   *
   * Main entry point for MCP requests. Validates auth, parses JSON-RPC,
   * calls handler, and formats response.
   */
  void handleMCPEndpoint(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Server thread function
   *
   * Runs in background thread, blocks on server.listen()
   */
  void serverThreadFunc();

  // Components
  std::unique_ptr<httplib::Server> server_;
  std::unique_ptr<AuthMiddleware> auth_;
  std::unique_ptr<JSONRPCHandler> json_rpc_;

  // Request handler callback
  RequestHandler request_handler_;

  // Server thread management
  std::thread server_thread_;
  std::atomic<bool> running_{false};

  // Config snapshot
  std::string host_;
  int port_;
  bool enable_cors_;

  // Logger
  rclcpp::Logger logger_;
};

}  // namespace robot_mcp::http

#endif  // ROBOT_MCP_HTTP_SERVER__HTTP_SERVER_HPP_
