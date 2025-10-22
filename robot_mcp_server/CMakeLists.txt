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
cmake_minimum_required(VERSION 3.22)
project(robot_mcp_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-Wl,-no-undefined)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_mcp_msg_pluginlib REQUIRED)
find_package(bondcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

set(include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Server library
add_library(${PROJECT_NAME} SHARED
  src/robot_mcp_server_node.cpp
  src/mcp_config/config_parser.cpp
  src/mcp_http_server/auth_middleware.cpp
  src/mcp_http_server/json_rpc_handler.cpp
  src/mcp_http_server/http_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${include_dir}>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  CPPHTTPLIB_OPENSSL_SUPPORT
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    robot_mcp_msg_pluginlib::robot_mcp_msg_pluginlib
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    ${std_msgs_TARGETS}
    pluginlib::pluginlib
    bondcpp::bondcpp
    nlohmann_json::nlohmann_json
    OpenSSL::SSL
    OpenSSL::Crypto
    cpp-httplib
)

# Register as a rclcpp_components component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "robot_mcp::MCPServerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/
  DESTINATION include
)

# Install test configuration files
install(DIRECTORY test_mcp_config/
  DESTINATION share/${PROJECT_NAME}/test_mcp_config
)

if(BUILD_TESTING)
  # Find robot_mcp_test package
  find_package(robot_mcp_test REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Include the custom test function
  include(${robot_mcp_test_DIR}/add_robot_mcp_test.cmake)

  # Add lifecycle tests
  add_robot_mcp_test(test_lifecycle
    test/test_lifecycle.cpp
    LIBRARIES
      ${PROJECT_NAME}
  )

  # Add configuration parser launch tests
  add_launch_test(
    test_mcp_config/test_complete_config.py
  )

  add_launch_test(
    test_mcp_config/test_minimal_config.py
  )
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
