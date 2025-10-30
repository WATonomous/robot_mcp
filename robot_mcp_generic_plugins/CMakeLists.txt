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
project(robot_mcp_generic_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-Wl,-no-undefined)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(robot_mcp_msg_pluginlib REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

set(include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Detect ROS distro for conditional compilation
if($ENV{ROS_DISTRO} STREQUAL "humble")
  set(HAS_GENERIC_CLIENT FALSE)
  message(STATUS "Building for ROS2 Humble - GenericClient not available")
elseif($ENV{ROS_DISTRO} STREQUAL "jazzy" OR $ENV{ROS_DISTRO} STREQUAL "rolling")
  set(HAS_GENERIC_CLIENT TRUE)
  message(STATUS "Building for ROS2 ${ROS_DISTRO} - GenericClient available")
else()
  # Default: assume recent distro has GenericClient
  set(HAS_GENERIC_CLIENT TRUE)
  message(WARNING "Unknown ROS distro: $ENV{ROS_DISTRO} - assuming GenericClient is available")
endif()

# Plugin library - always include topic plugin and conversion utilities
set(PLUGIN_SOURCES
  src/generic_topic_plugin.cpp
  src/message_conversion.cpp
)

# Add service plugin only if GenericClient is available (Jazzy/Rolling+)
if(HAS_GENERIC_CLIENT)
  list(APPEND PLUGIN_SOURCES src/generic_service_plugin.cpp)
  message(STATUS "Including GenericServicePlugin (GenericClient available in ${ROS_DISTRO})")
else()
  message(STATUS "Excluding GenericServicePlugin (GenericClient not available in ${ROS_DISTRO})")
endif()

# NOTE: Generic action plugin is NOT implemented
# Generic action client API doesn't exist even in Jazzy/Rolling
# Will need custom implementation or typed action clients in the future

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${include_dir}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    robot_mcp_msg_pluginlib::robot_mcp_msg_pluginlib
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    pluginlib::pluginlib
    nlohmann_json::nlohmann_json
    ${rosidl_typesupport_introspection_cpp_TARGETS}
    ${rosidl_runtime_cpp_TARGETS}
)

# Export plugin library
pluginlib_export_plugin_description_file(robot_mcp_msg_pluginlib plugins.xml)

# Install
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

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
