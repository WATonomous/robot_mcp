cmake_minimum_required(VERSION 3.22)
project(robot_mcp_geometry_msg_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-Wl,-no-undefined)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_mcp_msg_pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)

set(include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Plugin library (will be populated in Phase 3)
# add_library(${PROJECT_NAME} SHARED
#   src/plugins/pose_plugin.cpp
#   # More plugin implementations...
# )
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${include_dir}>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(${PROJECT_NAME}
#   PUBLIC
#     robot_mcp_msg_pluginlib::robot_mcp_msg_pluginlib
#   PRIVATE
#     ${geometry_msgs_TARGETS}
#     pluginlib::pluginlib
#     nlohmann_json::nlohmann_json
# )
#
# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}Targets
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
#
# install(EXPORT ${PROJECT_NAME}Targets
#   NAMESPACE ${PROJECT_NAME}::
#   DESTINATION share/${PROJECT_NAME}/cmake
# )

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(robot_mcp_msg_pluginlib plugins.xml)

# ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
