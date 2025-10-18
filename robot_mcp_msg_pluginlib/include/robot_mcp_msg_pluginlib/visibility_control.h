// Copyright 2025 WATonomous
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

#ifndef ROBOT_MCP_MSG_PLUGINLIB__VISIBILITY_CONTROL_H_
#define ROBOT_MCP_MSG_PLUGINLIB__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_MCP_MSG_PLUGINLIB_EXPORT __attribute__ ((dllexport))
    #define ROBOT_MCP_MSG_PLUGINLIB_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_MCP_MSG_PLUGINLIB_EXPORT __declspec(dllexport)
    #define ROBOT_MCP_MSG_PLUGINLIB_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_MCP_MSG_PLUGINLIB_BUILDING_DLL
    #define ROBOT_MCP_MSG_PLUGINLIB_PUBLIC ROBOT_MCP_MSG_PLUGINLIB_EXPORT
  #else
    #define ROBOT_MCP_MSG_PLUGINLIB_PUBLIC ROBOT_MCP_MSG_PLUGINLIB_IMPORT
  #endif
  #define ROBOT_MCP_MSG_PLUGINLIB_PUBLIC_TYPE ROBOT_MCP_MSG_PLUGINLIB_PUBLIC
  #define ROBOT_MCP_MSG_PLUGINLIB_LOCAL
#else
  #define ROBOT_MCP_MSG_PLUGINLIB_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_MCP_MSG_PLUGINLIB_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_MCP_MSG_PLUGINLIB_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_MCP_MSG_PLUGINLIB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_MCP_MSG_PLUGINLIB_PUBLIC
    #define ROBOT_MCP_MSG_PLUGINLIB_LOCAL
  #endif
  #define ROBOT_MCP_MSG_PLUGINLIB_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MCP_MSG_PLUGINLIB__VISIBILITY_CONTROL_H_
