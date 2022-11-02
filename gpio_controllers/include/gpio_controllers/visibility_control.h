// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef GPIO_CONTROLLERS__VISIBILITY_CONTROL_H_
#define GPIO_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GPIO_COMMAND_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define GPIO_COMMAND_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define GPIO_COMMAND_CONTROLLER_EXPORT __declspec(dllexport)
    #define GPIO_COMMAND_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef GPIO_COMMAND_CONTROLLER_BUILDING_LIBRARY
    #define GPIO_COMMAND_CONTROLLER_PUBLIC GPIO_COMMAND_CONTROLLER_EXPORT
  #else
    #define GPIO_COMMAND_CONTROLLER_PUBLIC GPIO_COMMAND_CONTROLLER_IMPORT
  #endif
  #define GPIO_COMMAND_CONTROLLER_PUBLIC_TYPE GPIO_COMMAND_CONTROLLER_PUBLIC
  #define GPIO_COMMAND_CONTROLLER_LOCAL
#else
  #define GPIO_COMMAND_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define GPIO_COMMAND_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define GPIO_COMMAND_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define GPIO_COMMAND_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GPIO_COMMAND_CONTROLLER_PUBLIC
    #define GPIO_COMMAND_CONTROLLER_LOCAL
  #endif
  #define GPIO_COMMAND_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // GPIO_CONTROLLERS__VISIBILITY_CONTROL_H_
