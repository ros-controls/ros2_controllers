// Copyright 2024 FZI Forschungszentrum Informatik
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

#ifndef POSE_BROADCASTER__VISIBILITY_CONTROL_H_
#define POSE_BROADCASTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define POSE_BROADCASTER_EXPORT __attribute__((dllexport))
#define POSE_BROADCASTER_IMPORT __attribute__((dllimport))
#else
#define POSE_BROADCASTER_EXPORT __declspec(dllexport)
#define POSE_BROADCASTER_IMPORT __declspec(dllimport)
#endif
#ifdef POSE_BROADCASTER_BUILDING_DLL
#define POSE_BROADCASTER_PUBLIC POSE_BROADCASTER_EXPORT
#else
#define POSE_BROADCASTER_PUBLIC POSE_BROADCASTER_IMPORT
#endif
#define POSE_BROADCASTER_PUBLIC_TYPE POSE_BROADCASTER_PUBLIC
#define POSE_BROADCASTER_LOCAL
#else
#define POSE_BROADCASTER_EXPORT __attribute__((visibility("default")))
#define POSE_BROADCASTER_IMPORT
#if __GNUC__ >= 4
#define POSE_BROADCASTER_PUBLIC __attribute__((visibility("default")))
#define POSE_BROADCASTER_LOCAL __attribute__((visibility("hidden")))
#else
#define POSE_BROADCASTER_PUBLIC
#define POSE_BROADCASTER_LOCAL
#endif
#define POSE_BROADCASTER_PUBLIC_TYPE
#endif

#endif  // POSE_BROADCASTER__VISIBILITY_CONTROL_H_
