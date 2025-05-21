// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#ifndef MOTION_PRIMITIVES_FORWARD_CONTROLLER__EXECUTION_STATE_HPP_
#define MOTION_PRIMITIVES_FORWARD_CONTROLLER__EXECUTION_STATE_HPP_

namespace ExecutionState
{
static constexpr uint8_t IDLE = 0;
static constexpr uint8_t EXECUTING = 1;
static constexpr uint8_t SUCCESS = 2;
static constexpr uint8_t ERROR = 3;
static constexpr uint8_t STOPPED = 4;
}  // namespace ExecutionState

#endif  // MOTION_PRIMITIVES_FORWARD_CONTROLLER__EXECUTION_STATE_HPP_
