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

#ifndef MOTION_TYPE_HPP
#define MOTION_TYPE_HPP


namespace MotionType
{   // Motion Primitives
    static constexpr uint8_t LINEAR_JOINT = 10; // changed to 10 because 0 is default value
    static constexpr uint8_t LINEAR_CARTESIAN = 50;
    static constexpr uint8_t CIRCULAR_CARTESIAN = 51;

    // Helper types
    static constexpr uint8_t STOP_MOTION = 66;  // added to stop motion
    static constexpr uint8_t MOTION_SEQUENCE_START = 100;  // added to indicate motion sequence instead of executing single primitives
    static constexpr uint8_t MOTION_SEQUENCE_END = 101;  // added to indicate end of motion sequence
}

#endif // MOTION_TYPE_HPP
