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
