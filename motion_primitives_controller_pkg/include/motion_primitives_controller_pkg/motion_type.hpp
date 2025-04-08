#ifndef MOTION_TYPE_HPP
#define MOTION_TYPE_HPP


namespace MotionType
{
    static constexpr uint8_t LINEAR_JOINT = 10; // changed to 10 because 0 is default value
    static constexpr uint8_t LINEAR_CARTESIAN = 50;
    static constexpr uint8_t CIRCULAR_CARTESIAN = 51;
    static constexpr uint8_t STOP_MOTION = 66;  // added to stop motion
}

#endif // MOTION_TYPE_HPP
