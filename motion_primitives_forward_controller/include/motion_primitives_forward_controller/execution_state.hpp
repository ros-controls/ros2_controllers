#ifndef EXECUTION_STATE_HPP
#define EXECUTION_STATE_HPP


namespace ExecutionState
{
    static constexpr uint8_t IDLE = 0;
    static constexpr uint8_t EXECUTING = 1;
    static constexpr uint8_t SUCCESS = 2;
    static constexpr uint8_t ERROR = 3;
}

#endif // EXECUTION_STATE_HPP
