#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include "config/planner.h"

enum Result
{
    Ok,
    Error,
    Blocked
};

extern float target_position[INPUT_AXIS_COUNT];

void resetCommandProcessor();
Result processCommand(const char* command);

#endif//COMMAND_PROCESSOR_H
