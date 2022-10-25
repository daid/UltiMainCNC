#include "commandProcessor.h"
#include "motion/planner.h"
#include "motion/stepper.h"

#include <avr/pgmspace.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

static void skipWhitespace(char* &s) { while(isspace(*s)) s++; }
static void skipWhitespace(const char* &s) { while(isspace(*s)) s++; }

float target_position[INPUT_AXIS_COUNT];
static float target_feedrate = 10;
static float target_position_offset[INPUT_AXIS_COUNT];
static bool relative_mode = false;

void resetCommandProcessor()
{
    for(uint8_t n=0; n<INPUT_AXIS_COUNT; n++)
        target_position_offset[n] = 0;
    target_feedrate = 10;
}

Result processCommand(const char* command)
{
    skipWhitespace(command);
    if (command[0] == '\0')
        return Ok;
    if (command[0] == 'G')
    {
        uint16_t g_code = atoi(&command[1]);
        switch(g_code)
        {
        case 0:
        case 1:{
            if (relative_mode)
            {
                if (strchr(command, 'X'))
                    target_position[0] += strtod(strchr(command, 'X') + 1, nullptr);
                if (strchr(command, 'Y'))
                    target_position[1] += strtod(strchr(command, 'Y') + 1, nullptr);
                if (strchr(command, 'Z'))
                    target_position[2] += strtod(strchr(command, 'Z') + 1, nullptr);
            }
            else
            {
                if (strchr(command, 'X'))
                    target_position[0] = strtod(strchr(command, 'X') + 1, nullptr) + target_position_offset[0];
                if (strchr(command, 'Y'))
                    target_position[1] = strtod(strchr(command, 'Y') + 1, nullptr) + target_position_offset[1];
                if (strchr(command, 'Z'))
                    target_position[2] = strtod(strchr(command, 'Z') + 1, nullptr) + target_position_offset[2];
            }
            if (strchr(command, 'F'))
                target_feedrate = strtod(strchr(command, 'F') + 1, nullptr) / 60.0;
            if (!planner_buffer_line(target_position, target_feedrate))
                return Blocked;
            return Ok;
            }
        case 4:
            if (blocks_queued())
                return Blocked;
            //TODO: Return Blocked until delay has passed.
            return Ok;
        case 90: //Relative mode disable
            relative_mode = false;
            return Ok;
        case 91: //Relative mode enabled
            relative_mode = true;
            return Ok;
        case 92:
            {
            if (strchr(command, 'X'))
                target_position_offset[0] = target_position[0] - strtod(strchr(command, 'X') + 1, nullptr);
            if (strchr(command, 'Y'))
                target_position_offset[1] = target_position[1] - strtod(strchr(command, 'Y') + 1, nullptr);
            if (strchr(command, 'Z'))
                target_position_offset[2] = target_position[2] - strtod(strchr(command, 'Z') + 1, nullptr);
            return Ok;
            }
        }
        //printf("Unknown gcode number: %s\n", command);
        return Error;
    }
    if (command[0] == 'M')
    {
        uint16_t m_code = atoi(&command[1]);
        switch(m_code)
        {
        case 18://disable motors
        case 84://disable motors
            if (blocks_queued())
                return Blocked;
            st_disable_motors();
            return Ok;
        case 201://acceleration limit per axis
            if (strchr(command, 'X'))
                max_acceleration_units_per_sq_second[0] = strtod(strchr(command, 'X') + 1, nullptr);
            if (strchr(command, 'Y'))
                max_acceleration_units_per_sq_second[1] = strtod(strchr(command, 'X') + 1, nullptr);
            if (strchr(command, 'Z'))
                max_acceleration_units_per_sq_second[2] = strtod(strchr(command, 'X') + 1, nullptr);
            reset_acceleration_rates();
            return Ok;
        case 204:
            if (strchr(command, 'S'))
                acceleration = strtod(strchr(command, 'S') + 1, nullptr);
            return Ok;
        case 205:
            if (strchr(command, 'S'))
                minimumfeedrate = strtod(strchr(command, 'S') + 1, nullptr);
            if(strchr(command, 'X'))
                max_xy_jerk = strtod(strchr(command, 'X') + 1, nullptr);
            if(strchr(command, 'Z'))
                max_z_jerk = strtod(strchr(command, 'Z') + 1, nullptr);
            return Ok;
        case 400://sync
            if (blocks_queued())
                return Blocked;
            return Ok;
        
        case 105://status request
            return Ok;
        
        case 110://Part of protocol, used to reset line numbering
            return Ok;
        }
        //printf("Unknown mcode number: %s\n", command);
        return Error;
    }
    //printf("Unable to parse command: %s\n", command);
    return Error;
}
