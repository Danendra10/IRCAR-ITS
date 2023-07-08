#ifndef __MACHINESTATE_HH_
#define __MACHINESTATE_HH_

#include <stdint.h>
#include <vector>
#include <math.h>
#include <chrono>
#include <string.h>
#include <cstdio>
#include <sys/time.h>

#include "string"
#include <stdarg.h>

class MachineState
{
public:
    int16_t value;
    std::chrono::system_clock::time_point uptime_timeout;
    std::chrono::system_clock::time_point uptime_reentry;

    MachineState()
    {
        value = 0;
        uptime_timeout = std::chrono::high_resolution_clock::now();
        uptime_reentry = std::chrono::high_resolution_clock::now();
    }

    void resetUptimeTimeout()
    {
        uptime_timeout = std::chrono::high_resolution_clock::now();
    }

    void resetUptimeReentry()
    {
        uptime_reentry = std::chrono::high_resolution_clock::now();
    }

    void timeout(int16_t target_state, float period)
    {
        std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = t_now - uptime_timeout;
        if (elapsed_seconds.count() > period)
        {
            value = target_state;
            uptime_timeout = std::chrono::high_resolution_clock::now();
        }
    }

    void reentry(int16_t target_state, float period)
    {
        std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = t_now - uptime_reentry;
        if (elapsed_seconds.count() > period)
        {
            value = target_state;
        }
        uptime_reentry = std::chrono::high_resolution_clock::now();
    }
};

enum machine_states
{
    FORWARD = 1,
    BACKWARD = 2,
    TURN_LEFT = 3,
    TURN_RIGHT = 4,
    AUTONOMOUS = 5,

    // Autonomous states
    AUTONOMOUS_IDLE = 10,
    AUTONOMOUS_TURN_LEFT = 11,
    AUTONOMOUS_TURN_RIGHT = 12,
    AUTONOMOUS_TURN_LEFT_90 = 13,
    AUTONOMOUS_TURN_RIGHT_90 = 14,
    AUTONOMOUS_TURN_LEFT_180 = 15,
    AUTONOMOUS_TURN_RIGHT_180 = 16,
    AUTONOMOUS_STOP_SIGN = 17,
    AUTONOMOUS_KEEP_FORWARD = 18,
    AUTONOMOUS_START_TUNNEL = 19,
    AUTONOMOUS_END_TUNNEL = 20,
    AUTONOMOUS_DEAD_END = 21,
    AUTONOMOUS_NO_ENTRY = 22,
    AUTONOMOUS_NO_SIGN = 23,

    RESET = 0,
};
#endif