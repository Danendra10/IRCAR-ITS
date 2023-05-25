#include "pid/pid.h"
#include "potential_field/potential_field.hh"

typedef struct Velocity_t
{
    float linear;
    float angular;
};

Velocity_t motion_return;

void ResetVel(Velocity_t *vel_ret = &motion_return);
void ManualMotion(float linear_vel, float angular_vel, float target_x, float target_y, Velocity_t *vel_ret = &motion_return);