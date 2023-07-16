#include "pid/pid.h"
#include "potential_field/potential_field.hh"
#include "entity/entity.hh"

typedef struct Velocity_t
{
    float linear;
    float angular;
};

Velocity_t motion_return;
bool linear_negative;
bool angular_negative;

extern PID_Const pid_linear_const;

void ResetVel(Velocity_t *vel_ret = &motion_return);
void ManualMotion(float linear_vel, float angular_vel, float target_x, float target_y, Velocity_t *vel_ret = &motion_return);
void MotionControl(float linear_vel, float angular_vel, Velocity_t *vel_ret = &motion_return);
void AngularControl(float angular_error, float angular_vel, Velocity_t *vel_ret = &motion_return);