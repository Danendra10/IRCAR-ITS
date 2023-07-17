#include "motion/motion.hh"
#include <cstdio>

void ResetVel(Velocity_t *vel_ret)
{
    vel_ret->angular = 0;
    vel_ret->linear = 0;
}

void ManualMotion(float linear_vel, float angular_vel, float target_x, float target_y, Velocity_t *vel_ret)
{
    static PID_t pid_linear;
    static PID_t pid_angular;

    PIDInit(&pid_linear, 0.5, 0.0, 0.0);
    PIDInit(&pid_angular, 0.5, 0.0, 0.0);

    float error_x = target_x - 400;
    float error_y = target_y - 800;

    float error_theta = atan2(error_y, error_x);

    float error_distance = sqrt(pow(error_x, 2) + pow(error_y, 2));

    float error_theta_deg = RAD2DEG(error_theta);

    float output_linear = PIDCalculate(&pid_linear, error_distance, linear_vel);
    float output_angular = PIDCalculate(&pid_angular, error_theta_deg, angular_vel);

    vel_ret->linear = output_linear;
    vel_ret->angular = output_angular;
}

void MotionControl(float linear_vel, float angular_vel, Velocity_t *vel_ret)
{
    static PID_t linear_pid;
    static PID_t angular_pid;

    linear_negative = false;
    angular_negative = false;

    float output_linear, output_angular;

    if (linear_vel < 0)
    {
        linear_negative = true;
        linear_vel *= -1.0;
    }
    if (angular_vel < 0)
    {
        angular_negative = true;
        angular_vel *= -1.0;
    }

    PIDInit(&linear_pid, pid_linear_const);
    PIDInit(&angular_pid, pid_angular_const);

    float error_vel_linear = linear_vel - vel_ret->linear;
    float error_vel_angular = angular_vel - vel_ret->angular;

    output_linear = PIDCalculate(&linear_pid, error_vel_linear, linear_vel);
    output_angular = PIDCalculate(&angular_pid, error_vel_angular, angular_vel);

    vel_ret->linear = output_linear;
    vel_ret->angular = output_angular;
}

void AngularControl(float angular_error, float angular_vel, Velocity_t *vel_ret)
{
    static PID_t angular_pid;

    angular_negative = false;

    if (angular_vel < 0)
    {
        angular_negative = true;
        angular_vel *= -1.0;
    }

    PIDInit(&angular_pid, pid_angular_const);

    float output_angular = PIDCalculate(&angular_pid, angular_error, angular_vel);

    vel_ret->angular = output_angular;
}