#ifndef __ENTITY_HH_
#define __ENTITY_HH_

typedef struct Pose3D_Tag
{
    float x;
    float y;
    float th;
} Pose3D, *Pose3DPtr;

typedef struct Pose2D_Tag
{
    float x;
    float y;
} Pose2D, *Pose2DPtr;

typedef struct Vector3D_Tag
{
    float x;
    float y;
    float th;
} Vector3D, *Vector3DPtr;

typedef struct Vector2D_Tag
{
    float x;
    float th;
} Vector2D, *Vector2DPtr;

typedef struct CarData_Tag
{
    float front_left_wheel_joint;
    float front_right_wheel_joint;
    float rear_left_wheel_joint;
    float rear_right_wheel_joint;

    float distance_between_wheels;
} CarData, *CarDataPtr;

typedef Pose3D CarPose, *CarPosePtr;
typedef Pose2D Lane, *LanePtr;
typedef Pose2D Obstacles, *ObstaclesPtr;
typedef Vector3D Velocity, *VelocityPtr;
typedef Vector3D Target, *TargetPtr;

#endif