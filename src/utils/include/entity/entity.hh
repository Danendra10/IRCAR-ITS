#ifndef __ENTITY_HH_
#define __ENTITY_HH_

#define SIGN_DEAD_END 0
#define SIGN_END_TUNNEL 1
#define SIGN_FORWARD 2
#define SIGN_LEFT 3
#define SIGN_NO_ENTRY 4
#define SIGN_RIGHT 5
#define SIGN_START_TUNNEL 6
#define SIGN_STOP 7

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
    float dist;
    int status;
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

struct CameraParameters
{
    double horizontal_fov; // in radian
    double vertical_fov;
    int image_width;      // in pixels
    int image_height;     // in pixels
    double near_clip;     // in meters
    double far_clip;      // in meters
    double noise_mean;    // in meters
    double noise_std_dev; // in meters
    double hack_baseline; // in meters
    double distortion_k1; // radial distortion coefficient
    double distortion_k2; // radial distortion coefficient
    double distortion_k3; // radial distortion coefficient
    double distortion_t1; // tangential distortion coefficient
    double distortion_t2; // tangential distortion coefficient
    double camera_pos_x;  // in cm
    double camera_pos_y;  // in cm
    double camera_pos_z;  // in cm
    double cam_scale_x;   // in cm
    double cam_scale_y;   // in cm
};

typedef Pose3D CarPose, *CarPosePtr;
typedef Pose2D Lane, *LanePtr;
typedef Pose2D RealLane, *RealLanePtr;
typedef Pose2D Obstacles, *ObstaclesPtr;
typedef Vector3D Velocity, *VelocityPtr;
typedef Vector3D Target, *TargetPtr;

#endif