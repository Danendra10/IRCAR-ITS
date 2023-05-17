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

typedef Pose3D CarPose, *CarPosePtr;
// array of lane on pose2d
typedef Pose2D Lane, *LanePtr;

#endif