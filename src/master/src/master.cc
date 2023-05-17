#include "master/master.hh"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    sub_car_pose = NH.subscribe("car_pose", 1, CllbckSubCarPose);
    sub_lines = NH.subscribe("lines", 1, CllbckSubLaneVector);

    tim_60_hz = NH.createTimer(ros::Duration(1.0 / 60.0), CllbckTim60Hz);

    MTS.spin();
    return 0;
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    return;
}