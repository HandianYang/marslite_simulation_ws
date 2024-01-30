#include "marslite_shared_control/static_virtual_zone.h"

#include <math.h>

namespace marslite_shared_control {

StaticVirtualZone::StaticVirtualZone(void)
{
    type_ = SVZ;
    fieldsPublisher_ = nh_.advertise<sensor_msgs::LaserScan>("/marslite_shared_control/SVZ", 1);
}

void StaticVirtualZone::calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr)
{
    // ROS_INFO_STREAM_ONCE("fieldSize_: " << fieldSize_);
    ax_ = ka_ * robotTwist_.linear.x;
    by_ = kb_ * robotTwist_.linear.x;
    for (uint i = 0; i < fieldSize_; ++i)
        fields_.ranges[i] = (float) pow((pow(ax_*cos(thetaArr_[i]), 2) + pow(by_*sin(thetaArr_[i]), 2)), 0.5);
}

} // namespace marslite_shared_control



int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_virtual_zone");
    std::shared_ptr<marslite_shared_control::StaticVirtualZone> SVZ
         = std::make_shared<marslite_shared_control::StaticVirtualZone>();

    // while (ros::ok()) {
    //     ros::Rate(60).sleep();
    // }
    ros::spin();
    return 0;
}