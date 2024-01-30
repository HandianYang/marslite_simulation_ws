#include "marslite_shared_control/deformable_virtual_zone.h"

#include <math.h>

namespace marslite_shared_control {

DeformableVirtualZone::DeformableVirtualZone(void)
{
    type_ = DVZ;
    fieldsPublisher_ = nh_.advertise<sensor_msgs::LaserScan>("/marslite_shared_control/DVZ", 1);
}

void DeformableVirtualZone::calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr)
{
    // ROS_INFO_STREAM_ONCE("fieldSize_: " << fieldSize_);
    ax_ = ka_ * robotTwist_.linear.x;
    by_ = kb_ * robotTwist_.linear.x;
    for (uint i = 0; i < fieldSize_; ++i) {
        fields_.ranges[i] = (float) pow((pow(ax_*cos(thetaArr_[i]), 2) + pow(by_*sin(thetaArr_[i]), 2)), 0.5);
        if (fields_.ranges[i] > laserPtr->ranges[i])
            fields_.ranges[i] = laserPtr->ranges[i];
    }
}

} // namespace marslite_shared_control



int main(int argc, char** argv)
{
    ros::init(argc, argv, "deformable_virtual_zone");
    std::shared_ptr<marslite_shared_control::DeformableVirtualZone> DVZ
         = std::make_shared<marslite_shared_control::DeformableVirtualZone>();

    // while (ros::ok()) {
    //     ros::Rate(60).sleep();
    // }
    ros::spin();
    return 0;
}