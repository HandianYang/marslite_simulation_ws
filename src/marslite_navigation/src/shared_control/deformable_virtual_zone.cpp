#include "marslite_navigation/shared_control/deformable_virtual_zone.h"
#include "marslite_properties/Properties.h"

#include <math.h>

namespace marslite_navigation {

namespace shared_control {

DeformableVirtualZone::DeformableVirtualZone(const ros::NodeHandle& nh) : VirtualZone(nh)
{
    const std::string topicName = "/marslite_navigation/DVZ";

    server_ = new dynamic_reconfigure::Server<marslite_navigation::VirtualZoneConfig>(ros::NodeHandle("~"+topicName));
    f_ = boost::bind(&DeformableVirtualZone::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);

    fieldsPublisher_ = nh_.advertise<sensor_msgs::LaserScan>(topicName, 1);
}

void DeformableVirtualZone::calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr)
{
    ax_ = ka_ * robotTwist_.linear.x;
    by_ = kb_ * robotTwist_.linear.x;
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        fields_.ranges[i] = (float) pow((pow(ax_*cos(marslite::THETA[i]), 2) + pow(by_*sin(marslite::THETA[i]), 2)), 0.5);
        if (fields_.ranges[i] > laserPtr->ranges[i])
            fields_.ranges[i] = laserPtr->ranges[i];
    }
}

void DeformableVirtualZone::reconfigureCB(marslite_navigation::VirtualZoneConfig &config, uint32_t level)
{
    ka_ = config.ka;
    kb_ = config.kb;
}

} // namespace shared_control

} // namespace marslite_navigation