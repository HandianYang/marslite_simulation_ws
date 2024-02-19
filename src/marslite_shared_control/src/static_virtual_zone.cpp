#include "marslite_shared_control/static_virtual_zone.h"
#include "marslite_properties/Properties.h"

#include <math.h>

namespace marslite_shared_control {

StaticVirtualZone::StaticVirtualZone(const ros::NodeHandle& nh) : VirtualZone(nh)
{
    server_ = new dynamic_reconfigure::Server<VirtualZoneConfig>(ros::NodeHandle("~/marslite_shared_control/SVZ"));
    f_ = boost::bind(&StaticVirtualZone::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);

    fieldsPublisher_ = nh_.advertise<sensor_msgs::LaserScan>("/marslite_shared_control/SVZ", 1);
}

void StaticVirtualZone::calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr)
{
    ax_ = ka_ * robotTwist_.linear.x;
    by_ = kb_ * robotTwist_.linear.x;
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i)
        fields_.ranges[i] = (float) pow((pow(ax_*cos(marslite::THETA[i]), 2) + pow(by_*sin(marslite::THETA[i]), 2)), 0.5);
}

void StaticVirtualZone::reconfigureCB(marslite_shared_control::VirtualZoneConfig &config, uint32_t level)
{
    ka_ = config.ka;
    kb_ = config.kb;
}

} // namespace marslite_shared_control