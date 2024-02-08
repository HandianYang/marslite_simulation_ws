#include "marslite_shared_control/virtual_zone.h"
#include "marslite_properties/Properties.h"

namespace marslite_shared_control {

VirtualZone::VirtualZone(const ros::NodeHandle& nh) : nh_(nh), firstAssign_(true)
{
    laserScanSubscriber_ = nh_.subscribe("/scan", 1, &VirtualZone::laserScanCB, this);
    robotTwistSubscriber_ = nh_.subscribe("/cmd_vel", 1, &VirtualZone::robotTwistCB, this);
}

void VirtualZone::laserScanCB(const sensor_msgs::LaserScanConstPtr& laserPtr)
{
    if (firstAssign_) {
        fields_.header.frame_id = laserPtr->header.frame_id;
        fields_.angle_max = laserPtr->angle_max;
        fields_.angle_min = laserPtr->angle_min;
        fields_.angle_increment = laserPtr->angle_increment;
        fields_.range_max = laserPtr->range_max;
        fields_.range_min = laserPtr->range_min;
        fields_.scan_time = laserPtr->scan_time;
        fields_.time_increment = laserPtr->time_increment;
        fields_.intensities = laserPtr->intensities;
        fields_.ranges = std::vector<float> (marslite::LASER_SIZE, 0);

        firstAssign_ = false;     
    }

    fields_.header.stamp = ros::Time::now();
    calculateFields(laserPtr);
    
    fieldsPublisher_.publish(fields_);
}

void VirtualZone::robotTwistCB(const geometry_msgs::TwistConstPtr& twistPtr)
{
    robotTwist_.linear  = twistPtr->linear;
    robotTwist_.angular = twistPtr->angular;
}

} // namespace marslite_shared_control