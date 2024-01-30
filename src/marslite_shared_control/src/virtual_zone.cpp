#include "marslite_shared_control/virtual_zone.h"

namespace marslite_shared_control {

VirtualZone::VirtualZone(const ros::NodeHandle& nh) : nh_(nh), firstAssign_(true)
{
    server_ = new dynamic_reconfigure::Server<VirtualZoneConfig>(ros::NodeHandle("~/"+ros::this_node::getName()));
    f_ = boost::bind(&VirtualZone::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);

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

        fieldSize_ = (size_t) ((fields_.angle_max - fields_.angle_min) / fields_.angle_increment);
        fields_.ranges = std::vector<float> (fieldSize_, 0);
        thetaArr_ = std::vector<float> (fieldSize_);
        for (uint i = 0; i < fieldSize_; ++i)
            thetaArr_[i] = fields_.angle_min + fields_.angle_increment * i;

        firstAssign_ = false;     
    }

    fields_.header.stamp = ros::Time::now();
    if (type_ == SVZ)
        calculateFields();
    else if (type_ == DVZ)
        calculateFields(laserPtr);
    
    fieldsPublisher_.publish(fields_);
}

void VirtualZone::robotTwistCB(const geometry_msgs::TwistConstPtr& twistPtr)
{
    robotTwist_.linear  = twistPtr->linear;
    robotTwist_.angular = twistPtr->angular;
}

void VirtualZone::reconfigureCB(marslite_shared_control::VirtualZoneConfig &config, uint32_t level)
{
    ka_ = config.ka;
    kb_ = config.kb;
}

} // namespace marslite_shared_control