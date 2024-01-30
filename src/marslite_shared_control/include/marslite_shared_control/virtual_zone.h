#ifndef VIRTUAL_ZONE_H_
#define VIRTUAL_ZONE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "marslite_shared_control/VirtualZoneConfig.h"

namespace marslite_shared_control {


typedef enum {
    SVZ,
    DVZ
} VirtualZoneType;


class VirtualZone {
public:
    explicit VirtualZone(const ros::NodeHandle& nh = ros::NodeHandle());
    virtual ~VirtualZone(void) = default;
    
protected:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher  fieldsPublisher_;
    ros::Subscriber laserScanSubscriber_;
    ros::Subscriber robotTwistSubscriber_;

    // robot properties
    geometry_msgs::Twist robotTwist_;

    // virtual zone properties
    VirtualZoneType type_;
    bool firstAssign_;
    float ax_, by_;    // semi-major & semi-minor axes
    float ka_, kb_;    // proportional ratio of ax_ and by_
    
    std::vector<float> thetaArr_;
    sensor_msgs::LaserScan fields_;
    size_t fieldSize_;

    // dynamic reconfigure
    void reconfigureCB(marslite_shared_control::VirtualZoneConfig &config, uint32_t level);
    dynamic_reconfigure::Server<marslite_shared_control::VirtualZoneConfig>* server_;
    dynamic_reconfigure::Server<marslite_shared_control::VirtualZoneConfig>::CallbackType f_;

    virtual void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) = 0;

private:
    void laserScanCB(const sensor_msgs::LaserScanConstPtr& laserPtr);
    void robotTwistCB(const geometry_msgs::TwistConstPtr& twistPtr);
};


} // namespace marslite_shared_control

#endif  // #ifndef VIRTUAL_ZONE_H_