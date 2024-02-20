#ifndef VIRTUAL_ZONE_H_
#define VIRTUAL_ZONE_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

namespace marslite_navigation {

namespace shared_control {

class VirtualZone {
public:
    explicit VirtualZone(const ros::NodeHandle& nh = ros::NodeHandle());
    virtual ~VirtualZone(void) = default;
    inline sensor_msgs::LaserScan getFieldsData(void) const { return this->fields_; }
    
protected:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher  fieldsPublisher_;
    ros::Subscriber laserScanSubscriber_;
    ros::Subscriber robotTwistSubscriber_;

    // robot properties
    geometry_msgs::Twist robotTwist_;

    // virtual zone properties
    bool firstAssign_;
    float ax_, by_;    // semi-major & semi-minor axes
    float ka_, kb_;    // proportional ratio of ax_ and by_
    sensor_msgs::LaserScan fields_;

    virtual void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) = 0;

private:
    void laserScanCB(const sensor_msgs::LaserScanConstPtr& laserPtr);
    void robotTwistCB(const geometry_msgs::TwistConstPtr& twistPtr);
};

} // namespace shared_control

} // namespace marslite_navigation

#endif  // #ifndef VIRTUAL_ZONE_H_