#ifndef APATIVE_CONTROLLER_H_
#define APATIVE_CONTROLLER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <mutex>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "marslite_navigation/move_base.h"
#include "marslite_shared_control/properties.h"
#include "marslite_shared_control/static_virtual_zone.h"
#include "marslite_shared_control/deformable_virtual_zone.h"


namespace marslite_shared_control {

class AdaptiveController {
public:;
    explicit AdaptiveController(const ros::NodeHandle& nh = ros::NodeHandle());
    void calculateIntrusionRatio();
    void calculateAverageAngle();

    inline float getIntrusionRatio(void) const { return this->intrusionRatio_; }
    inline float getAverageAngle(void) const { return this->averageObstableAngle_; } 

    inline sensor_msgs::LaserScan getSVZFields(void) const { return this->SVZPtr_->getFieldsData(); }
    inline sensor_msgs::LaserScan getDVZFields(void) const { return this->DVZPtr_->getFieldsData(); }

private:
    ros::NodeHandle nh_;
    ros::Subscriber amclPoseSubscriber_;

    marslite_navigation::move_base::Pose robotPose_;
    std::mutex amclPoseMutex_;

    float intrusionRatio_;
    float averageObstableAngle_;

    std::shared_ptr<marslite_shared_control::StaticVirtualZone> SVZPtr_;
    std::shared_ptr<marslite_shared_control::DeformableVirtualZone> DVZPtr_;

    void amclPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPosePtr);
};

} // namespace marslite_shared_control

#endif  // #define APATIVE_CONTROLLER_H_