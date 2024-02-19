#ifndef APATIVE_CONTROLLER_H_
#define APATIVE_CONTROLLER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <mutex>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/server.h>
#include "marslite_shared_control/AdaptiveControllerConfig.h"

#include "marslite_properties/Properties.h"
#include "marslite_shared_control/static_virtual_zone.h"
#include "marslite_shared_control/deformable_virtual_zone.h"


namespace marslite_shared_control {

class AdaptiveController {
public:;
    explicit AdaptiveController(const ros::NodeHandle& nh = ros::NodeHandle());
    void calculateController();

    inline float getIntrusionRatio(void) const { return this->intrusionRatio_; }
    inline float getAverageAngle(void) const { return this->averageObstableAngle_; } 

private:
    // ROS related
    ros::NodeHandle nh_;
    ros::Publisher controllerPublisher_;
    ros::Subscriber amclPoseSubscriber_;

    float intrusionRatio_;
    float averageObstableAngle_;
    float averageObstableAngleDiff_;

    float kx_, ky_, kw_;

    marslite::move_base::Velocity linearController_;
    marslite::move_base::Velocity angularController_;

    std::shared_ptr<marslite_shared_control::StaticVirtualZone> SVZPtr_;
    std::shared_ptr<marslite_shared_control::DeformableVirtualZone> DVZPtr_;

    // AMCL pose
    void amclPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPosePtr);
    marslite::move_base::Pose robotPose_;
    std::mutex amclPoseMutex_;

    // dynamic reconfigure
    void reconfigureCB(marslite_shared_control::AdaptiveControllerConfig& config, uint32_t level);
    dynamic_reconfigure::Server<marslite_shared_control::AdaptiveControllerConfig>* server_;
    dynamic_reconfigure::Server<marslite_shared_control::AdaptiveControllerConfig>::CallbackType f_;
};

} // namespace marslite_shared_control

#endif  // #define APATIVE_CONTROLLER_H_