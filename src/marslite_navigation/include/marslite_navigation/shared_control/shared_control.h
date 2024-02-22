#ifndef SHARED_CONTROL_H
#define SHARED_CONTROL_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <mutex>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/server.h>
#include "marslite_navigation/SharedControlConfig.h"

#include "marslite_properties/Properties.h"
#include "marslite_navigation/shared_control/static_virtual_zone.h"
#include "marslite_navigation/shared_control/deformable_virtual_zone.h"


namespace marslite_navigation {

namespace shared_control {

class SharedControl {
public:;
    explicit SharedControl(const ros::NodeHandle& nh = ros::NodeHandle());
    bool run();

    inline float getIntrusionRatio(void) const noexcept { return this->intrusionRatio_; }
    inline float getAverageAngle(void) const noexcept { return this->averageObstableAngle_; }

    // inline Velocity getLinearController(void) const noexcept { return this->linearController_; }
    // inline Velocity getAngularController(void) const noexcept { return this->angularController_; }
    // inline float getAllocationWeight(void) const noexcept { return this->allocationWeight_; }

private:
    // ROS related
    ros::NodeHandle nh_;
    ros::Publisher assistiveInputPublisher_;
    ros::Publisher sharedControllerPublisher_;
    ros::Subscriber amclPoseSubscriber_;
    ros::Subscriber userInputSubscriber_;
    ros::Rate publishRate_;

    // metrics
    float intrusionRatio_;
    float averageObstableAngle_;
    float averageObstableAngleDiff_;
    float allocationWeight_;

    // proportional gains
    float kx_, ky_, kw_;

    // velocity controller
    geometry_msgs::Twist userInput_;
    geometry_msgs::Twist assistiveInput_;
    geometry_msgs::Twist sharedController_;

    // static virtual zone (SVZ)
    std::shared_ptr<StaticVirtualZone> SVZPtr_;
    // deformable virtual zone (DVZ)
    std::shared_ptr<DeformableVirtualZone> DVZPtr_;

    // AMCL pose
    void amclPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPosePtr);
    marslite::move_base::Pose robotPose_;
    std::mutex amclPoseMutex_;

    // dynamic reconfigure
    void reconfigureCB(marslite_navigation::SharedControlConfig& config, uint32_t level);
    dynamic_reconfigure::Server<marslite_navigation::SharedControlConfig>* server_;
    dynamic_reconfigure::Server<marslite_navigation::SharedControlConfig>::CallbackType f_;

    // 
    void userInputCB(const geometry_msgs::TwistConstPtr& userInputPtr);

    void calculateAssistiveInput();
    void calculateAllocationWeight();
};

} // namespace shared_control

} // namespace marslite_navigation

#endif  // #define SHARED_CONTROL_H