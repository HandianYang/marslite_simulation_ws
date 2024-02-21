#ifndef _TELEOP_INTERFACE_H_
#define _TELEOP_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "marslite_properties/Properties.h"
#include "marslite_navigation/shared_control/adaptive_controller.h"

using marslite_navigation::shared_control::AdaptiveController;

namespace marslite_navigation {

namespace teleoperation {

class TeleopInterface {
public:
    explicit TeleopInterface(const ros::NodeHandle& nh = ros::NodeHandle());
    virtual ~TeleopInterface(void) = default;
    virtual bool run(void) = 0;

protected:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher robotTwistPublisher_;
    ros::Rate publishRate_;

    // parsed parameters (yaml)
    marslite::move_base::Velocity linearVelocity_;
    marslite::move_base::Velocity angularVelocity_;
    float linearVelocityStep_;
    float angularVelocityStep_;

    // parsed parameters (launch)
    bool directControl_;
    bool messageEnabled_;
    bool autoSlowDownEnabled_;

    // robot messages
    geometry_msgs::Twist robotTwist_;
    marslite::move_base::Pose robotPose_;

    // adaptive controller
    std::shared_ptr<AdaptiveController> adaptiveControllerPtr_;

protected:
    /**
     * @brief Extract necessary parameters for teleoperation
     * @return true if all parameters were successfully parsed
    */
    bool parseParameters(void);
};

} // namespace teleoperation

} // namespace marslite_navigation



#endif // _TELEOP_INTERFACE_H_