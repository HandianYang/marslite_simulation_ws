#ifndef _TELEOP_INTERFACE_H_
#define _TELEOP_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "marslite_navigation/move_base.h"


namespace marslite_navigation {

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
    move_base::Velocity linearVelocity_;
    move_base::Velocity angularVelocity_;
    float linearVelocityStep_;
    float angularVelocityStep_;

    // parsed parameters (launch)
    bool messageEnabled_;
    bool autoSlowDownEnabled_;

    // robot messages
    geometry_msgs::Twist robotTwist_;
    move_base::Pose robotPose_;

protected:
    /**
     * @brief Extract necessary parameters for teleoperation
     * @return true if all parameters were successfully parsed
    */
    bool parseParameters(void);
};

} // namespace marslite_navigation

#endif // _TELEOP_INTERFACE_H_