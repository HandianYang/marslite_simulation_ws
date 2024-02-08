#include "marslite_navigation/teleop_interface.h"

#include <iostream>
#include <math.h>

namespace marslite_navigation {

TeleopInterface::TeleopInterface(const ros::NodeHandle& nh) : nh_(nh), publishRate_(ros::Rate(60))
{
    ROS_ASSERT(parseParameters());
    robotTwistPublisher_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

bool TeleopInterface::parseParameters(void)
{
    ros::NodeHandle pNh("~");

    /***** Parsed parameters (yaml) *****/
    // (maximum) linear velocity limits
    linearVelocity_.limit.max = pNh.param<float>("linear_velocity_limit_front", 0.7);
    linearVelocity_.limit.min = pNh.param<float>("linear_velocity_limit_back", -0.5);

    // (maximum) angular velocity limits
    angularVelocity_.limit.max = pNh.param<float>("angular_velocity_limit_left",   0.5);
    angularVelocity_.limit.min = pNh.param<float>("angular_velocity_limit_right", -0.5);

    // velocity step
    linearVelocityStep_  = pNh.param<float>("linear_velocity_step",  0.01);
    angularVelocityStep_ = pNh.param<float>("angular_velocity_step", 0.05);

    /***** Parsed parameters (launch) *****/
    // Determine whether the node should print out the message
    messageEnabled_ = pNh.param<bool>("message_enabled", true);

    // Determine whether the robot should gradually slow down if there's no user inputs currently
    autoSlowDownEnabled_ = pNh.param<bool>("auto_slow_down_enabled", true);

    return true;
}

} // namespace marslite_navigation