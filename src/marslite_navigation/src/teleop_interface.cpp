#include "marslite_navigation/teleop_interface.h"

#include <iostream>
#include <math.h>

namespace marslite_navigation {

TeleopInterface::TeleopInterface(const ros::NodeHandle& nh)
: nh_(nh), stopNode_(true), lockState_(false), frontStatus_(move_base::MOVE_BASE_NORMAL), backStatus_(move_base::MOVE_BASE_NORMAL)
{
    ROS_ASSERT(parseParameters());

    robotTwistPublisher_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // amclPoseSubscriber_ = nh_.subscribe("/amcl_pose", 1, &TeleopInterface::amclPoseCallback, this);
    

    // laserScanSubscriber_ = nh_.subscribe("/scan", 1, &TeleopInterface::publishRobotTwistCallback, this);

}

bool TeleopInterface::parseParameters(void)
{
    ros::NodeHandle pNh("~");

    /***** Parsed parameters (yaml) *****/
    // (maximum) linear velocity limits
    linearVelocityLimit_.front = linearVelocityLimitBound_.front = pNh.param<float>("linear_velocity_limit_front", 0.7);
    linearVelocityLimit_.back  = linearVelocityLimitBound_.back  = pNh.param<float>("linear_velocity_limit_back", -0.5);;

    // (maximum) angular velocity limits
    angularVelocityLimit_.left  = angularVelocityLimitBound_.left  = pNh.param<float>("angular_velocity_limit_left", 0.5);
    angularVelocityLimit_.right = angularVelocityLimitBound_.right = pNh.param<float>("angular_velocity_limit_right", -0.5);

    // velocity step
    linearVelocityStep_.velocity = pNh.param<float>("linear_velocity_step", 0.01);
    angularVelocityStep_.velocity = pNh.param<float>("angular_velocity_step", 0.05);

    // time boundary for collision avoidance
    timeDecelBound_ = pNh.param<float>("time_decel_bound", 5);
    timeStopBound_ = pNh.param<float>("time_stop_bound", 1);

    // scanning range for the front and the back orientation
    scanRange_ = pNh.param<int>("scan_range", 40);

    /***** Parsed parameters (launch) *****/
    assistEnabled_ = pNh.param<bool>("assist_enabled", true);
    messageEnabled_ = pNh.param<bool>("message_enabled", true);
    autoSlowDownEnabled_ = pNh.param<bool>("auto_slow_down_enabled", true);

    return true;
}

void TeleopInterface::collisionAvoidance(std::vector<float>& lidar_distance)
{
    /***** Linear velocity criteria *****/

    // distance detection
    distance_.front = *std::min_element(lidar_distance.begin()+LASERSCAN_FRONT-2*scanRange_, lidar_distance.end()-LASERSCAN_FRONT+2*scanRange_);
    distance_.back = MIN (
        *std::min_element(lidar_distance.begin(), lidar_distance.begin()+2*scanRange_),
        *std::min_element(lidar_distance.end()-2*scanRange_, lidar_distance.end())
    );

    // distance boundary (deceleration zone)
    const move_base::Distance distanceDecelBound {
        linearVelocityLimitBound_.front * timeDecelBound_,
        linearVelocityLimitBound_.back  * timeDecelBound_
    };
    // distance boundary (stop zone)
    const move_base::Distance distanceStopBound {
        linearVelocityLimitBound_.front * timeStopBound_,
        linearVelocityLimitBound_.back  * timeStopBound_
    };

    // linear velocity (front)
    if (distance_.front >= distanceDecelBound.front) {
        // MOVE_BASE_NORMAL: the front limit is set to the maximum 
        frontStatus_ = move_base::MOVE_BASE_NORMAL;
        linearVelocityLimit_.front = linearVelocityLimitBound_.front;
    } else if (distance_.front >= distanceStopBound.front) {
        // MOVE_BASE_DECEL: the front limit decays in exponential as the robot approaches obstacles
        frontStatus_ = move_base::MOVE_BASE_DECEL;
        linearVelocityLimit_.front = linearVelocityLimitBound_.front
            * pow(5*M_E, (distance_.front-distanceStopBound.front)/(distanceDecelBound.front-distanceStopBound.front)-1);
    } else {
        // MOVE_BASE_STOP: the front limit is set to 0
        frontStatus_ = move_base::MOVE_BASE_STOP;
        linearVelocityLimit_.front = 0;
    }
    
    // linear velocity (back)
    if (distance_.back >= distanceDecelBound.back) {
        // MOVE_BASE_NORMAL: the back limit is set to the maximum 
        backStatus_ = move_base::MOVE_BASE_NORMAL;
        linearVelocityLimit_.back = linearVelocityLimitBound_.back;
    } else if (distance_.back >= distanceStopBound.back) {
        // MOVE_BASE_DECEL: the back limit decays in exponential as the robot approaches obstacles
        backStatus_ = move_base::MOVE_BASE_DECEL;
        linearVelocityLimit_.back = linearVelocityLimitBound_.back
            * pow(5*M_E, (distance_.back-distanceStopBound.back)/(distanceDecelBound.back-distanceStopBound.back)-1);
    } else {
        // MOVE_BASE_STOP: the back limit is set to 0
        backStatus_ = move_base::MOVE_BASE_STOP;
        linearVelocityLimit_.back = 0;
    }

    // Apply linear velocity constraints
    if (linearVelocity_.velocity >= linearVelocityLimit_.front)
        linearVelocity_.velocity = linearVelocityLimit_.front;
    else if (linearVelocity_.velocity <= linearVelocityLimit_.back)
        linearVelocity_.velocity = linearVelocityLimit_.back;

    // ROS_INFO_COND(messageEnabled_, "front status: %s, back status: %s",
    //     STATUS_TO_STR(frontStatus_), STATUS_TO_STR(backStatus_));
    

    /***** Angular velocity criteria *****/
    
}

void TeleopInterface::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amcl_pose)
{
    if (stopNode_)  return;

    std::unique_lock<std::mutex> lock(amclPoseMutex_);
    {
        double row, pitch, yaw;
        tf::Quaternion q (
            amcl_pose->pose.pose.orientation.x,
            amcl_pose->pose.pose.orientation.y,
            amcl_pose->pose.pose.orientation.z,
            amcl_pose->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(row, pitch, yaw);
        
        moveBasePose_.x = amcl_pose->pose.pose.position.x;
        moveBasePose_.y = amcl_pose->pose.pose.position.y;
        moveBasePose_.theta = yaw;

        // ROS_INFO_COND(messageEnabled_, "[amcl_pose]: x=%.2lf, y=%.2lf, theta=%.2lf"
        //     , moveBasePose_.x, moveBasePose_.y, moveBasePose_.theta);
    } // lock(amclPoseMutex_)
}

} // namespace marslite_navigation