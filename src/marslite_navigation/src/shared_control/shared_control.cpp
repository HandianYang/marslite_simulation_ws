/**
 * @file shared_control.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the shared control class. The class depends
 *       on the `StaticVirtualZone` and the `DeformableVirtualZone` classes.
 * 
 * @note `shared_control.cpp` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 *  with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>.
*/

#include <math.h>

#include "marslite_navigation/shared_control/shared_control.h"

/**
 * @namespace marslite operation namespace
 */
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

SharedControl::SharedControl(const ros::NodeHandle& nh) : nh_(nh), publishRate_(ros::Rate(60))
{
    const std::string topicName = "/marslite_navigation/shared_controller";

    server_ = new dynamic_reconfigure::Server<marslite_navigation::SharedControlConfig>(ros::NodeHandle("~"+topicName));
    f_ = boost::bind(&SharedControl::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);
    
    assistiveInputPublisher_ = nh_.advertise<geometry_msgs::Twist>("/marslite_navigation/assistive_input", 1);
    sharedControllerPublisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    amclPoseSubscriber_ = nh_.subscribe("/amcl_pose", 1, &SharedControl::amclPoseCB, this);
    userInputSubscriber_ = nh_.subscribe("/marslite_navigation/user_input", 1, &SharedControl::userInputCB, this);

    SVZPtr_ = std::make_shared<StaticVirtualZone>(nh_);
    DVZPtr_ = std::make_shared<DeformableVirtualZone>(nh_);
}

bool SharedControl::run()
{
    while (ros::ok()) {
        this->calculateAssistiveInput();
        this->calculateAllocationWeight();
        sharedController_.linear.x = userInput_.linear.x * allocationWeight_ + assistiveInput_.linear.x * (1 - allocationWeight_);
        sharedController_.angular.z = userInput_.angular.z * allocationWeight_ + assistiveInput_.angular.z * (1 - allocationWeight_);
        
        assistiveInputPublisher_.publish(assistiveInput_);
        sharedControllerPublisher_.publish(sharedController_);

        publishRate_.sleep();
        ros::spinOnce();
    }

    return true;
}

void SharedControl::calculateAssistiveInput()
{
    const sensor_msgs::LaserScan& SVZ_fields = SVZPtr_->getFieldsData();
    const sensor_msgs::LaserScan& DVZ_fields = DVZPtr_->getFieldsData();

    if (SVZ_fields.ranges.size() != marslite::LASER_SIZE || DVZ_fields.ranges.size() != marslite::LASER_SIZE)   return;

    /* Step 1. Calculate the intrusion ratio */
    std::vector<float> IRfunc(marslite::LASER_SIZE);   // Intrusion Ratio (IR) function

    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        if (marslite::math::reachZero(SVZ_fields.ranges[i]))
            IRfunc[i] = 0;
        else
            IRfunc[i] = (SVZ_fields.ranges[i] - DVZ_fields.ranges[i]) / SVZ_fields.ranges[i];
    }
    intrusionRatio_ = marslite::math::integral(marslite::THETA, IRfunc);

    /* Step 2. Calculate the average obstacle angle */
    std::vector<float> AOANumFunc(marslite::LASER_SIZE);    // Average Obstacle Angle (AOA) function (numerator part)
    std::vector<float> AOADenFunc(marslite::LASER_SIZE);    // Average Obstacle Angle (AOA) function (denominator part)
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        AOADenFunc[i] = SVZ_fields.ranges[i] - DVZ_fields.ranges[i];
        AOANumFunc[i] = AOADenFunc[i] * marslite::THETA[i];
    }
    const float AOADen = marslite::math::integral(marslite::THETA, AOADenFunc);
    const float AOANum = marslite::math::integral(marslite::THETA, AOANumFunc);

    const float currentAOA = averageObstableAngle_;
    if (!marslite::math::reachZero(AOADen)) {
        averageObstableAngle_ = AOANum / AOADen + robotPose_.theta;
        while (averageObstableAngle_ >= M_PI)   averageObstableAngle_ -= 2*M_PI;
        while (averageObstableAngle_ <= -M_PI)  averageObstableAngle_ += 2*M_PI;
    }
    averageObstableAngleDiff_ = averageObstableAngle_ - currentAOA;
    if (averageObstableAngleDiff_ >= 1.8*M_PI)  averageObstableAngleDiff_ = 2*M_PI - averageObstableAngleDiff_;
    if (averageObstableAngleDiff_ <= -1.8*M_PI) averageObstableAngleDiff_ = 2*M_PI + averageObstableAngleDiff_;

    /* Step 3. Calculate the translational and rotational velocity controllers */
    std::vector<float> JxFunc(marslite::LASER_SIZE);        // Jx function
    std::vector<float> JyFunc(marslite::LASER_SIZE);        // Jy function
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        if (marslite::math::reachZero(SVZ_fields.ranges[i]) || marslite::math::reachZero(DVZ_fields.ranges[i])) {
            JxFunc[i] = 0;
            JyFunc[i] = 0;
        } else {
            JxFunc[i] = cos(marslite::THETA[i]+robotPose_.theta) / SVZ_fields.ranges[i];
            JyFunc[i] = sin(marslite::THETA[i]+robotPose_.theta) / SVZ_fields.ranges[i];
        }
    }
    const float Jx = marslite::math::integral(marslite::THETA, JxFunc);
    const float Jy = marslite::math::integral(marslite::THETA, JyFunc);

    assistiveInput_.linear.x  = -kx_*Jx*cos(robotPose_.theta) - ky_*Jy*sin(robotPose_.theta);
    assistiveInput_.angular.z = -kw_*(robotPose_.theta + averageObstableAngle_) + averageObstableAngleDiff_;
}

void SharedControl::calculateAllocationWeight()
{
    // TODO: implement the allocation weight calculation
    allocationWeight_ = 0.5;
}

void SharedControl::reconfigureCB(marslite_navigation::SharedControlConfig& config, uint32_t level)
{
    kx_ = config.kx;
    ky_ = config.ky;
    kw_ = config.kw;
}

void SharedControl::amclPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPosePtr)
{
    std::unique_lock<std::mutex> lock(amclPoseMutex_);
    {   
        double row, pitch, yaw;
        tf::Quaternion q (
            amclPosePtr->pose.pose.orientation.x,
            amclPosePtr->pose.pose.orientation.y,
            amclPosePtr->pose.pose.orientation.z,
            amclPosePtr->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(row, pitch, yaw);
        
        robotPose_.x = amclPosePtr->pose.pose.position.x;
        robotPose_.y = amclPosePtr->pose.pose.position.y;
        robotPose_.theta = yaw;
    } // lock(amclPoseMutex_)
}

void SharedControl::userInputCB(const geometry_msgs::TwistConstPtr& userInputPtr)
{
    userInput_ = *userInputPtr;
}

} // namespace navigation

} // namespace marslite