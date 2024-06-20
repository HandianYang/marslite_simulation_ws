/**
 * @file shared_control.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for the shared control class. The class depends
 *       on the `StaticVirtualZone` and the `DeformableVirtualZone` classes.
 * 
 * @note `shared_control.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_NAVIGATION_SHARED_CONTROL_H
#define MARSLITE_NAVIGATION_SHARED_CONTROL_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <mutex>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/server.h>
#include "marslite_navigation/SharedControlConfig.h"

#include "marslite_navigation/move_base.h"
#include "marslite_navigation/shared_control/static_virtual_zone.h"
#include "marslite_navigation/shared_control/deformable_virtual_zone.h"

/**
 * @namespace marslite operation namespace
 */
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

class SharedControl {
public:;
    explicit SharedControl(const ros::NodeHandle& nh = ros::NodeHandle());
    bool run();

    /**
     * @brief Obtain the intrusion ratio (IR).
     * @return The intrusion ratio (IR) of the robot.
     */
    inline float getIntrusionRatio(void) const noexcept { return this->intrusionRatio_; }

    /**
     * @brief Obtain the average obstacle angle (AOA).
     * @return The average obstacle angle (AOA) of the robot.
     */
    inline float getAverageAngle(void) const noexcept { return this->averageObstableAngle_; }

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

    
    void userInputCB(const geometry_msgs::TwistConstPtr& userInputPtr);

    void calculateAssistiveInput();
    void calculateAllocationWeight();
};

} // namespace navigation

} // namespace marslite

#endif  // #ifndef MARSLITE_NAVIGATION_SHARED_CONTROL_H