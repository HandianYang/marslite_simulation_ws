/**
 * @file teleop_interface.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the teleoperation interface class. The class is
 *        an abstract class that provides the basic structure for teleoperation.
 * 
 * @note `teleop_interface.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_NAVIGATION_TELEOP_INTERFACE_H_
#define MARSLITE_NAVIGATION_TELEOP_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "marslite_navigation/move_base.h"

/**
 * @namespace marslite operation namespace
 */
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

class TeleopInterface {
public:
    explicit TeleopInterface(const ros::NodeHandle& nh = ros::NodeHandle());
    virtual ~TeleopInterface(void) = default;
    virtual bool run(void) = 0;

protected:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher userInputPublisher_;
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

    // user input message
    geometry_msgs::Twist userInput_;

protected:
    /**
     * @brief Extract necessary parameters for teleoperation
     * @return true if all parameters were successfully parsed
    */
    bool parseParameters(void);
};

} // namespace navigation

} // namespace marslite



#endif // MARSLITE_NAVIGATION_TELEOP_INTERFACE_H_