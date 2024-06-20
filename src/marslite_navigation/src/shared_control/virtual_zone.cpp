/**
 * @file virtual_zone.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the virtual zone class. The class is
 *        an abstract class that provides the basic structure for the virtual zone.
 * 
 * @note `virtual_zone.h` is part of `marslite_simulation_ws`.
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

#include "marslite_navigation/shared_control/virtual_zone.h"
#include "marslite_navigation/move_base.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

VirtualZone::VirtualZone(const ros::NodeHandle& nh) : nh_(nh), firstAssign_(true)
{
    laserScanSubscriber_ = nh_.subscribe("/scan", 1, &VirtualZone::laserScanCB, this);
    robotTwistSubscriber_ = nh_.subscribe("/cmd_vel", 1, &VirtualZone::robotTwistCB, this);
}

void VirtualZone::laserScanCB(const sensor_msgs::LaserScanConstPtr& laserPtr)
{
    if (firstAssign_) {
        fields_.header.frame_id = laserPtr->header.frame_id;
        fields_.angle_max = laserPtr->angle_max;
        fields_.angle_min = laserPtr->angle_min;
        fields_.angle_increment = laserPtr->angle_increment;
        fields_.range_max = laserPtr->range_max;
        fields_.range_min = laserPtr->range_min;
        fields_.scan_time = laserPtr->scan_time;
        fields_.time_increment = laserPtr->time_increment;
        fields_.intensities = laserPtr->intensities;
        fields_.ranges = std::vector<float> (marslite::LASER_SIZE, 0);

        firstAssign_ = false;     
    }

    fields_.header.stamp = ros::Time::now();
    calculateFields(laserPtr);
    
    fieldsPublisher_.publish(fields_);
}

void VirtualZone::robotTwistCB(const geometry_msgs::TwistConstPtr& twistPtr)
{
    robotTwist_.linear  = twistPtr->linear;
    robotTwist_.angular = twistPtr->angular;
}

} // namespace navigation

} // namespace marslite