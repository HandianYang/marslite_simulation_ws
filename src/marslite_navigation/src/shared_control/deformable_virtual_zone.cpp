/**
 * @file deformable_virtual_zone.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the deformable virtual zone (DVZ) class. The class is
 *        derived from the virtual zone class.
 * 
 * @note `deformable_virtual_zone.cpp` is part of `marslite_simulation_ws`.
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

#include "marslite_navigation/shared_control/deformable_virtual_zone.h"
#include "marslite_navigation/move_base.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

DeformableVirtualZone::DeformableVirtualZone(const ros::NodeHandle& nh) : VirtualZone(nh)
{
    const std::string topicName = "/marslite_navigation/DVZ";

    server_ = new dynamic_reconfigure::Server<marslite_navigation::VirtualZoneConfig>(ros::NodeHandle("~"+topicName));
    f_ = boost::bind(&DeformableVirtualZone::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);

    fieldsPublisher_ = nh_.advertise<sensor_msgs::LaserScan>(topicName, 1);
}

void DeformableVirtualZone::calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr)
{
    ax_ = ka_ * robotTwist_.linear.x;
    by_ = kb_ * robotTwist_.linear.x;
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        fields_.ranges[i] = (float) pow((pow(ax_*cos(marslite::THETA[i]), 2) + pow(by_*sin(marslite::THETA[i]), 2)), 0.5);
        if (fields_.ranges[i] > laserPtr->ranges[i])
            fields_.ranges[i] = laserPtr->ranges[i];
    }
}

void DeformableVirtualZone::reconfigureCB(marslite_navigation::VirtualZoneConfig &config, uint32_t level)
{
    ka_ = config.ka;
    kb_ = config.kb;
}

} // namespace navigation

} // namespace marslite