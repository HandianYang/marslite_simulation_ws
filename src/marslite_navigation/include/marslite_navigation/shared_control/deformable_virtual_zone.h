/**
 * @file deformable_virtual_zone.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for the deformable virtual zone (DVZ) class. The class is
 *        derived from the virtual zone class.
 * 
 * @note `deformable_virtual_zone.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_NAVIGATION_DEFORMABLE_VIRTUAL_ZONE_H_
#define MARSLITE_NAVIGATION_DEFORMABLE_VIRTUAL_ZONE_H_

#include <dynamic_reconfigure/server.h>
#include "marslite_navigation/VirtualZoneConfig.h"

#include "marslite_navigation/shared_control/virtual_zone.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

class DeformableVirtualZone : public VirtualZone {
public:
    explicit DeformableVirtualZone(const ros::NodeHandle& nh = ros::NodeHandle());
    
protected:
    /**
     * @brief Calculate the deformable virtual zone (DVZ) fields.
     * @param laserPtr The LaserScan data pointer (default nullptr).
     */
    void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) override;

private:
    // dynamic reconfigure
    void reconfigureCB(marslite_navigation::VirtualZoneConfig &config, uint32_t level);
    dynamic_reconfigure::Server<marslite_navigation::VirtualZoneConfig>* server_;
    dynamic_reconfigure::Server<marslite_navigation::VirtualZoneConfig>::CallbackType f_;
};

} // namespace navigation

} // namespace marslite

#endif  // #ifndef MARSLITE_NAVIGATION_DEFORMABLE_VIRTUAL_ZONE_H_