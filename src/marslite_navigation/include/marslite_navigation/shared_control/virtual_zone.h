/**
 * @file virtual_zone.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for the virtual zone class. The class is
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

#ifndef MARSLITE_NAVIGATION_VIRTUAL_ZONE_H_
#define MARSLITE_NAVIGATION_VIRTUAL_ZONE_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

class VirtualZone {
public:
    explicit VirtualZone(const ros::NodeHandle& nh = ros::NodeHandle());
    virtual ~VirtualZone(void) = default;

    /**
     * @brief Obtain the virtual zone fields data.
     * @return The virtual zone fields data (in LaserScan format)
     */
    inline sensor_msgs::LaserScan getFieldsData(void) const { return this->fields_; }
    
protected:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher  fieldsPublisher_;
    ros::Subscriber laserScanSubscriber_;
    ros::Subscriber robotTwistSubscriber_;

    // robot properties
    geometry_msgs::Twist robotTwist_;

    // virtual zone properties
    bool firstAssign_;
    float ax_, by_;    // semi-major & semi-minor axes
    float ka_, kb_;    // proportional ratio of ax_ and by_
    sensor_msgs::LaserScan fields_;

    /**
     * @brief Calculate the virtual zone fields.
     * @param laserPtr The LaserScan data pointer (default nullptr).
     * @note This is a pure virtual function. It must be implemented in the derived class.
     */
    virtual void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) = 0;

private:
    /**
     * @brief The callback function for the LaserScan data.
     * @param laserPtr The pointer to the LaserScan data.
     */
    void laserScanCB(const sensor_msgs::LaserScanConstPtr& laserPtr);
    
    /**
     * @brief The callback function for the robot twist data.
     * @param twistPtr The pointer to the robot twist data.
     */
    void robotTwistCB(const geometry_msgs::TwistConstPtr& twistPtr);
};

} // namespace navigation

} // namespace marslite

#endif  // #ifndef MARSLITE_NAVIGATION_VIRTUAL_ZONE_H_