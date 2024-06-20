/**
 * @file teleop_joystick.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for the joystick teleoperation class. The class is
 *        derived from the `TeleopInterface` class.
 * 
 * @note `teleop_joystick.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_NAVIGATION_TELEOP_JOYSTICK_H_
#define MARSLITE_NAVIGATION_TELEOP_JOYSTICK_H_

#include <sensor_msgs/Joy.h>
#include <mutex>

#include "marslite_navigation/teleoperation/teleop_interface.h"

/**
 * @namespace marslite operation namespace
 */
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

class TeleopJoystick : public TeleopInterface {
public:
    explicit TeleopJoystick(void) {}
    bool run(void) override;

private:
    const char* TOPIC_NAME = "/unity/joy";
    const float DEADZONE_THRESHOLD = 0.3;  // value below this threshold will be ignored
    const float TRIGGER_THRESHOLD  = 0.95;

    // ROS-related
    ros::Subscriber joySubscriber_;

    // joy messages
    size_t axesNum_;
    size_t buttonsNum_;
    sensor_msgs::Joy joy_;

    // flags
    bool indexTriggerIsPressed_;
    bool handTriggerIsPressed_;

    // mutex
    std::mutex joyMutex_;

private:
    /**
     * @brief Callback function for the joystick message
     * @param joyPtr the pointer to the joystick message
     */
    void joyCB(const sensor_msgs::JoyConstPtr& joyPtr);
};

} // namespace navigation

} // namespace marslite



#endif // MARSLITE_NAVIGATION_TELEOP_JOYSTICK_H_