/**
 * @file teleop_joystick.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for the joystick teleoperation class.
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

#ifndef _TELEOP_JOYSTICK_H_
#define _TELEOP_JOYSTICK_H_

#include "marslite_navigation/teleoperation/teleop_interface.h"

#include <sensor_msgs/Joy.h>
#include <mutex>

namespace marslite_navigation {

namespace teleoperation {

const static char* TOPIC_NAME = "/unity/joy";
const static float DEADZONE_THRESHOLD = 0.3;  // value below this threshold will be ignored
const static float TRIGGER_THRESHOLD  = 0.95;

class TeleopJoystick : public TeleopInterface {
public:
    explicit TeleopJoystick(void) {}
    bool run(void) override;

private:
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
    
    // callbacks
    void joyCB(const sensor_msgs::JoyConstPtr& joyPtr);
};

} // namespace teleoperation

} // namespace marslite_navigation



#endif // _TELEOP_JOYSTICK_H_