/**
 * @file teleop_joystick.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the joystick teleoperation class.
 * 
 * @note `teleop_joystick.cpp` is part of `marslite_simulation_ws`.
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

#include "marslite_navigation/teleoperation/teleop_joystick.h"

namespace marslite_navigation {

namespace teleoperation {

bool TeleopJoystick::run(void)
{
    joySubscriber_ = nh_.subscribe(TOPIC_NAME, 1, &TeleopJoystick::joyCB, this);
    
    /* Read the message of `TOPIC_NAME` topic, and translate it into the robot's velocity */
    while (ros::ok()) {
        std::unique_lock<std::mutex> lock(joyMutex_);
        {   
            switch (axesNum_) {
            case 4:
                // [3]: primary hand trigger
                handTriggerIsPressed_ = (joy_.axes[3] > TRIGGER_THRESHOLD);
            case 3:
                // [2]: primary index trigger
                indexTriggerIsPressed_ = (joy_.axes[2] > TRIGGER_THRESHOLD);
            case 2:
                // [1]: primary thumbstick x-axis (in reverse direction)
                if (joy_.axes[1] >= DEADZONE_THRESHOLD) {
                    angularVelocity_.velocity = (joy_.axes[1] - DEADZONE_THRESHOLD) * angularVelocity_.limit.max;
                } else if (joy_.axes[1] <= -DEADZONE_THRESHOLD) {
                    angularVelocity_.velocity = (joy_.axes[1] + DEADZONE_THRESHOLD) * angularVelocity_.limit.max;
                } else {
                    angularVelocity_.velocity = 0.0;
                }
            case 1:
                // [0]: primary thumbstick y-axis
                if (joy_.axes[0] >= DEADZONE_THRESHOLD) {
                    linearVelocity_.velocity = (joy_.axes[0] - DEADZONE_THRESHOLD) * linearVelocity_.limit.max;
                } else if (joy_.axes[0] <= -DEADZONE_THRESHOLD) {
                    linearVelocity_.velocity = (joy_.axes[0] + DEADZONE_THRESHOLD) * linearVelocity_.limit.max;
                } else {
                    linearVelocity_.velocity = 0.0;
                }
                break;
            default:
                ROS_WARN_ONCE("Failed to receive \"%s\".", TOPIC_NAME);
                ROS_WARN_ONCE(" Please check your joystick(s) setup or rosbridge connection.");
                break;
            }
        }
        lock.unlock();

        // Print out the message (if enabled)
        if (messageEnabled_) {
            std::cout << std::fixed << std::setprecision(2)
                << "Linear velocity: " << linearVelocity_.velocity << "\t"
                << "Angular velocity: " << angularVelocity_.velocity << "\t\t\t\r";
        }

		// Publish the user input
        userInput_.linear.x  = linearVelocity_.velocity;
        userInput_.angular.z = angularVelocity_.velocity;
		userInputPublisher_.publish(userInput_);

        // Delay 
        publishRate_.sleep();
        ros::spinOnce();
    }
    
    return true;
}

void TeleopJoystick::joyCB(const sensor_msgs::JoyConstPtr& joyPtr)
{
    std::unique_lock<std::mutex> lock(joyMutex_);
    {
        axesNum_ = joyPtr->axes.size();
        buttonsNum_ = joyPtr->buttons.size();
        ROS_INFO_STREAM_ONCE(ros::this_node::getName() << " has subscribed /unity/joy topic!");
        ROS_INFO_STREAM_ONCE("Received message contains: "
            << "\t" << axesNum_    << " axes and "
            << "\t" << buttonsNum_ << " buttons.");
        
        joy_.header = joyPtr->header;
        joy_.axes.assign(joyPtr->axes.begin(), joyPtr->axes.end());
        joy_.buttons.assign(joyPtr->buttons.begin(), joyPtr->buttons.end());
    }
}

} // namespace teleoperation

} // namespace marslite_navigation