/**
 * @file teleop_keyboard.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for the keyboard teleoperation class. The class is
 *        derived from the `TeleopInterface` class. 
 * 
 * @note `teleop_keyboard.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_NAVIGATION_TELEOP_KEYBOARD_H_
#define MARSLITE_NAVIGATION_TELEOP_KEYBOARD_H_

#include "marslite_navigation/teleoperation/teleop_interface.h"

/**
 * @namespace marslite operation namespace
 */
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

class TeleopKeyboard : public TeleopInterface {
public:
    explicit TeleopKeyboard(void) : stopNode_(false) {}
    bool run(void) override;

private:
    char inputKey_;
    bool stopNode_;
    const std::string userGuideMsg_ = R"(
Control Your Robot!
---------------------------
Moving around:
    w
a   s   d

w/s : increase/decrease linear velocity [)" + std::to_string(linearVelocity_.limit.min) + ", " + std::to_string(linearVelocity_.limit.max) + R"(]
a/d : increase/decrease angular velocity [)" + std::to_string(angularVelocity_.limit.min) + ", " + std::to_string(angularVelocity_.limit.max) + R"(]

space key : force stop

q to quit
)";

private:
    /**
     * @brief Obtain keyboard inputs
     * @return true if the keyboard input was successfully fetched
    */
    bool getInput(void);
};

} // namespace navigation

} // namespace marslite



#endif // MARSLITE_NAVIGATION_TELEOP_KEYBOARD_H_
