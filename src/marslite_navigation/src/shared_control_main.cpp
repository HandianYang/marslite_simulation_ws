/**
 * @file shared_control_main.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The executable of the shared control node. The node is responsible for
 *        the shared control navigation of the robot.
 * 
 * @note `shared_control_main.cpp` is part of `marslite_simulation_ws`.
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

#include "marslite_navigation/shared_control/shared_control.h"

using marslite::navigation::SharedControl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shared_control");
    
    std::shared_ptr<SharedControl> sharedController = std::make_shared<SharedControl>();

    ROS_ASSERT(sharedController->run());
    
    return 0;
}