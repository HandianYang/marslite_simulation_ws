/**
 * @file teleop_keyboard.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the keyboard teleoperation class. The class is
 *        derived from the `TeleopInterface` class. 
 * 
 * @note `teleop_keyboard.cpp` is part of `marslite_simulation_ws`.
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

#include "marslite_navigation/teleoperation/teleop_keyboard.h"

#include <iostream>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

/**
 * @namespace marslite operation namespace
 */
namespace marslite {

/**
 * @namespace navigation namespace
 */
namespace navigation {

bool TeleopKeyboard::run(void)
{
	// Print out the guiding message
	ROS_INFO_STREAM(userGuideMsg_);
	
	while (ros::ok() && !stopNode_) {
		// Obtain keyboard inputs
		ROS_ASSERT(getInput());

		switch (inputKey_) {
		case 'w':
		case 'W':
			// Move forward
			linearVelocity_ += linearVelocityStep_;
			break;
		case 's':
		case 'S':
			// Move backward
			linearVelocity_ -= linearVelocityStep_;
			break;
		case 'a':
		case 'A':
			// Turn left
			angularVelocity_ += angularVelocityStep_;
			break;
		case 'd':
		case 'D':
			// Turn right
			angularVelocity_ -= angularVelocityStep_;
			break;
		case ' ':
			linearVelocity_.velocity = 0;
			angularVelocity_.velocity = 0;
			break;
		case 'q':
		case 'Q':
			stopNode_ = true;
			break;
		default:
			if (autoSlowDownEnabled_) {
				linearVelocity_.slowDown(linearVelocityStep_);
				angularVelocity_.slowDown(angularVelocityStep_);
			}
			break;
		}

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

bool TeleopKeyboard::getInput(void)
{
    struct termios oldt, newt;
    inputKey_ = '\0';

    // Get the file descriptor for the standard input (keyboard)
    int fd = fileno(stdin);

    // Save the current terminal settings
    tcgetattr(fd, &oldt);
    newt = oldt;

    // Set the terminal to raw mode
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(fd, TCSANOW, &newt);

    // Set non-blocking mode on stdin
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    // Check if there is input available
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100 ms timeout

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    int selectRet = select(fd + 1, &fds, NULL, NULL, &tv);
    if (selectRet > 0 && FD_ISSET(fd, &fds))
    {
        inputKey_ = getchar(); // Read a character from the keyboard
    }

    // Restore the terminal settings
    tcsetattr(fd, TCSANOW, &oldt);

	return true;
}

} // namespace navigation

} // namespace marslite