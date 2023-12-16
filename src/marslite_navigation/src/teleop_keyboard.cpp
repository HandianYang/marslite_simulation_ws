#include "marslite_navigation/teleop_keyboard.h"

namespace marslite_navigation {

bool TeleopKeyboard::run(void)
{
    laserScanSubscriber_ = nh_.subscribe("/scan", 1, &TeleopKeyboard::publishRobotTwistCallback, this);

	stopNode_ = false;

	ROS_INFO_STREAM(userGuideMsg_);
	if (laserScanSubscriber_ ) {
		while (ros::ok() && !stopNode_) {
			ROS_ASSERT(getInput());
			
			ros::Rate(60).sleep();
			ros::spinOnce();
		}
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



void TeleopKeyboard::publishRobotTwistCallback(const sensor_msgs::LaserScanConstPtr& lidar)
{
	switch (inputKey_) {
	case 'w':
	case 'W':
		linearVelocity_.increase(linearVelocityStep_.velocity, linearVelocityLimit_.front);
		break;
	case 's':
	case 'S':
		linearVelocity_.decrease(linearVelocityStep_.velocity, linearVelocityLimit_.back);
		break;
	case 'a':
	case 'A':
		angularVelocity_.increase(angularVelocityStep_.velocity, angularVelocityLimit_.left);
		break;
	case 'd':
	case 'D':
		angularVelocity_.decrease(angularVelocityStep_.velocity, angularVelocityLimit_.right);
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
			linearVelocity_.slowDown(linearVelocityStep_.velocity);
			angularVelocity_.slowDown(angularVelocityStep_.velocity);
		}
		break;
	}

	if (messageEnabled_) {
		std::cout << std::fixed << std::setprecision(2)
			 << "Linear velocity: " << linearVelocity_.velocity << "\t"
			 << "Angular velocity: " << angularVelocity_.velocity << "\t\t\t\r";
	}

	moveBaseTwist_.linear.x  = linearVelocity_.velocity;
	moveBaseTwist_.angular.z = angularVelocity_.velocity;
	robotTwistPublisher_.publish(moveBaseTwist_);
}

} // namespace marslite_navigation


int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_keyboard");
	marslite_navigation::TeleopInterfacePtr keyboardHandler
		 = std::make_shared<marslite_navigation::TeleopKeyboard>();
	
	ROS_ASSERT(keyboardHandler->run());
	return 0;
}
