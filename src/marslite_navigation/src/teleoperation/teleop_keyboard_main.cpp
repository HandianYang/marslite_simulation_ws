#include "marslite_navigation/teleoperation/teleop_keyboard.h"

using marslite_navigation::teleoperation::TeleopKeyboard;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_keyboard");
	std::shared_ptr<TeleopKeyboard> keyboardHandler = std::make_shared<TeleopKeyboard>();
	
	ROS_ASSERT(keyboardHandler->run());
	return 0;
}
