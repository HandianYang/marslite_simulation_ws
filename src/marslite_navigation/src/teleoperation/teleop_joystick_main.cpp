#include "marslite_navigation/teleoperation/teleop_joystick.h"

using marslite_navigation::teleoperation::TeleopJoystick;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_joystick");
    std::shared_ptr<TeleopJoystick> joystickHandler = std::make_shared<TeleopJoystick>();
    
    ROS_ASSERT(joystickHandler->run());
    return 0;
}