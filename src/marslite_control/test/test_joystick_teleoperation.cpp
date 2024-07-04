#include <ros/ros.h>

#include "marslite_control/marslite_control.h"
using marslite::control::MarsliteControlScheme;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_joystick_teleoperation");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    try {
        MarsliteControlScheme::ControlClassPtr controlClassPtr = std::make_shared<MarsliteControlScheme>();

        controlClassPtr->updateInitialStateFromCurrent();
        controlClassPtr->joystickTeleoperation();

        ROS_INFO_STREAM("Test completed! Return...");
    } catch (const ConstructorInitializationFailedException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }
    

    ros::shutdown();
    return 0;
}