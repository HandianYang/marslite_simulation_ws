#ifndef _TELEOP_JOYSTICK_H_
#define _TELEOP_JOYSTICK_H_

#include "marslite_navigation/teleoperation/teleop_interface.h"

#include <sensor_msgs/Joy.h>
#include <mutex>

namespace marslite_navigation {

namespace teleoperation {

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

    // mutex
    std::mutex joyMutex_;
    
    // callbacks
    void joyCB(const sensor_msgs::JoyConstPtr& joyPtr);
};

} // namespace teleoperation

} // namespace marslite_navigation



#endif // _TELEOP_JOYSTICK_H_