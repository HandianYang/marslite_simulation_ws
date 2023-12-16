#ifndef _TELEOP_JOYSTICK_H_
#define _TELEOP_JOYSTICK_H_

#include "marslite_navigation/teleop_interface.h"

#include <sensor_msgs/Joy.h>
#include <mutex>

namespace marslite_navigation {

class TeleopJoystick : public TeleopInterface {
public:
    explicit TeleopJoystick(void) { teleoperationType_ = TELEOP_JOYSTICK; }

    bool run(void) override;

protected:
    ros::Subscriber joySubscriber_;

    size_t axesNum_;
    size_t buttonsNum_;
    sensor_msgs::Joy joy_;

    // mutex
    std::mutex joyMutex_;
    
    void joyMsgCallback(const sensor_msgs::JoyConstPtr& joy);
    void publishRobotTwistCallback(const sensor_msgs::LaserScanConstPtr& lidar);
};

} // namespace marslite_navigation



#endif // _TELEOP_JOYSTICK_H_