#ifndef _TELEOP_KEYBOARD_H_
#define _TELEOP_KEYBOARD_H_

#include "marslite_navigation/teleop_interface.h"

#include <iostream>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

namespace marslite_navigation {

class TeleopKeyboard : public TeleopInterface {
public:
    explicit TeleopKeyboard(void) { teleoperationType_ = TELEOP_KEYBOARD; }

    bool run(void) override;
    
protected:
    char inputKey_;
    const std::string userGuideMsg_ = R"(
Control Your Robot!
---------------------------
Moving around:
    w
a   s   d

w/s : increase/decrease linear velocity [)" + std::to_string(linearVelocityLimit_.front) + ", " + std::to_string(linearVelocityLimit_.back) + R"(]
a/d : increase/decrease angular velocity [)" + std::to_string(angularVelocityLimit_.left) + ", " + std::to_string(angularVelocityLimit_.right) + R"(]

space key : force stop

q to quit
)";

    bool getInput(void);
    
    void publishRobotTwistCallback(const sensor_msgs::LaserScanConstPtr& lidar);
};


} // namespace marslite_navigation

#endif // _TELEOP_KEYBOARD_H_
