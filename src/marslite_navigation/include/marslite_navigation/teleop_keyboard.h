#ifndef _TELEOP_KEYBOARD_H_
#define _TELEOP_KEYBOARD_H_

#include "marslite_navigation/teleop_interface.h"

namespace marslite_navigation {

class TeleopKeyboard : public TeleopInterface {
public:
    explicit TeleopKeyboard(void) : stopNode_(false) {}
    bool run(void) override;

private:
    // display message
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

    // keyboard inputs
    char inputKey_;

    /**
     * @brief Obtain keyboard inputs
     * @return true if the keyboard input was successfully fetched
    */
    bool getInput(void);

    // flags
    bool stopNode_;
};


} // namespace marslite_navigation

#endif // _TELEOP_KEYBOARD_H_
