#include "marslite_navigation/shared_control/shared_control.h"
using marslite_navigation::shared_control::SharedControl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shared_control");
    std::shared_ptr<SharedControl> sharedController = std::make_shared<SharedControl>();
    sharedController->run();
    return 0;
}