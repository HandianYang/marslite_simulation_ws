#include "marslite_navigation/teleoperation/teleop_joystick.h"

namespace marslite_navigation {

namespace teleoperation {

bool TeleopJoystick::run(void)
{
    joySubscriber_ = nh_.subscribe("/joy", 1, &TeleopJoystick::joyCB, this);

    while (ros::ok()) {
        std::unique_lock<std::mutex> lock(joyMutex_);
        {   
            // Read the message of `/joy` topic, and translate it into the robot's velocity
            switch (axesNum_) {
            case 2:
                angularVelocity_.velocity = (-1) * joy_.axes[1] * angularVelocity_.limit.max;
            case 1:
                linearVelocity_.velocity = joy_.axes[0] * linearVelocity_.limit.max;
                break;
            default:
                ROS_WARN_ONCE("Unmatch number of joystick axes (expected: 1-2, received: %ld).", axesNum_);
                ROS_WARN_ONCE(" Please check your joystick(s) setup or rosbridge connection.");
                break;
            }
        }
        lock.unlock();

        // Print out the message (if enabled)
        if (messageEnabled_) {
            std::cout << std::fixed << std::setprecision(2)
                << "Linear velocity: " << linearVelocity_.velocity << "\t"
                << "Angular velocity: " << angularVelocity_.velocity << "\t\t\t\r";
        }

        // Publish the twist message to `/cmd_vel` topic
        robotTwist_.linear.x  = linearVelocity_.velocity;    robotTwist_.linear.y  = 0;   robotTwist_.linear.z  = 0;
        robotTwist_.angular.z = angularVelocity_.velocity;   robotTwist_.angular.x = 0;   robotTwist_.angular.y = 0;
        robotTwistPublisher_.publish(robotTwist_);

        // Delay 
        publishRate_.sleep();
        ros::spinOnce();     
    }
    
    return true;
}

void TeleopJoystick::joyCB(const sensor_msgs::JoyConstPtr& joyPtr)
{
    std::unique_lock<std::mutex> lock(joyMutex_);
    {
        axesNum_ = joyPtr->axes.size();
        buttonsNum_ = joyPtr->buttons.size();
        ROS_INFO_STREAM_ONCE(ros::this_node::getName() << " has subscribed /joy topic!");
        ROS_INFO_STREAM_ONCE("Received message contains: "
            << "\t" << axesNum_    << " axes and "
            << "\t" << buttonsNum_ << " buttons.");
        
        joy_.header = joyPtr->header;
        joy_.axes.assign(joyPtr->axes.begin(), joyPtr->axes.end());
        joy_.buttons.assign(joyPtr->buttons.begin(), joyPtr->buttons.end());
    }
}

} // namespace teleoperation

} // namespace marslite_navigation




int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_joystick");
    std::shared_ptr<marslite_navigation::teleoperation::TeleopJoystick> joystickHandler
        = std::make_shared<marslite_navigation::teleoperation::TeleopJoystick>();
    
    ROS_ASSERT(joystickHandler->run());
    return 0;
}