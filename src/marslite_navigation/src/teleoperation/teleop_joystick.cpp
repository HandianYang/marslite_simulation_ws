#include "marslite_navigation/teleop_joystick.h"

namespace marslite_navigation {

bool TeleopJoystick::run()
{
    ROS_ASSERT(parseParameters());
	
	linearVelocityLimit_.front_ = constantLinearVelocityLimit_;
	linearVelocityLimit_.back_ = -constantLinearVelocityLimit_;

    robotTwistPublisher_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    joySubscriber_ = nh_.subscribe("/joy", 1, &TeleopJoystick::joyMsgCallback, this);

    laserScanSubscriber_ = nh_.subscribe("/scan", 1, &TeleopJoystick::publishRobotTwistCallback, this);
    
    ros::spin();

    return true;
}

void TeleopJoystick::joyMsgCallback(const sensor_msgs::JoyConstPtr& joy)
{
    std::unique_lock<std::mutex> lock(joyMutex_);
    {
        axesNum_ = joy->axes.size();
        buttonsNum_ = joy->buttons.size();
        ROS_INFO_STREAM_ONCE(ros::this_node::getName() << " has subscribed /joy topic!");
        ROS_INFO_STREAM_ONCE("Received message contains: "
            << "\t" << axesNum_    << " axes and "
            << "\t" << buttonsNum_ << " buttons.");
        
        joy_.header = joy->header;
        joy_.axes.assign(joy->axes.begin(), joy->axes.end());
        joy_.buttons.assign(joy->buttons.begin(), joy->buttons.end());
    }
}

void TeleopJoystick::publishRobotTwistCallback(const sensor_msgs::LaserScanConstPtr& lidar)
{
    std::unique_lock<std::mutex> lock(joyMutex_);
    {
        switch (axesNum_) {
        case 2:
            angularVelocity_.velocity_ = (-1) * joy_.axes[1] * constantAngularVelocityLimit_/2;
        case 1:
            linearVelocity_.velocity_ = joy_.axes[0] * constantLinearVelocityLimit_/2;
            break;
        default:
            ROS_WARN_ONCE("Unmatch numebrs of axes/buttons. Please check your joystick(s) or ROS_sharp setup.");
            break;
        }
    }
    lock.unlock();

    if (messageEnabled_) {
		std::cout << std::fixed << std::setprecision(2)
			 << "Linear velocity: " << linearVelocity_.velocity_ << "\t"
			 << "Angular velocity: " << angularVelocity_.velocity_ << "\t\t\t\r";
	}

    currentTwist_.linear.x  = linearVelocity_.velocity_;
	currentTwist_.angular.z = angularVelocity_.velocity_;
    robotTwistPublisher_.publish(currentTwist_);
}


} // namespace marslite_navigation

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_joystick");
    marslite_navigation::TeleopInterfacePtr joystickHandler
        = std::make_shared<marslite_navigation::TeleopJoystick>();
    
    ROS_ASSERT(joystickHandler->run());
    return 0;
}