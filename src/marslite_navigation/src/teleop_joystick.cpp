#include "marslite_navigation/teleop_joystick.h"

namespace marslite_navigation {

bool TeleopJoystick::run()
{
    joySubscriber_ = nh_.subscribe("/joy", 1, &TeleopJoystick::joyMsgCallback, this);

    laserScanSubscriber_ = nh_.subscribe("/scan", 1, &TeleopJoystick::publishRobotTwistCallback, this);
    
    stopNode_ = false;

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
            angularVelocity_.velocity = (-1) * joy_.axes[1] * angularVelocityLimit_.left;
        case 1:
            linearVelocity_.velocity = joy_.axes[0] * linearVelocityLimit_.front;
            break;
        default:
            ROS_WARN_ONCE("Unmatch number of joystick axes (expected: 1-2, received: %ld).", axesNum_);
            ROS_WARN_ONCE(" Please check your joystick(s) setup or rosbridge connection.");
            break;
        }
    }
    lock.unlock();

    if (assistEnabled_) {
        std::vector<float> lidar_distance;
        lidar_distance.assign(lidar->ranges.begin(), lidar->ranges.end());
        collisionAvoidance(lidar_distance);
    }
        

    if (messageEnabled_) {
		// std::cout << std::fixed << std::setprecision(2)
		// 	 << "Linear velocity: " << linearVelocity_.velocity << "\t"
		// 	 << "Angular velocity: " << angularVelocity_.velocity << "\t\t\t\r";
	}

    moveBaseTwist_.linear.x  = linearVelocity_.velocity;
	moveBaseTwist_.angular.z = angularVelocity_.velocity;
    robotTwistPublisher_.publish(moveBaseTwist_);
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