#include "marslite_navigation/artificial_potential_field.h"

namespace marslite_navigation {

ArtificialPotentialField::ArtificialPotentialField(const ros::NodeHandle& nh)
     : nh_(nh) {}

bool ArtificialPotentialField::run(void)
{
    ROS_ASSERT(parseParameters());

    // publisher 
    robotTwistPublisher_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // subscribers
    userControlSubscriber_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, userControlTopicName_, 1);
    laserScanSubscriber_   = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan", 1);

    // synchronizer
    sync_.reset(new apfSync(apfSyncPolicy(10), *userControlSubscriber_, *laserScanSubscriber_));
    sync_->registerCallback(boost::bind(&ArtificialPotentialField::twistMsgCallback, this, _1, _2));

    ros::spin();

    return true;
}

bool ArtificialPotentialField::parseParameters(void)
{
    ros::NodeHandle pNh("~");

    userControlTopicName_ = pNh.param<std::string>("user_control_topic_name", "/marslite_navigation/joy_ts");
    forceStopDistance_ = pNh.param<float>("force_stop_distance", 0.5);
    constantVelocityLimit_ = pNh.param<float>("constant_velocity_limit", 0.7);
    predictTimeInterval_ = pNh.param<float>("predict_time_interval", 2);
    sectorHalfRange_ = pNh.param<unsigned int>("sector_half_range", 90);

    return true;
}

void ArtificialPotentialField::twistMsgCallback(const geometry_msgs::TwistStampedConstPtr& userControl, const sensor_msgs::LaserScanConstPtr& lidar)
{
    /* TODO
     * - teleop_keyboard.py -> teleop_keyboard.cpp
     * - merge teleop_keyboard/joystick.cpp to artificial_potential_field.cpp
     * - let artificial_potential_field perform all deceleration 
     */

    // ROS_INFO_STREAM_ONCE(lidar->ranges.size());
    // ROS_INFO_STREAM(lidar->ranges.at(100));

    geometry_msgs::Twist userControlTwist = userControl->twist;

    bool frontForceStop = false;
    for (size_t i = 540; i < 900; ++i) {
        if (lidar->ranges.at(i) <= forceStopBound_) {
            frontForceStop = true;
            break;
        }
    }

    bool backForceStop = false;
    for (size_t i = 0; i < 1440; ++i) {
        if (lidar->ranges.at(i) <= forceStopBound_) {
            backForceStop = true;
            break;
        }

        if (i == 179)   i = 1260;
    }

    static int count = 0;
    if (count >= 1000) {
        ROS_INFO_COND(frontForceStop, "Limit front velocity...");
        ROS_INFO_COND(backForceStop, "Limit back velocity...");
        count = 0;
    }
    ++count;

    frontVelocity_ = (frontForceStop) ?  0.05 :  0.7;
    backVelocity_  = (backForceStop)  ? -0.05 : -0.7;

    if (userControlTwist.linear.x >= frontVelocity_)
        userControlTwist.linear.x = frontVelocity_;
    else if (userControlTwist.linear.x <= backVelocity_)
        userControlTwist.linear.x = backVelocity_;
    
    robotTwistPublisher_.publish(userControlTwist);
}

} // namespace marslite_navigation

int main(int argc, char** argv)
{
    ros::init(argc, argv, "artificial_potential_field");
    marslite_navigation::ArtificialPotentialField APF_Handler;
    APF_Handler.run();
    return 0;
}