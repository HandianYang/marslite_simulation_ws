#ifndef _TELEOP_INTERFACE_H_
#define _TELEOP_INTERFACE_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace marslite_navigation {

namespace move_base {

typedef enum {
    MOVE_BASE_NORMAL,
    MOVE_BASE_DECEL,
    MOVE_BASE_STOP
} Status;

typedef enum {
    VELOCITY_LINEAR,
    VELOCITY_ANGULAR
} VelocityType;

typedef struct {
    float front_;
    float back_;
} Distance;
using LinearVelocityLimit = Distance;

typedef struct Velocity {
    VelocityType type_;
    float velocity_;
    
    Velocity(const VelocityType& type = VELOCITY_LINEAR)
     : type_(type), velocity_(0.0) {}

    void increase(const float& upperLimit, const float& amount) {
        velocity_ = (velocity_ + amount > upperLimit) ? upperLimit : velocity_ + amount;
    }

    void decrease(const float& lowerLimit, const float& amount) {
        velocity_ = (velocity_ - amount < lowerLimit) ? lowerLimit : velocity_ - amount;
    }

    void slowDown(const float& amount) {
        if (velocity_ > 1e-03 && velocity_ - amount > 1e-03)
            velocity_ = velocity_ - amount;
        else if (velocity_ < -1e-03 && velocity_ + amount < -1e-03)
            velocity_ = velocity_ + amount;
        else
            velocity_ = 0.0;
    }
} Velocity;


} // namespace move_base


typedef enum {
    TELEOP_KEYBOARD,
    TELEOP_JOYSTICK
} TeleoperationType;

typedef enum {
    LASERSCAN_BACK = 0,
    LASERSCAN_RIGHT = 360,
    LASERSCAN_FRONT = 720,
    LASERSCAN_LEFT = 1080
} LaserScanOrientation;  // corresponding LaserScan->ranges point


class TeleopInterface {
public:
    explicit TeleopInterface(const ros::NodeHandle& nh = ros::NodeHandle())
     : nh_(nh), stopNode_(false), status_(move_base::MOVE_BASE_NORMAL) {}
    
    virtual ~TeleopInterface(void) = default;

    virtual bool run(void) = 0;

protected:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher robotTwistPublisher_;
    ros::Subscriber laserScanSubscriber_;
    // std::shared_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped>> userControlSubscriber_;
    // std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laserScanSubscriber_;

    // // message_filter related
    // using syncPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, sensor_msgs::LaserScan>;
    // using synchronizer = message_filters::Synchronizer<syncPolicy>;
    // boost::shared_ptr<synchronizer> sync_;

    // parsed parameters (yaml)
    float forceStopDistance_;
    float constantLinearVelocityLimit_;
    float constantAngularVelocityLimit_;
    float constantLinearVelocityStep_;
    float constantAngularVelocityStep_;
    float predictTimeInterval_;
    unsigned int sectorHalfRange_;

    // parsed parameters (launch)
    bool useAPF_;
    bool messageEnabled_;
    bool autoSlowDownEnabled_;

    // status
    TeleoperationType teleoperationType_;
    geometry_msgs::Twist currentTwist_;
    move_base::Velocity linearVelocity_{move_base::VELOCITY_LINEAR};
    move_base::Velocity angularVelocity_{move_base::VELOCITY_ANGULAR};
    move_base::Distance distance_;
    move_base::LinearVelocityLimit linearVelocityLimit_;
    move_base::Status status_;
    bool stopNode_;

    bool parseParameters(void);

    virtual void publishRobotTwistCallback(const sensor_msgs::LaserScanConstPtr& lidar) = 0;
};

using TeleopInterfacePtr = std::shared_ptr<TeleopInterface>;

} // namespace marslite_navigation

#endif // _TELEOP_INTERFACE_H_