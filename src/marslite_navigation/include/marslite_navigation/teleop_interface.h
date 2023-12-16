#ifndef _TELEOP_INTERFACE_H_
#define _TELEOP_INTERFACE_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mutex>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

namespace marslite_navigation {

namespace move_base {

/**
 * @enum marslite_navigation::move_base::Status
 * @brief Status of the robot's move base
 * @var marslite_navigation::move_base::Status::MOVE_BASE_NORMAL
 *   status with no limit on linear velocity
 * @var marslite_navigation::move_base::Status::MOVE_BASE_DECEL
 *   status when the robot enters the deceleration zone
 * @var marslite_navigation::move_base::Status::MOVE_BASE_STOP
 *   status when the robot enters the stop zone
*/
typedef enum {
    MOVE_BASE_NORMAL,
    MOVE_BASE_DECEL,
    MOVE_BASE_STOP
} Status;

#define STATUS_TO_STR(status) \
    ((status) == move_base::MOVE_BASE_NORMAL) ? "normal" : \
    ((status) == move_base::MOVE_BASE_DECEL)  ? "decel"  : \
    ((status) == move_base::MOVE_BASE_STOP)   ? "stop"   : \
    "unknown"

/**
 * @struct marslite_navigation::move_base::Pose
 * @brief Pose of the robot's move base
 * @var marslite_navigation::move_base::Pose::x
 *   
 * @var marslite_navigation::move_base::Pose::y
 *   
 * @var marslite_navigation::move_base::Pose::theta
 *   
*/
typedef struct {
    float x;
    float y;
    float theta;
} Pose;

typedef struct {
    float front;
    float back;
} LinearVelocityLimit;
using Distance = LinearVelocityLimit;

typedef struct {
    float left;
    float right;
} AngularVelocityLimit;

typedef enum {
    VELOCITY_LINEAR,
    VELOCITY_ANGULAR
} VelocityType;

typedef struct Velocity {
    VelocityType type;
    float velocity;
    
    Velocity(const VelocityType& vel_type = VELOCITY_LINEAR)
     : type(vel_type), velocity(0.0) {}

    void increase(const float& amount, const float& upperLimit) {
        velocity = (velocity + amount > upperLimit) ? upperLimit : velocity + amount;
    }

    void decrease(const float& amount, const float& lowerLimit) {
        velocity = (velocity - amount < lowerLimit) ? lowerLimit : velocity - amount;
    }

    void slowDown(const float& amount) {
        if (velocity > 1e-03 && velocity - amount > 1e-03)
            velocity = velocity - amount;
        else if (velocity < -1e-03 && velocity + amount < -1e-03)
            velocity = velocity + amount;
        else
            velocity = 0.0;
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
    explicit TeleopInterface(const ros::NodeHandle& nh = ros::NodeHandle());
    
    virtual ~TeleopInterface(void) = default;

    virtual bool run(void) = 0;

protected:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher robotTwistPublisher_;
    ros::Subscriber laserScanSubscriber_;
    ros::Subscriber amclPoseSubscriber_;

    // parsed parameters (yaml)
    move_base::LinearVelocityLimit linearVelocityLimitBound_;
    move_base::AngularVelocityLimit angularVelocityLimitBound_;
    move_base::Velocity linearVelocityStep_{move_base::VELOCITY_LINEAR};
    move_base::Velocity angularVelocityStep_{move_base::VELOCITY_ANGULAR};
    float timeDecelBound_;
    float timeStopBound_;
    unsigned int scanRange_;

    // parsed parameters (launch)
    bool assistEnabled_;
    bool messageEnabled_;
    bool autoSlowDownEnabled_;

    // status
    TeleoperationType teleoperationType_;
    geometry_msgs::Twist moveBaseTwist_;
    move_base::Pose moveBasePose_;
    move_base::Velocity linearVelocity_{move_base::VELOCITY_LINEAR};
    move_base::Velocity angularVelocity_{move_base::VELOCITY_ANGULAR};
    // move_base::Velocity refVelocity_{move_base::VELOCITY_LINEAR};
    move_base::LinearVelocityLimit linearVelocityLimit_;
    move_base::AngularVelocityLimit angularVelocityLimit_;
    move_base::Distance distance_;
    move_base::Status frontStatus_;
    move_base::Status backStatus_;
    bool stopNode_;
    bool lockState_;

    // mutex
    std::mutex amclPoseMutex_;

protected:
    /**
     * @brief Extract necessary parameters for teleoperation
     * @return true if all parameters were successfully parsed
    */
    bool parseParameters(void);

    /**
     * @brief Provide assistive teleoperation function
    */
    void collisionAvoidance(std::vector<float>& lidar_distance);

    virtual void publishRobotTwistCallback(const sensor_msgs::LaserScanConstPtr& lidar) = 0;

private:

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amcl_pose);

};

using TeleopInterfacePtr = std::shared_ptr<TeleopInterface>;

} // namespace marslite_navigation

#endif // _TELEOP_INTERFACE_H_