#ifndef _ARTIFICIAL_POTENTIAL_FIELD_H_
#define _ARTIFICIAL_POTENTIAL_FIELD_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace marslite_navigation {

typedef enum {
    MARSLITE_LASERSCAN_BACK = 0,
    MARSLITE_LASERSCAN_RIGHT = 360,
    MARSLITE_LASERSCAN_FRONT = 720,
    MARSLITE_LASERSCAN_LEFT = 1080
} MarsliteLaserScan;  // corresponding LaserScan->ranges point


namespace move_base_property {

typedef enum {
    MOVE_BASE_NORMAL,
    MOVE_BASE_DECEL,
    MOVE_BASE_STOP
} Status;

typedef struct {
    float front;
    float back;
} Distance;

using LinearVelocityLimit = Distance;

} // namespace move_base_property


class ArtificialPotentialField {
public:
    explicit ArtificialPotentialField(const ros::NodeHandle& nh = ros::NodeHandle());
    bool run(void);

private:
    // ROS-related
    ros::NodeHandle nh_;
    ros::Publisher robotTwistPublisher_;
    message_filters::Subscriber<geometry_msgs::TwistStamped>* userControlSubscriber_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* laserScanSubscriber_;

    // message_filter related
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, sensor_msgs::LaserScan> apfSyncPolicy;
    typedef message_filters::Synchronizer<apfSyncPolicy> apfSync;
    boost::shared_ptr<apfSync> sync_;

    // Parsed parameters
    std::string userControlTopicName_;
    float forceStopDistance_;
    float constantVelocityLimit_;
    float predictTimeInterval_;
    unsigned int sectorHalfRange_;

    // Robot status
    geometry_msgs::Twist robotCurrentTwist_;
    move_base_property::Distance distance_;
    move_base_property::LinearVelocityLimit linearVelocityLimit_;
    move_base_property::Status moveBaseStatus_;
    
    bool parseParameters(void);
    void twistMsgCallback(const geometry_msgs::TwistStampedConstPtr& userControl, const sensor_msgs::LaserScanConstPtr& lidar);
};

} // namespace marslite_navigation


#endif // _ARTIFICIAL_POTENTIAL_FIELD_H_