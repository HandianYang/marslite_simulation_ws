#include "marslite_navigation/shared_control/adaptive_controller.h"

#include <math.h>

namespace marslite_navigation {

namespace shared_control {

AdaptiveController::AdaptiveController(const ros::NodeHandle& nh) : nh_(nh)
{
    const std::string topicName = "/marslite_navigation/shared_control/controller";

    server_ = new dynamic_reconfigure::Server<marslite_navigation::AdaptiveControllerConfig>(ros::NodeHandle("~"+topicName));
    f_ = boost::bind(&AdaptiveController::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);

    SVZPtr_ = std::make_shared<StaticVirtualZone>(nh_);
    DVZPtr_ = std::make_shared<DeformableVirtualZone>(nh_);
    amclPoseSubscriber_ = nh_.subscribe("/amcl_pose", 1, &AdaptiveController::amclPoseCB, this);
    controllerPublisher_ = nh_.advertise<geometry_msgs::Twist>(topicName, 1);
}

void AdaptiveController::calculateController()
{
    const sensor_msgs::LaserScan& SVZ_fields = SVZPtr_->getFieldsData();
    const sensor_msgs::LaserScan& DVZ_fields = DVZPtr_->getFieldsData();

    if (SVZ_fields.ranges.size() != marslite::LASER_SIZE || DVZ_fields.ranges.size() != marslite::LASER_SIZE)   return;

    /* Step 1. Calculate the intrusion ratio */
    std::vector<float> IRfunc(marslite::LASER_SIZE);   // Intrusion Ratio (IR) function

    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        if (marslite::math::reachZero(SVZ_fields.ranges[i]))
            IRfunc[i] = 0;
        else
            IRfunc[i] = (SVZ_fields.ranges[i] - DVZ_fields.ranges[i]) / SVZ_fields.ranges[i];
    }
    intrusionRatio_ = marslite::math::integral(marslite::THETA, IRfunc);

    /* Step 2. Calculate the average obstacle angle */
    std::vector<float> AOANumFunc(marslite::LASER_SIZE);    // Average Obstacle Angle (AOA) function (numerator part)
    std::vector<float> AOADenFunc(marslite::LASER_SIZE);    // Average Obstacle Angle (AOA) function (denominator part)
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        AOADenFunc[i] = SVZ_fields.ranges[i] - DVZ_fields.ranges[i];
        AOANumFunc[i] = AOADenFunc[i] * marslite::THETA[i];
    }
    const float AOADen = marslite::math::integral(marslite::THETA, AOADenFunc);
    const float AOANum = marslite::math::integral(marslite::THETA, AOANumFunc);

    const float currentAOA = averageObstableAngle_;
    if (!marslite::math::reachZero(AOADen)) {
        averageObstableAngle_ = AOANum / AOADen + robotPose_.theta;
        while (averageObstableAngle_ >= M_PI)   averageObstableAngle_ -= 2*M_PI;
        while (averageObstableAngle_ <= -M_PI)  averageObstableAngle_ += 2*M_PI;
    }
    averageObstableAngleDiff_ = averageObstableAngle_ - currentAOA;
    if (averageObstableAngleDiff_ >= 1.8*M_PI)  averageObstableAngleDiff_ = 2*M_PI - averageObstableAngleDiff_;
    if (averageObstableAngleDiff_ <= -1.8*M_PI) averageObstableAngleDiff_ = 2*M_PI + averageObstableAngleDiff_;

    /* Step 3. Calculate the translational and rotational velocity controllers */
    std::vector<float> JxFunc(marslite::LASER_SIZE);        // Jx function
    std::vector<float> JyFunc(marslite::LASER_SIZE);        // Jy function
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        if (marslite::math::reachZero(SVZ_fields.ranges[i]) || marslite::math::reachZero(DVZ_fields.ranges[i])) {
            JxFunc[i] = 0;
            JyFunc[i] = 0;
        } else {
            JxFunc[i] = cos(marslite::THETA[i]+robotPose_.theta) / SVZ_fields.ranges[i];
            JyFunc[i] = sin(marslite::THETA[i]+robotPose_.theta) / SVZ_fields.ranges[i];
        }
    }
    const float Jx = marslite::math::integral(marslite::THETA, JxFunc);
    const float Jy = marslite::math::integral(marslite::THETA, JyFunc);

    linearController_.velocity = -kx_*Jx*cos(robotPose_.theta)-ky_*Jy*sin(robotPose_.theta);
    angularController_.velocity = -kw_*(robotPose_.theta+averageObstableAngle_) + averageObstableAngleDiff_;
}

void AdaptiveController::reconfigureCB(marslite_navigation::AdaptiveControllerConfig& config, uint32_t level)
{
    kx_ = config.kx;
    ky_ = config.ky;
    kw_ = config.kw;
}

void AdaptiveController::amclPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPosePtr)
{
    std::unique_lock<std::mutex> lock(amclPoseMutex_);
    {   
        double row, pitch, yaw;
        tf::Quaternion q (
            amclPosePtr->pose.pose.orientation.x,
            amclPosePtr->pose.pose.orientation.y,
            amclPosePtr->pose.pose.orientation.z,
            amclPosePtr->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(row, pitch, yaw);
        
        robotPose_.x = amclPosePtr->pose.pose.position.x;
        robotPose_.y = amclPosePtr->pose.pose.position.y;
        robotPose_.theta = yaw;
    } // lock(amclPoseMutex_)
}

} // namespace shared_control

} // namespace marslite_navigation



int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_controller");
    
    std::shared_ptr<marslite_navigation::shared_control::AdaptiveController> controllerPtr
         = std::make_shared<marslite_navigation::shared_control::AdaptiveController>();

    while (ros::ok()) {
        // controllerPtr->calculateIntrusionRatio();
        // controllerPtr->calculateAverageAngle();
        controllerPtr->calculateController();
        std::cout << std::fixed << std::setprecision(2)
                << "Intrusion ratio: " << controllerPtr->getIntrusionRatio() << "\t"
                << "Average angle: "   << controllerPtr->getAverageAngle()   << "\t\t\t\r";
        
        ros::Rate(60).sleep();
        ros::spinOnce();
    }

    return 0;
}