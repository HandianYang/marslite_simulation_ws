#include "marslite_shared_control/adaptive_controller.h"

#include <math.h>

namespace marslite_shared_control {

AdaptiveController::AdaptiveController(const ros::NodeHandle& nh) : nh_(nh)
{
    server_ = new dynamic_reconfigure::Server<AdaptiveControllerConfig>(ros::NodeHandle("~/marslite_shared_control/AdaptiveController"));
    f_ = boost::bind(&AdaptiveController::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);

    SVZPtr_ = std::make_shared<marslite_shared_control::StaticVirtualZone>(nh_);
    DVZPtr_ = std::make_shared<marslite_shared_control::DeformableVirtualZone>(nh_);
    amclPoseSubscriber_ = nh_.subscribe("/amcl_pose", 1, &AdaptiveController::amclPoseCB, this);
    controllerPublisher_ = nh_.advertise<geometry_msgs::Twist>("/marslite_shared_control/controller", 1);
}

void AdaptiveController::calculateIntrusionRatio()
{
    const sensor_msgs::LaserScan& SVZ_fields = SVZPtr_->getFieldsData();
    const sensor_msgs::LaserScan& DVZ_fields = DVZPtr_->getFieldsData();

    const uint16_t fieldSize = SVZ_fields.ranges.size();
    std::vector<float> IRfunc(fieldSize);   // Intrusion Ratio (IR) function

    for (uint i = 0; i < fieldSize; ++i) {
        if (abs(SVZ_fields.ranges[i]) < 1e-03)
            IRfunc[i] = 0;
        else
            IRfunc[i] = (SVZ_fields.ranges[i] - DVZ_fields.ranges[i]) / SVZ_fields.ranges[i];
    }

    // Calculate the definite integral
    intrusionRatio_ = 0;
    for (uint i = 1; i < fieldSize; ++i) {
        intrusionRatio_ += (IRfunc[i] - IRfunc[i-1]) / SVZ_fields.angle_increment;
    }
}

void AdaptiveController::calculateAverageAngle()
{
    const sensor_msgs::LaserScan& SVZ_fields = SVZPtr_->getFieldsData();
    const sensor_msgs::LaserScan& DVZ_fields = DVZPtr_->getFieldsData();

    const size_t fieldSize = SVZ_fields.ranges.size();
    std::vector<float> AOANumFunc(fieldSize);    // Average Obstacle Angle (AOA) function (numerator part)
    std::vector<float> AOADenFunc(fieldSize);    // Average Obstacle Angle (AOA) function (denominator part)
    
    for (uint i = 0; i < fieldSize; ++i) {
        AOADenFunc[i] = SVZ_fields.ranges[i] - DVZ_fields.ranges[i];
        AOANumFunc[i] = AOADenFunc[i] * marslite::THETA[i];
    }

    // Calculate the definite integral
    float AOANum = 0, AOADen = 0;     // definite integral on `AOANumFunc` and `AOADenFunc`
    for (uint i = 1; i < fieldSize; ++i) {
        AOANum += (AOANumFunc[i] - AOANumFunc[i-1]) / SVZ_fields.angle_increment;
        AOADen += (AOADenFunc[i] - AOADenFunc[i-1]) / SVZ_fields.angle_increment;
    }

    float newAOA = averageObstableAngle_;
    if (abs(AOADen) >= 1e-03) {
        newAOA = AOANum / AOADen + robotPose_.theta;

        // Fix the angle in range [-pi, pi]
        while (averageObstableAngle_ > M_PI)   averageObstableAngle_ -= 2*M_PI;
        while (averageObstableAngle_ < -M_PI)  averageObstableAngle_ += 2*M_PI;
    }
        
    averageObstableAngleDiff_ = newAOA - averageObstableAngle_;
}

void AdaptiveController::calculateController()
{
    const sensor_msgs::LaserScan& SVZ_fields = SVZPtr_->getFieldsData();
    const sensor_msgs::LaserScan& DVZ_fields = DVZPtr_->getFieldsData();

    std::vector<float> JxFunc(marslite::LASER_SIZE), JyFunc(marslite::LASER_SIZE);
    for (uint16_t i = 0; i < marslite::LASER_SIZE; ++i) {
        JxFunc[i] = (DVZ_fields.ranges[i] * cos(marslite::THETA[i]+robotPose_.theta)) / (SVZ_fields.ranges[i] * DVZ_fields.ranges[i]);
        JyFunc[i] = (DVZ_fields.ranges[i] * sin(marslite::THETA[i]+robotPose_.theta)) / (SVZ_fields.ranges[i] * DVZ_fields.ranges[i]);
    }

    float Jx = 0, Jy = 0;
    for (uint16_t i = 1; i < marslite::LASER_SIZE; ++i) {
        Jx += (JxFunc[i] - JxFunc[i-1]) / SVZ_fields.angle_increment;
        Jy += (JyFunc[i] - JyFunc[i-1]) / SVZ_fields.angle_increment;
    }

    linearController_.velocity = -kx_*Jx*cos(robotPose_.theta)-ky_*Jy*sin(robotPose_.theta);
    angularController_.velocity = -kw_*(robotPose_.theta+averageObstableAngle_) + averageObstableAngleDiff_;
}

void AdaptiveController::reconfigureCB(marslite_shared_control::AdaptiveControllerConfig& config, uint32_t level)
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

} // namespace marslite_shared_control



int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_controller");
    
    std::shared_ptr<marslite_shared_control::AdaptiveController> controllerPtr
         = std::make_shared<marslite_shared_control::AdaptiveController>();

    while (ros::ok()) {
        controllerPtr->calculateIntrusionRatio();
        controllerPtr->calculateAverageAngle();
        std::cout << std::fixed << std::setprecision(2)
                << "Intrusion ratio: " << controllerPtr->getIntrusionRatio() << "\t"
                << "Average angle: "   << controllerPtr->getAverageAngle()   << "\t\t\t\r";
        
        ros::Rate(60).sleep();
        ros::spinOnce();
    }

    return 0;
}