#include "marslite_shared_control/adaptive_controller.h"

#include <math.h>

namespace marslite_shared_control {

AdaptiveController::AdaptiveController(const ros::NodeHandle& nh) : nh_(nh)
{
    SVZPtr_ = std::make_shared<marslite_shared_control::StaticVirtualZone>(nh_);
    DVZPtr_ = std::make_shared<marslite_shared_control::DeformableVirtualZone>(nh_);
    amclPoseSubscriber_ = nh_.subscribe("/amcl_pose", 1, &AdaptiveController::amclPoseCB, this);
}

void AdaptiveController::calculateIntrusionRatio()
{
    const sensor_msgs::LaserScan& SVZ_fields = SVZPtr_->getFieldsData();
    const sensor_msgs::LaserScan& DVZ_fields = DVZPtr_->getFieldsData();

    size_t fieldSize = SVZ_fields.ranges.size();
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

    size_t fieldSize = SVZ_fields.ranges.size();
    std::vector<float> AANumFunc(fieldSize);    // Average Obstacle Angle (AA) function (numerator part)
    std::vector<float> AADenFunc(fieldSize);    // Average Obstacle Angle (AA) function (denominator part)
    
    for (uint i = 0; i < fieldSize; ++i) {
        AADenFunc[i] = SVZ_fields.ranges[i] - DVZ_fields.ranges[i];
        AANumFunc[i] = AADenFunc[i] * marslite::THETA[i];
    }

    // Calculate the definite integral
    float AANum = 0, AADen = 0;     // definite integral on `AANumFunc` and `AADenFunc`
    for (uint i = 1; i < fieldSize; ++i) {
        AANum += (AANumFunc[i] - AANumFunc[i-1]) / SVZ_fields.angle_increment;
        AADen += (AADenFunc[i] - AADenFunc[i-1]) / SVZ_fields.angle_increment;
    }

    if (abs(AADen) < 1e-03)
        averageObstableAngle_ = -robotPose_.theta;      // the opposite direction of the robot
    else
        averageObstableAngle_ = (AANum / AADen + robotPose_.theta);
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