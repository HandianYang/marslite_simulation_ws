#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>

using namespace std;

class ImuDownSampling {
public:
    explicit ImuDownSampling(const ros::NodeHandle& nh = ros::NodeHandle());
	
private:
  	ros::NodeHandle nh_;
  	ros::Subscriber sub_;
  	ros::Publisher pub_;
  	sensor_msgs::Imu imu_msg_;

    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);
};

ImuDownSampling::ImuDownSampling(const ros::NodeHandle& nh) : nh_(nh) {
    sub_ = nh_.subscribe("depth_camera_3/imu", 1, &ImuDownSampling::ImuCallback, this);
    pub_ = nh_.advertise<sensor_msgs::Imu>("depth_camera_3/imu_low_fre", 1);
}

void ImuDownSampling::ImuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    imu_msg_ = *msg;
    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.angular_velocity.y -= (-0.0012);
    imu_msg_.angular_velocity.y = round(imu_msg_.angular_velocity.y*100)/100;
    pub_.publish(imu_msg_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_downsampling");
    ImuDownSampling imuHandler;
    ros::Rate loop_rate(50);
    
    while(ros::ok()) {
    	ros::spinOnce();
    	loop_rate.sleep();
    }

    return 0;
}
