#include <ros/ros.h> 
#include "tm_driver_t/tm_driver.h"
#include "std_msgs/Float64MultiArray.h"

#ifdef USE_BOOST
  #include <boost/thread/condition_variable.hpp>
#else
  #include <condition_variable>
#endif

#ifdef USE_BOOST
  boost::condition_variable data_cv;
  boost::condition_variable data_cv_rt;
#else
  std::condition_variable data_cv;
  std::condition_variable data_cv_rt;
#endif

TmDriver TmRobot(data_cv, data_cv_rt, "195.168.0.5", 0);

void callback(const std_msgs::Float64MultiArray::ConstPtr& cmd)
{
    static bool servoOpened = false;
    std::vector<double> vec;

    for(int i=0; i<6; i++)
    {
        vec.push_back(cmd->data[i]);
    }

    if(!servoOpened && (vec[0] > 0.000001 || vec[0] < -0.000001 || vec[1] > 0.000001 || vec[1] < -0.000001 || vec[2] > 0.000001 || vec[2] < -0.000001 || vec[3] > 0.000001 || vec[3] < -0.000001 || vec[4] > 0.000001 || vec[4] < -0.000001 || vec[5] > 0.000001 || vec[5] < -0.000001))
    {
	TmRobot.setServoOpen("speedj");
        servoOpened = true;
        ROS_INFO("Velocity Control Mode On.");
    }

    if(servoOpened && vec[0] < 0.000001 && vec[0] > -0.000001 && vec[1] < 0.000001 && vec[1] > -0.000001 && vec[2] < 0.000001 && vec[2] > -0.000001 && vec[3] < 0.000001 && vec[3] > -0.000001 && vec[4] < 0.000001 && vec[4] > -0.000001 && vec[5] < 0.000001 && vec[5] > -0.000001)
    {
	TmRobot.setServoStop();
        servoOpened = false;
        ROS_INFO("Velocity Control Mode Off.");
    }

    if(servoOpened)
    {
        TmRobot.setSpeedj(vec);
    }

    ROS_INFO("I heard: [%f, %f, %f, %f, %f, %f]", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"velocity_control");
    ros::NodeHandle nh;

    TmRobot.interface->start();

    ros::Subscriber sub = nh.subscribe("/velocity_cmd", 1, callback);
    ros::spin();

    return 0;
}
