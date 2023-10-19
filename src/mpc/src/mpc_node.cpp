#include <ros/ros.h>
#include "mpc/mpc.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "mpc");
    mpc::ModelPredictiveControl mpc;
    mpc.performReplenishment();
    return 0;
}
