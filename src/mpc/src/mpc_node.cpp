#include <mpc/mpc.h>
#include <signal.h>

ModelPredictiveControl *mpc;

void sigHandler(int signum) 
{
    delete mpc;

    exit(0);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mpc");
    ros::NodeHandle nh;
    
    mpc = new ModelPredictiveControl(nh);

    signal(SIGINT, sigHandler);    

    mpc->performReplenishment();

    return 0;
}
